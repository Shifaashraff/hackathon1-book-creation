#!/usr/bin/env python3

"""
Isaac ROS Navigation Interface Node
ROS 2 node that interfaces between Isaac ROS perception and Nav2 navigation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu, LaserScan
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from std_msgs.msg import Header, String, Bool
from builtin_interfaces.msg import Time
from tf2_ros import TransformListener, Buffer

from nav2_msgs.action import NavigateToPose, ComputePathToPose
from geometry_msgs.msg import PoseStamped
import action_msgs.msg

import numpy as np
from typing import List, Optional, Dict, Any
import threading
import time
from collections import deque


class Nav2InterfaceNode(Node):
    """
    ROS 2 Node for interfacing between Isaac ROS perception and Nav2 navigation
    Handles perception data and converts it to navigation commands
    """

    def __init__(self):
        super().__init__('nav2_interface_node')

        # Declare parameters
        self.declare_parameter('robot_name', 'humanoid_robot')
        self.declare_parameter('namespace', 'humanoid')
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('navigation_frequency', 10.0)  # Hz
        self.declare_parameter('enable_perception_fusion', True)
        self.declare_parameter('perception_topic', '/perception_data')
        self.declare_parameter('vslam_topic', '/visual_slam/poses')
        self.declare_parameter('detection_topic', '/detectnet/detections')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.namespace = self.get_parameter('namespace').value
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.navigation_frequency = self.get_parameter('navigation_frequency').value
        self.enable_perception_fusion = self.get_parameter('enable_perception_fusion').value
        self.perception_topic = self.get_parameter('perception_topic').value
        self.vslam_topic = self.get_parameter('vslam_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # Initialize perception data buffers
        self.perception_data = None
        self.vslam_pose = None
        self.detections = []
        self.odom_data = None
        self.last_perception_time = None

        # Initialize navigation state
        self.current_goal = None
        self.navigation_active = False
        self.perception_fusion_active = True

        # Create QoS profiles
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        cmd_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Create subscribers for perception data
        self.perception_sub = self.create_subscription(
            'isaac_ros_messages/msg/PerceptionData',
            self.perception_topic,
            self.perception_callback,
            sensor_qos
        )

        self.vslam_sub = self.create_subscription(
            'isaac_ros_messages/msg/VslamResults',
            self.vslam_topic,
            self.vslam_callback,
            sensor_qos
        )

        self.detection_sub = self.create_subscription(
            'isaac_ros_messages/msg/ObjectDetection2D',
            self.detection_topic,
            self.detection_callback,
            sensor_qos
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            cmd_qos
        )

        # Create publishers for navigation commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        # Initialize navigation action clients
        self.nav_to_pose_client = rclpy.action.ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.compute_path_client = rclpy.action.ActionClient(
            self,
            ComputePathToPose,
            'compute_path_to_pose'
        )

        # Initialize TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize processing timer
        self.navigation_timer = self.create_timer(
            1.0 / self.navigation_frequency,
            self.process_navigation_with_perception
        )

        # Initialize internal state
        self.processing_lock = threading.Lock()
        self.is_processing = False

        # Initialize perception fusion modules
        self.initialize_perception_fusion()

        self.get_logger().info(f'Nav2 Interface Node initialized for {self.robot_name}')
        self.get_logger().info(f'Navigation frequency: {self.navigation_frequency} Hz')
        self.get_logger().info(f'Perception fusion enabled: {self.enable_perception_fusion}')

    def initialize_perception_fusion(self):
        """
        Initialize perception fusion modules for navigation
        """
        self.get_logger().info('Initializing perception fusion modules...')

        # Initialize perception fusion if enabled
        if self.enable_perception_fusion:
            self.get_logger().info('Initializing perception fusion for navigation')
            self.perception_fusion_module = {
                'initialized': True,
                'fusion_active': True,
                'confidence_threshold': 0.7,
                'obstacle_detection_active': True,
                'semantic_mapping_active': True
            }

    def perception_callback(self, msg):
        """
        Callback for perception data

        Args:
            msg (PerceptionData): Perception data message
        """
        with self.processing_lock:
            try:
                self.perception_data = msg
                self.last_perception_time = msg.header.stamp
            except Exception as e:
                self.get_logger().error(f'Error processing perception data: {e}')

    def vslam_callback(self, msg):
        """
        Callback for VSLAM results

        Args:
            msg (VslamResults): VSLAM results message
        """
        with self.processing_lock:
            try:
                self.vslam_pose = msg
            except Exception as e:
                self.get_logger().error(f'Error processing VSLAM data: {e}')

    def detection_callback(self, msg):
        """
        Callback for object detections

        Args:
            msg (ObjectDetection2D): Object detection message
        """
        with self.processing_lock:
            try:
                # Add detection to the list (limit to recent detections)
                if len(self.detections) > 50:  # Keep only last 50 detections
                    self.detections.pop(0)
                self.detections.append(msg)
            except Exception as e:
                self.get_logger().error(f'Error processing detection: {e}')

    def odom_callback(self, msg):
        """
        Callback for odometry data

        Args:
            msg (Odometry): Odometry message
        """
        with self.processing_lock:
            self.odom_data = msg

    def process_navigation_with_perception(self):
        """
        Main navigation processing function that integrates perception data
        Called at the specified navigation frequency
        """
        if self.is_processing:
            return  # Skip if already processing

        with self.processing_lock:
            self.is_processing = True

        try:
            # Check if we have required data
            if self.odom_data is None:
                self.get_logger().debug('Waiting for odometry data...')
                return

            # Process perception fusion if enabled
            if self.enable_perception_fusion and self.perception_fusion_active:
                self.process_perception_fusion()

            # Update navigation based on perception data
            self.update_navigation_with_perception()

        except Exception as e:
            self.get_logger().error(f'Error in navigation processing: {e}')
        finally:
            with self.processing_lock:
                self.is_processing = False

    def process_perception_fusion(self):
        """
        Process perception fusion for navigation
        """
        try:
            # Process obstacle detection from perception data
            if self.perception_data is not None:
                obstacles = self.extract_obstacles_from_perception()
                if obstacles:
                    # Update navigation costmap with detected obstacles
                    self.update_navigation_costmap(obstacles)

            # Process semantic information for navigation
            if self.perception_data is not None and self.perception_data.segmentation_image:
                semantic_info = self.extract_semantic_info_from_segmentation()
                if semantic_info:
                    # Use semantic information for navigation decisions
                    self.use_semantic_navigation(semantic_info)

        except Exception as e:
            self.get_logger().error(f'Error in perception fusion: {e}')

    def extract_obstacles_from_perception(self):
        """
        Extract obstacles from perception data

        Returns:
            List: List of detected obstacles with positions and properties
        """
        obstacles = []

        # Extract obstacles from detections
        for detection in self.detections:
            if detection.confidence > self.perception_fusion_module['confidence_threshold']:
                # Convert detection to obstacle representation
                obstacle = {
                    'position': self.convert_detection_to_world_frame(detection),
                    'confidence': detection.confidence,
                    'class_name': detection.class_name,
                    'bbox': {
                        'x_min': detection.x_min,
                        'y_min': detection.y_min,
                        'x_max': detection.x_max,
                        'y_max': detection.y_max
                    }
                }
                obstacles.append(obstacle)

        return obstacles

    def convert_detection_to_world_frame(self, detection):
        """
        Convert detection from camera frame to world frame
        This is a simplified implementation - in reality, this would require
        pose estimation and geometric calculations

        Args:
            detection: Detection object

        Returns:
            dict: World frame position
        """
        # Simplified conversion - in a real implementation, this would use
        # camera pose, intrinsic parameters, and geometric calculations
        return {
            'x': 0.0,  # Placeholder
            'y': 0.0,  # Placeholder
            'z': 0.0   # Placeholder
        }

    def extract_semantic_info_from_segmentation(self):
        """
        Extract semantic information from segmentation data

        Returns:
            dict: Semantic information for navigation
        """
        # Simplified implementation - in reality, this would process
        # the segmentation image to extract semantic regions
        return {
            'traversable_areas': [],
            'obstacle_regions': [],
            'semantic_classes': []
        }

    def use_semantic_navigation(self, semantic_info):
        """
        Use semantic information for navigation decisions

        Args:
            semantic_info (dict): Semantic information from perception
        """
        # Use semantic information to make navigation decisions
        # e.g., prefer paths through certain semantic classes
        pass

    def update_navigation_costmap(self, obstacles):
        """
        Update navigation costmap with detected obstacles

        Args:
            obstacles (List): List of detected obstacles
        """
        # In a real implementation, this would update the Nav2 costmap
        # with the detected obstacles
        if obstacles:
            self.get_logger().debug(f'Updating costmap with {len(obstacles)} obstacles')

    def update_navigation_with_perception(self):
        """
        Update navigation behavior based on perception data
        """
        try:
            # Check if we have a navigation goal
            if self.current_goal is not None:
                # Use perception data to improve navigation
                self.adjust_navigation_for_perceived_environment()

        except Exception as e:
            self.get_logger().error(f'Error updating navigation with perception: {e}')

    def adjust_navigation_for_perceived_environment(self):
        """
        Adjust navigation based on the perceived environment
        """
        # Adjust navigation plan based on real-time perception data
        # This could include:
        # - Dynamic obstacle avoidance
        # - Path replanning based on detected obstacles
        # - Semantic navigation (preferring certain surface types)
        pass

    def send_navigation_goal(self, pose_stamped: PoseStamped):
        """
        Send navigation goal to Nav2

        Args:
            pose_stamped (PoseStamped): Goal pose
        """
        try:
            # Wait for the action server to be available
            if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error('Navigation action server not available')
                return False

            # Create the goal message
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose_stamped

            # Send the goal
            future = self.nav_to_pose_client.send_goal_async(
                goal_msg,
                feedback_callback=self.navigation_feedback_callback
            )

            future.add_done_callback(self.navigation_goal_response_callback)
            self.current_goal = pose_stamped
            self.navigation_active = True

            return True
        except Exception as e:
            self.get_logger().error(f'Error sending navigation goal: {e}')
            return False

    def navigation_goal_response_callback(self, future):
        """
        Callback for navigation goal response
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Navigation goal rejected')
                self.navigation_active = False
                return

            self.get_logger().info('Navigation goal accepted')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.navigation_result_callback)
        except Exception as e:
            self.get_logger().error(f'Error in navigation goal response: {e}')

    def navigation_result_callback(self, future):
        """
        Callback for navigation result
        """
        try:
            result = future.result().result
            self.get_logger().info(f'Navigation result: {result}')
            self.navigation_active = False
        except Exception as e:
            self.get_logger().error(f'Error in navigation result: {e}')

    def navigation_feedback_callback(self, feedback_msg):
        """
        Callback for navigation feedback
        """
        try:
            feedback = feedback_msg.feedback
            self.get_logger().debug(f'Navigation feedback: {feedback}')
        except Exception as e:
            self.get_logger().error(f'Error in navigation feedback: {e}')

    def get_navigation_state(self) -> Dict[str, Any]:
        """
        Get current navigation state

        Returns:
            Dict: Current navigation state
        """
        return {
            'navigation_active': self.navigation_active,
            'current_goal': self.current_goal,
            'perception_fusion_active': self.perception_fusion_active,
            'perception_data_available': self.perception_data is not None,
            'vslam_available': self.vslam_pose is not None,
            'detection_count': len(self.detections)
        }


def main(args=None):
    """
    Main function to run the Nav2 Interface Node
    """
    rclpy.init(args=args)

    node = Nav2InterfaceNode()

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()