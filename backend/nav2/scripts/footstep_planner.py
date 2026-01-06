#!/usr/bin/env python3

"""
Footstep Planner for Bipedal Humanoid Navigation
This script implements a footstep planner specifically for bipedal humanoid navigation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Pose, Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
from typing import List, Tuple, Optional
import threading


class FootstepPlanner(Node):
    """
    Footstep planner for bipedal humanoid navigation
    Plans safe and stable footsteps for humanoid robots
    """

    def __init__(self):
        super().__init__('footstep_planner')

        # Declare parameters
        self.declare_parameter('robot_name', 'humanoid_robot')
        self.declare_parameter('namespace', 'humanoid')
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('planning_frequency', 10.0)  # Hz
        self.declare_parameter('step_length', 0.3)  # meters
        self.declare_parameter('step_width', 0.2)   # meters (distance between feet)
        self.declare_parameter('max_step_height', 0.2)  # meters
        self.declare_parameter('min_step_width', 0.1)   # minimum distance between feet
        self.declare_parameter('max_step_angle', 0.3)   # radians
        self.declare_parameter('foot_size', 0.15)       # foot size for collision checking

        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.namespace = self.get_parameter('namespace').value
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.planning_frequency = self.get_parameter('planning_frequency').value
        self.step_length = self.get_parameter('step_length').value
        self.step_width = self.get_parameter('step_width').value
        self.max_step_height = self.get_parameter('max_step_height').value
        self.min_step_width = self.get_parameter('min_step_width').value
        self.max_step_angle = self.get_parameter('max_step_angle').value
        self.foot_size = self.get_parameter('foot_size').value

        # Initialize robot state
        self.left_foot_pose = None
        self.right_foot_pose = None
        self.robot_pose = None
        self.current_path = None
        self.footstep_sequence = []
        self.support_polygon = None

        # Initialize planning state
        self.planning_active = False
        self.path_received = False

        # Create QoS profiles
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Create subscribers
        # Note: In a real implementation, these would subscribe to actual topics
        # For this simulation, we'll generate dummy data

        # Create publishers
        self.footstep_path_pub = self.create_publisher(
            Path,
            'footstep_path',
            qos
        )

        self.visualization_pub = self.create_publisher(
            MarkerArray,
            'footstep_visualization',
            qos
        )

        # Initialize footstep planning timer
        self.planning_timer = self.create_timer(
            1.0 / self.planning_frequency,
            self.plan_footsteps
        )

        # Initialize internal state
        self.processing_lock = threading.Lock()
        self.is_processing = False

        # Initialize footstep planning parameters
        self.initialize_footstep_parameters()

        self.get_logger().info(f'Footstep Planner initialized for {self.robot_name}')
        self.get_logger().info(f'Planning frequency: {self.planning_frequency} Hz')
        self.get_logger().info(f'Step length: {self.step_length}m, Step width: {self.step_width}m')

    def initialize_footstep_parameters(self):
        """
        Initialize footstep planning parameters
        """
        self.get_logger().info('Initializing footstep planning parameters...')

        # Bipedal-specific parameters
        self.foot_parameters = {
            'length': 0.15,  # meters
            'width': 0.08,   # meters
            'height': 0.02,  # meters
            'center_offset': 0.05  # offset from ankle to foot center
        }

        # Gait parameters
        self.gait_parameters = {
            'step_height': 0.05,  # clearance during swing phase
            'step_duration': 0.8,  # seconds per step
            'stance_duration': 0.4,  # time in stance phase
            'swing_duration': 0.4,   # time in swing phase
            'double_support_ratio': 0.2  # ratio of double support phase
        }

        # Stability parameters
        self.stability_parameters = {
            'zmp_margin': 0.05,  # ZMP safety margin
            'com_height': 0.8,   # Center of mass height
            'com_tolerance': 0.05,  # COM position tolerance
            'ankle_roll_gain': 0.2,  # Ankle roll for lateral balance
            'hip_sway_gain': 0.1     # Hip sway for balance
        }

        self.get_logger().info('Footstep parameters initialized')

    def plan_footsteps(self):
        """
        Main footstep planning function
        Called at the specified planning frequency
        """
        if self.is_processing:
            return  # Skip if already processing

        with self.processing_lock:
            self.is_processing = True

        try:
            # Generate dummy footstep plan for demonstration
            if self.path_received or True:  # Always generate for demo
                self.generate_footstep_sequence()

                # Publish the footstep path
                self.publish_footstep_path()

                # Publish visualization markers
                self.publish_visualization()

        except Exception as e:
            self.get_logger().error(f'Error in footstep planning: {e}')
        finally:
            with self.processing_lock:
                self.is_processing = False

    def generate_footstep_sequence(self):
        """
        Generate a sequence of footsteps based on the navigation path
        This is a simplified implementation for demonstration
        """
        try:
            # Initialize foot poses if not set
            if self.left_foot_pose is None:
                self.left_foot_pose = self.create_initial_foot_pose(-self.step_width/2, 0, 0)
            if self.right_foot_pose is None:
                self.right_foot_pose = self.create_initial_foot_pose(self.step_width/2, 0, 0)

            # Generate footsteps along a straight path for demonstration
            # In a real implementation, this would follow the actual navigation path
            self.footstep_sequence = []

            # Generate a sequence of steps
            num_steps = 10  # For demonstration
            for i in range(num_steps):
                # Alternate between left and right foot
                if i % 2 == 0:  # Left foot step
                    step_pose = self.calculate_next_foot_pose(
                        self.left_foot_pose,
                        self.right_foot_pose,
                        i
                    )
                    self.footstep_sequence.append(('left', step_pose))
                    self.left_foot_pose = step_pose
                else:  # Right foot step
                    step_pose = self.calculate_next_foot_pose(
                        self.right_foot_pose,
                        self.left_foot_pose,
                        i
                    )
                    self.footstep_sequence.append(('right', step_pose))
                    self.right_foot_pose = step_pose

            self.get_logger().debug(f'Generated {len(self.footstep_sequence)} footsteps')

        except Exception as e:
            self.get_logger().error(f'Error generating footstep sequence: {e}')

    def create_initial_foot_pose(self, x_offset: float, y_offset: float, z_offset: float) -> Pose:
        """
        Create an initial foot pose with specified offsets

        Args:
            x_offset: X offset from robot center
            y_offset: Y offset from robot center
            z_offset: Z offset from robot center

        Returns:
            Pose: Initial foot pose
        """
        pose = Pose()
        pose.position.x = x_offset
        pose.position.y = y_offset
        pose.position.z = z_offset
        pose.orientation.w = 1.0  # Identity quaternion
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        return pose

    def calculate_next_foot_pose(self, current_foot_pose: Pose, other_foot_pose: Pose, step_index: int) -> Pose:
        """
        Calculate the next foot pose based on current poses and step index

        Args:
            current_foot_pose: Current pose of the foot being moved
            other_foot_pose: Pose of the supporting foot
            step_index: Index of the current step

        Returns:
            Pose: Next foot pose
        """
        next_pose = Pose()

        # Calculate forward step
        step_distance = self.step_length
        step_angle = 0.0  # For straight line walking

        # Calculate new position based on current support foot and walking direction
        # This is a simplified model - in reality, this would consider balance, obstacles, etc.
        next_pose.position.x = other_foot_pose.position.x + step_distance * math.cos(step_angle)
        next_pose.position.y = other_foot_pose.position.y + step_distance * math.sin(step_angle)
        next_pose.position.z = 0.0  # Ground level

        # Add slight lateral offset to maintain balance
        if step_index % 2 == 0:  # Left foot step
            next_pose.position.y -= self.step_width / 2
        else:  # Right foot step
            next_pose.position.y += self.step_width / 2

        # Set orientation (facing forward for straight line walking)
        next_pose.orientation.w = math.cos(step_angle / 2)
        next_pose.orientation.z = math.sin(step_angle / 2)
        next_pose.orientation.x = 0.0
        next_pose.orientation.y = 0.0

        return next_pose

    def publish_footstep_path(self):
        """
        Publish the planned footstep path
        """
        try:
            path_msg = Path()
            path_msg.header = Header()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = f'{self.namespace}/map'

            # Add footsteps to path
            for foot, pose in self.footstep_sequence:
                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose = pose
                path_msg.poses.append(pose_stamped)

            self.footstep_path_pub.publish(path_msg)
            self.get_logger().debug(f'Published footstep path with {len(path_msg.poses)} steps')

        except Exception as e:
            self.get_logger().error(f'Error publishing footstep path: {e}')

    def publish_visualization(self):
        """
        Publish visualization markers for the footsteps
        """
        try:
            marker_array = MarkerArray()

            # Create markers for each footstep
            for i, (foot, pose) in enumerate(self.footstep_sequence):
                # Create a marker for the foot
                marker = Marker()
                marker.header = Header()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = f'{self.namespace}/map'
                marker.ns = "footsteps"
                marker.id = i
                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                # Set position
                marker.pose = pose
                # Adjust Z position slightly above ground for visibility
                marker.pose.position.z = 0.02

                # Set size (foot dimensions)
                marker.scale.x = self.foot_parameters['length']
                marker.scale.y = self.foot_parameters['width']
                marker.scale.z = 0.01  # Thin for visibility

                # Set color based on foot
                if foot == 'left':
                    marker.color.r = 1.0  # Red for left foot
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                else:  # right foot
                    marker.color.r = 0.0  # Blue for right foot
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                marker.color.a = 0.8  # Alpha

                marker_array.markers.append(marker)

            self.visualization_pub.publish(marker_array)
            self.get_logger().debug(f'Published visualization for {len(marker_array.markers)} footsteps')

        except Exception as e:
            self.get_logger().error(f'Error publishing visualization: {e}')

    def check_footstep_stability(self, foot_pose: Pose, support_foot_pose: Pose) -> bool:
        """
        Check if a footstep is stable based on support polygon

        Args:
            foot_pose: Pose of the foot to check
            support_foot_pose: Pose of the supporting foot

        Returns:
            bool: True if footstep is stable, False otherwise
        """
        # Simplified stability check
        # In a real implementation, this would calculate the Zero Moment Point (ZMP)
        # and check if it's within the support polygon

        # Calculate distance between feet
        dx = foot_pose.position.x - support_foot_pose.position.x
        dy = foot_pose.position.y - support_foot_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Check if distance is within acceptable range
        min_distance = self.min_step_width
        max_distance = self.step_length * 1.5  # Allow some flexibility

        return min_distance <= distance <= max_distance

    def get_footstep_plan(self) -> List[Tuple[str, Pose]]:
        """
        Get the current footstep plan

        Returns:
            List of tuples containing (foot_name, pose) for each step
        """
        return self.footstep_sequence.copy()

    def reset_planner(self):
        """
        Reset the footstep planner to initial state
        """
        with self.processing_lock:
            self.left_foot_pose = None
            self.right_foot_pose = None
            self.robot_pose = None
            self.current_path = None
            self.footstep_sequence = []
            self.support_polygon = None
            self.planning_active = False
            self.path_received = False


def main(args=None):
    """
    Main function to run the Footstep Planner
    """
    rclpy.init(args=args)

    node = FootstepPlanner()

    try:
        # For demonstration, we'll just let it run and generate dummy steps
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()