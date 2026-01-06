#!/usr/bin/env python3

"""
Isaac ROS Perception Pipeline Node
ROS 2 node that processes sensor data for perception tasks including VSLAM, object detection, and segmentation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu, LaserScan
from geometry_msgs.msg import Pose, Point, Quaternion, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Header, String
from builtin_interfaces.msg import Time
from tf2_ros import TransformListener, Buffer

import numpy as np
import cv2
from cv_bridge import CvBridge
import message_filters
from typing import List, Optional, Dict, Any
import threading
import time
from collections import deque


class PerceptionPipelineNode(Node):
    """
    ROS 2 Node for perception pipeline processing
    Handles VSLAM, object detection, segmentation, and other perception tasks
    """

    def __init__(self):
        super().__init__('perception_pipeline_node')

        # Declare parameters
        self.declare_parameter('robot_name', 'humanoid_robot')
        self.declare_parameter('namespace', 'humanoid')
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('processing_frequency', 10.0)  # Hz
        self.declare_parameter('enable_vslam', True)
        self.declare_parameter('enable_object_detection', True)
        self.declare_parameter('enable_segmentation', True)
        self.declare_parameter('enable_apriltag_detection', True)
        self.declare_parameter('camera_topic', '/front_stereo_camera/left/image_rect_color')
        self.declare_parameter('camera_info_topic', '/front_stereo_camera/left/camera_info')
        self.declare_parameter('right_camera_topic', '/front_stereo_camera/right/image_rect')
        self.declare_parameter('imu_topic', '/humanoid/imu/data')
        self.declare_parameter('lidar_topic', '/humanoid/scan')

        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.namespace = self.get_parameter('namespace').value
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.processing_frequency = self.get_parameter('processing_frequency').value
        self.enable_vslam = self.get_parameter('enable_vslam').value
        self.enable_object_detection = self.get_parameter('enable_object_detection').value
        self.enable_segmentation = self.get_parameter('enable_segmentation').value
        self.enable_apriltag_detection = self.get_parameter('enable_apriltag_detection').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.right_camera_topic = self.get_parameter('right_camera_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize sensor data buffers
        self.left_image = None
        self.right_image = None
        self.camera_info = None
        self.imu_data = None
        self.lidar_data = None
        self.last_image_time = None

        # Initialize perception results
        self.vslam_results = None
        self.detection_results = []
        self.segmentation_results = None
        self.apriltag_results = []

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

        # Create subscribers
        self.left_image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.left_image_callback,
            sensor_qos
        )

        self.right_image_sub = self.create_subscription(
            Image,
            self.right_camera_topic,
            self.right_image_callback,
            sensor_qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            sensor_qos
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.lidar_callback,
            sensor_qos
        )

        # Create publishers for perception results following Isaac ROS topic conventions
        self.perception_data_pub = self.create_publisher(
            'isaac_ros_messages/msg/PerceptionData',
            'perception_data',
            1
        )

        # Visual SLAM topics
        self.visual_slam_poses_pub = self.create_publisher(
            Odometry,  # Using Odometry for pose data as per ROS conventions
            'visual_slam/poses',
            10
        )

        self.visual_slam_path_pub = self.create_publisher(
            'nav_msgs/msg/Path',
            'visual_slam/path',
            10
        )

        # Object detection topics
        self.detectnet_detections_pub = self.create_publisher(
            'isaac_ros_messages/msg/ObjectDetection2D',
            'detectnet/detections',
            10
        )

        # Segmentation topics
        self.segmentation_map_pub = self.create_publisher(
            Image,
            'segmentation/map',
            sensor_qos
        )

        self.segmentation_colormap_pub = self.create_publisher(
            Image,
            'segmentation/colormap',
            sensor_qos
        )

        # AprilTag detection topics
        self.apriltag_detections_pub = self.create_publisher(
            'isaac_ros_messages/msg/AprilTagDetection',
            'apriltag/detections',
            10
        )

        # Feature tracking topics
        self.visual_slam_features_pub = self.create_publisher(
            PointCloud2,
            'visual_slam/features',
            10
        )

        # Initialize TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize processing timer
        self.processing_timer = self.create_timer(
            1.0 / self.processing_frequency,
            self.process_perception_pipeline
        )

        # Initialize internal state
        self.processing_lock = threading.Lock()
        self.is_processing = False

        # Initialize perception modules
        self.initialize_perception_modules()

        self.get_logger().info(f'Perception Pipeline Node initialized for {self.robot_name}')
        self.get_logger().info(f'Processing frequency: {self.processing_frequency} Hz')
        self.get_logger().info(f'VSLAM enabled: {self.enable_vslam}')
        self.get_logger().info(f'Object detection enabled: {self.enable_object_detection}')
        self.get_logger().info(f'Segmentation enabled: {self.enable_segmentation}')

    def initialize_perception_modules(self):
        """
        Initialize perception modules based on enabled features
        """
        self.get_logger().info('Initializing perception modules...')

        # Initialize VSLAM module if enabled
        if self.enable_vslam:
            self.get_logger().info('Initializing VSLAM module')
            # In a real implementation, this would initialize the VSLAM algorithm
            # For now, we'll create a placeholder
            self.vslam_module = {
                'initialized': True,
                'tracking': False,
                'keyframes': [],
                'map_points': []
            }

        # Initialize object detection module if enabled
        if self.enable_object_detection:
            self.get_logger().info('Initializing object detection module')
            # In a real implementation, this would initialize a detection model
            self.detection_module = {
                'initialized': True,
                'model_loaded': True
            }

        # Initialize segmentation module if enabled
        if self.enable_segmentation:
            self.get_logger().info('Initializing segmentation module')
            # In a real implementation, this would initialize a segmentation model
            self.segmentation_module = {
                'initialized': True,
                'model_loaded': True
            }

        # Initialize AprilTag detection module if enabled
        if self.enable_apriltag_detection:
            self.get_logger().info('Initializing AprilTag detection module')
            # In a real implementation, this would initialize AprilTag detection
            self.apriltag_module = {
                'initialized': True,
                'detector_params': {
                    'family': 'tag36h11',
                    'max_hamming_dist': 3,
                    'quad_decimate': 1.0,
                    'quad_sigma': 0.0
                }
            }

    def left_image_callback(self, msg: Image):
        """
        Callback for left camera image from stereo pair

        Args:
            msg (Image): Left camera image message
        """
        with self.processing_lock:
            try:
                self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.last_image_time = msg.header.stamp
            except Exception as e:
                self.get_logger().error(f'Error converting left image: {e}')

    def right_image_callback(self, msg: Image):
        """
        Callback for right camera image from stereo pair

        Args:
            msg (Image): Right camera image message
        """
        with self.processing_lock:
            try:
                self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f'Error converting right image: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        """
        Callback for camera info

        Args:
            msg (CameraInfo): Camera info message
        """
        with self.processing_lock:
            self.camera_info = msg

    def imu_callback(self, msg: Imu):
        """
        Callback for IMU data

        Args:
            msg (Imu): IMU data message
        """
        with self.processing_lock:
            self.imu_data = msg

    def lidar_callback(self, msg: LaserScan):
        """
        Callback for LIDAR data

        Args:
            msg (LaserScan): LIDAR data message
        """
        with self.processing_lock:
            self.lidar_data = msg

    def process_perception_pipeline(self):
        """
        Main perception pipeline processing function
        Called at the specified processing frequency
        """
        if self.is_processing:
            return  # Skip if already processing

        with self.processing_lock:
            self.is_processing = True

        try:
            # Check if we have required sensor data
            if self.left_image is None or self.camera_info is None:
                self.get_logger().debug('Waiting for camera data...')
                return

            # Process VSLAM if enabled
            if self.enable_vslam and self.right_image is not None:
                self.process_vslam()

            # Process object detection if enabled
            if self.enable_object_detection:
                self.process_object_detection()

            # Process segmentation if enabled
            if self.enable_segmentation:
                self.process_segmentation()

            # Process AprilTag detection if enabled
            if self.enable_apriltag_detection:
                self.process_apriltag_detection()

            # Publish perception results
            self.publish_perception_results()

        except Exception as e:
            self.get_logger().error(f'Error in perception pipeline: {e}')
        finally:
            with self.processing_lock:
                self.is_processing = False

    def process_vslam(self):
        """
        Process Visual SLAM using stereo images
        """
        try:
            # In a real implementation, this would run the VSLAM algorithm
            # For now, we'll create simulated results

            # Create simulated VSLAM results
            current_time = self.get_clock().now()
            pose = Pose()
            pose.position.x = 0.0  # Simulated position
            pose.position.y = 0.0
            pose.position.z = 0.0
            pose.orientation.w = 1.0  # Identity quaternion
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0

            self.vslam_results = {
                'timestamp': current_time,
                'pose': pose,
                'tracking_confidence': 0.95,
                'is_localized': True,
                'keyframe_count': len(self.vslam_module['keyframes']),
                'map_point_count': len(self.vslam_module['map_points'])
            }

            # Update VSLAM module state
            self.vslam_module['tracking'] = True

            # Add simulated keyframe
            if len(self.vslam_module['keyframes']) < 100:  # Limit for simulation
                self.vslam_module['keyframes'].append({
                    'timestamp': current_time,
                    'pose': pose,
                    'features': 100  # Simulated feature count
                })

        except Exception as e:
            self.get_logger().error(f'Error in VSLAM processing: {e}')

    def process_object_detection(self):
        """
        Process object detection on the current image
        """
        try:
            # In a real implementation, this would run an object detection model
            # For now, we'll create simulated detection results

            # Clear previous detections
            self.detection_results = []

            # Create simulated detections
            for i in range(3):  # Simulate 3 detections
                detection = {
                    'id': i,
                    'class_name': 'person' if i == 0 else 'chair' if i == 1 else 'table',
                    'class_id': 1 if i == 0 else 2 if i == 1 else 3,
                    'confidence': 0.85 + (0.1 * i),  # Vary confidence
                    'bbox': {
                        'x_min': 0.2 + (0.1 * i),
                        'y_min': 0.3 + (0.05 * i),
                        'x_max': 0.4 + (0.1 * i),
                        'y_max': 0.6 + (0.05 * i)
                    }
                }
                self.detection_results.append(detection)

        except Exception as e:
            self.get_logger().error(f'Error in object detection processing: {e}')

    def process_segmentation(self):
        """
        Process semantic segmentation on the current image
        """
        try:
            # In a real implementation, this would run a segmentation model
            # For now, we'll create a simulated segmentation result

            # Create a dummy segmentation image (same size as input image)
            if self.left_image is not None:
                height, width = self.left_image.shape[:2]
                # Create a dummy segmentation mask with random class IDs
                segmentation_mask = np.random.randint(0, 10, (height, width), dtype=np.uint8)

                # Convert to ROS Image message
                try:
                    seg_image_msg = self.bridge.cv2_to_imgmsg(segmentation_mask, encoding='mono8')
                    seg_image_msg.header.stamp = self.get_clock().now().to_msg()
                    seg_image_msg.header.frame_id = 'camera_link'

                    self.segmentation_results = seg_image_msg
                except Exception as e:
                    self.get_logger().error(f'Error converting segmentation result: {e}')

        except Exception as e:
            self.get_logger().error(f'Error in segmentation processing: {e}')

    def process_apriltag_detection(self):
        """
        Process AprilTag detection in the current image
        """
        try:
            # In a real implementation, this would use an AprilTag detector
            # For now, we'll simulate detection results

            # Clear previous detections
            self.apriltag_results = []

            # Simulate AprilTag detection (only if we have a recent image)
            if self.left_image is not None:
                # For simulation, create a detection every few seconds
                current_time = self.get_clock().now().nanoseconds / 1e9
                if int(current_time) % 5 == 0:  # Detect every 5 seconds
                    tag_detection = {
                        'id': 1,
                        'size': 0.16,  # 16cm tag
                        'confidence': 0.9,
                        'pose': Pose()  # Identity pose for simulation
                    }
                    self.apriltag_results.append(tag_detection)

        except Exception as e:
            self.get_logger().error(f'Error in AprilTag detection processing: {e}')

    def publish_perception_results(self):
        """
        Publish perception results to ROS topics
        """
        try:
            # Publish VSLAM results if available
            if self.vslam_results is not None and self.enable_vslam:
                # Create VSLAM pose message using Odometry (standard for pose data)
                from nav_msgs.msg import Odometry
                from geometry_msgs.msg import PoseWithCovarianceStamped

                vslam_pose_msg = Odometry()
                vslam_pose_msg.header.stamp = self.get_clock().now().to_msg()
                vslam_pose_msg.header.frame_id = 'map'
                vslam_pose_msg.child_frame_id = 'camera_link'
                vslam_pose_msg.pose.pose = self.vslam_results['pose']

                # Publish to visual_slam/poses topic
                self.visual_slam_poses_pub.publish(vslam_pose_msg)

            # Publish object detections
            if self.detection_results and self.enable_object_detection:
                from geometry_msgs.msg import Point
                from std_msgs.msg import String, Int32, Float32

                for detection in self.detection_results:
                    # For now, create a simple detection using standard messages
                    # In a real implementation, we would use the custom ObjectDetection2D message
                    detection_point = Point()
                    detection_point.x = (detection['bbox']['x_min'] + detection['bbox']['x_max']) / 2.0
                    detection_point.y = (detection['bbox']['y_min'] + detection['bbox']['y_max']) / 2.0
                    detection_point.z = detection['confidence']

            # Publish segmentation results
            if self.segmentation_results is not None and self.enable_segmentation:
                # Publish segmentation map
                self.segmentation_map_pub.publish(self.segmentation_results)

                # Create and publish a dummy colormap (for visualization)
                from std_msgs.msg import Header
                colormap_msg = Image()
                colormap_msg.header.stamp = self.get_clock().now().to_msg()
                colormap_msg.header.frame_id = 'camera_link'
                colormap_msg.height = 480
                colormap_msg.width = 640
                colormap_msg.encoding = 'rgb8'
                colormap_msg.is_bigendian = 0
                colormap_msg.step = 640 * 3  # Width * channels
                colormap_msg.data = [0] * (480 * 640 * 3)  # Dummy data

                self.segmentation_colormap_pub.publish(colormap_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing perception results: {e}')

    def get_perception_state(self) -> Dict[str, Any]:
        """
        Get current perception state

        Returns:
            Dict: Current perception state including detections, VSLAM results, etc.
        """
        return {
            'vslam_results': self.vslam_results,
            'detection_results': self.detection_results,
            'segmentation_results': self.segmentation_results,
            'apriltag_results': self.apriltag_results,
            'sensor_data_available': {
                'left_image': self.left_image is not None,
                'right_image': self.right_image is not None,
                'camera_info': self.camera_info is not None,
                'imu': self.imu_data is not None,
                'lidar': self.lidar_data is not None
            }
        }


def main(args=None):
    """
    Main function to run the Perception Pipeline Node
    """
    rclpy.init(args=args)

    node = PerceptionPipelineNode()

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