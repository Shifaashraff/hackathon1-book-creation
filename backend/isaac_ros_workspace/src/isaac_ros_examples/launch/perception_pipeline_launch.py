# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def generate_launch_description():
    """Generate launch description for Isaac ROS Perception Pipeline."""

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    namespace = LaunchConfiguration('namespace', default='humanoid')

    # Isaac ROS visual slam node (stereo camera based)
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        name='visual_slam_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'enable_rectified_pose': True},
            {'rectified_frame_id': 'camera_color_optical_frame'},
            {'publish_odom_tf': True},
            {'map_frame': 'map'},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'use_vio_pose_graph_optimizer': True},
        ],
        namespace=namespace,
        remappings=[
            ('/stereo_camera/left/image_rect_color', '/front_stereo_camera/left/image_rect_color'),
            ('/stereo_camera/left/camera_info', '/front_stereo_camera/left/camera_info'),
            ('/stereo_camera/right/image_rect', '/front_stereo_camera/right/image_rect'),
            ('/stereo_camera/right/camera_info', '/front_stereo_camera/right/camera_info'),
            ('/visual_slam/imu', '/imu/data'),
        ]
    )

    # Isaac ROS apriltag node for VSLAM
    apriltag_node = Node(
        package='isaac_ros_apriltag',
        executable='apriltag_node',
        name='apriltag_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'family': 'tag36h11'},
            {'max_tags': 64},
            {'tag_size': 0.032},
        ],
        namespace=namespace,
        remappings=[
            ('/image', '/front_stereo_camera/left/image_rect_color'),
            ('/camera_info', '/front_stereo_camera/left/camera_info'),
        ]
    )

    # Isaac ROS detectnet node for object detection
    detectnet_node = Node(
        package='isaac_ros_detectnet',
        executable='detectnet_node',
        name='detectnet_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'model_name': 'ssd_mobilenet_v2_coco'},
            {'confidence_threshold': 0.7},
            {'input_tensor': 'input'},
            {'input_format': 'channel_first'},
            {'output_tensor': 'scores'},
        ],
        namespace=namespace,
        remappings=[
            ('/image', '/front_stereo_camera/left/image_rect_color'),
        ]
    )

    # Isaac ROS image segmentation node
    segmentation_node = Node(
        package='isaac_ros_segmentation',
        executable='segmentation_node',
        name='segmentation_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'model_name': 'dnn_segmentation'},
        ],
        namespace=namespace,
        remappings=[
            ('/image', '/front_stereo_camera/left/image_rect_color'),
        ]
    )

    return launch.LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Isaac Sim) clock if true'),
        DeclareLaunchArgument(
            'namespace',
            default_value='humanoid',
            description='Namespace for the robot nodes'),

        # Nodes
        visual_slam_node,
        apriltag_node,
        detectnet_node,
        segmentation_node,
    ])