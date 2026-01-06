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
    """Generate launch description for Isaac Sim Robot Control."""

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    robot_name = LaunchConfiguration('robot_name', default='humanoid_robot')
    namespace = LaunchConfiguration('namespace', default='humanoid')

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description':
                '''<?xml version="1.0"?>
                <robot name="humanoid_robot">
                    <!-- This will be loaded from URDF file -->
                </robot>'''
            }
        ],
        namespace=namespace,
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # Isaac Sim robot control node
    isaac_sim_robot_control_node = Node(
        package='isaac_ros_examples',
        executable='isaac_sim_robot_control_node',
        name='isaac_sim_robot_control_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_name': robot_name},
            {'namespace': namespace}
        ],
        namespace=namespace,
        remappings=[
            ('/joint_states', 'joint_states'),
            ('/cmd_vel', 'cmd_vel'),
            ('/odom', 'odom'),
        ]
    )

    # TF2 broadcaster node for odom to base_link transform
    tf2_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_base_tf_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    return launch.LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Isaac Sim) clock if true'),
        DeclareLaunchArgument(
            'robot_name',
            default_value='humanoid_robot',
            description='Name of the robot'),
        DeclareLaunchArgument(
            'namespace',
            default_value='humanoid',
            description='Namespace for the robot nodes'),

        # Nodes
        robot_state_publisher,
        isaac_sim_robot_control_node,
        tf2_broadcaster,
    ])