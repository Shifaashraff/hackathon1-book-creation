#!/usr/bin/env python3
"""
Launch file for the complete Vision-Language-Action (VLA) system.
Launches all necessary nodes for voice command recognition, cognitive planning, and task execution.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Generate the launch description for the VLA system."""

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Define the nodes to launch
    voice_recognition_nodes = [
        Node(
            package='vla_module',
            executable='audio_capture',
            name='audio_capture_node',
            parameters=[
                {'audio_rate': 16000},
                {'audio_channels': 1},
                {'chunk_size': 1024},
                {'noise_threshold': 0.01},
                {'silence_duration': 1.0}
            ],
            output='screen'
        ),
        Node(
            package='vla_module',
            executable='whisper_interface',
            name='whisper_interface_node',
            parameters=[
                {'whisper_model': 'whisper-1'},
                {'language': 'en'},
                {'confidence_threshold': 0.7}
            ],
            output='screen'
        ),
        Node(
            package='vla_module',
            executable='voice_processor',
            name='voice_processor_node',
            parameters=[
                {'min_confidence': 0.7}
            ],
            output='screen'
        )
    ]

    cognitive_planning_nodes = [
        Node(
            package='vla_module',
            executable='command_parser',
            name='command_parser_node',
            output='screen'
        ),
        Node(
            package='vla_module',
            executable='task_planner',
            name='task_planner_node',
            parameters=[
                {'max_plan_steps': 10},
                {'enable_validation': True}
            ],
            output='screen'
        ),
        Node(
            package='vla_module',
            executable='action_converter',
            name='action_converter_node',
            parameters=[
                {'enable_detailed_conversion': True}
            ],
            output='screen'
        )
    ]

    task_execution_nodes = [
        Node(
            package='vla_module',
            executable='task_execution_manager',
            name='task_execution_manager',
            output='screen'
        )
    ]

    # Combine all nodes
    all_nodes = (
        voice_recognition_nodes +
        cognitive_planning_nodes +
        task_execution_nodes
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        )
    ] + all_nodes)