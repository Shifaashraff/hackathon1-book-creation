#!/usr/bin/env python3

"""
Full Simulation Launch File for Isaac AI Robot Brain
This launch file combines Isaac Sim, Isaac ROS perception, and Nav2 navigation
into a complete integrated system for humanoid robot simulation and control.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for full simulation system."""

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='humanoid',
        description='Namespace for the robot'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    declare_isaac_sim_cmd = DeclareLaunchArgument(
        'launch_isaac_sim',
        default_value='true',
        description='Launch Isaac Sim if true'
    )

    declare_isaac_ros_cmd = DeclareLaunchArgument(
        'launch_isaac_ros',
        default_value='true',
        description='Launch Isaac ROS components if true'
    )

    declare_nav2_cmd = DeclareLaunchArgument(
        'launch_nav2',
        default_value='true',
        description='Launch Nav2 if true'
    )

    # Get configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_isaac_sim = LaunchConfiguration('launch_isaac_sim')
    launch_isaac_ros = LaunchConfiguration('launch_isaac_ros')
    launch_nav2 = LaunchConfiguration('launch_nav2')

    # Isaac Sim launch (if needed)
    # Note: Isaac Sim typically runs separately as it's a standalone application
    # We'll include a placeholder for Isaac Sim integration

    # Isaac ROS perception pipeline launch
    perception_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_ros_examples'),
                'launch',
                'perception_pipeline.launch.py'
            ])
        ]),
        condition_if=launch_isaac_ros
    )

    # Isaac ROS robot control launch
    robot_control_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_ros_examples'),
                'launch',
                'robot_control.launch.py'
            ])
        ]),
        condition_if=launch_isaac_ros
    )

    # Isaac ROS navigation interface launch
    nav_interface_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_ros_examples'),
                'launch',
                'nav2_interface.launch.py'
            ])
        ]),
        condition_if=launch_isaac_ros
    )

    # Nav2 navigation launch
    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2'),
                'launch',
                'humanoid_navigation.launch.py'
            ])
        ]),
        condition_if=launch_nav2
    )

    # Rosbridge server launch for web integration
    rosbridge_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('isaac_ros_examples'),
                'launch',
                'rosbridge_launch.py'
            ])
        ])
    )

    # Isaac Sim bridge node to connect with Isaac Sim
    isaac_sim_bridge_node = Node(
        package='isaac_ros_examples',
        executable='isaac_sim_bridge_node',
        name='isaac_sim_bridge',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_namespace': namespace},
            {'sim_rate': 1.0}  # Real-time simulation
        ],
        output='screen',
        respawn=True
    )

    # Perception integration node
    perception_integration_node = Node(
        package='isaac_ros_examples',
        executable='perception_integration_node',
        name='perception_integration',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_namespace': namespace}
        ],
        output='screen',
        respawn=True
    )

    # Navigation integration node
    navigation_integration_node = Node(
        package='isaac_ros_examples',
        executable='navigation_integration_node',
        name='navigation_integration',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_namespace': namespace}
        ],
        output='screen',
        respawn=True
    )

    # TF broadcaster for humanoid robot
    tf_broadcaster_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='humanoid_tf_broadcaster',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'odom',
            '--child-frame-id', [namespace, '/base_link']
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_isaac_sim_cmd)
    ld.add_action(declare_isaac_ros_cmd)
    ld.add_action(declare_nav2_cmd)

    # Add Isaac Sim integration
    ld.add_action(isaac_sim_bridge_node)

    # Add Isaac ROS components
    ld.add_action(perception_launch_cmd)
    ld.add_action(robot_control_launch_cmd)
    ld.add_action(nav_interface_launch_cmd)
    ld.add_action(perception_integration_node)

    # Add Nav2 components
    ld.add_action(nav2_launch_cmd)
    ld.add_action(navigation_integration_node)

    # Add Rosbridge
    ld.add_action(rosbridge_launch_cmd)

    # Add TF broadcasters
    ld.add_action(tf_broadcaster_node)

    return ld


def main(args=None):
    """Main function to run the full simulation launch file."""
    ld = generate_launch_description()
    return ld


if __name__ == '__main__':
    main()