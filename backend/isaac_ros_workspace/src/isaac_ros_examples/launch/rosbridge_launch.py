#!/usr/bin/env python3

"""
ROS Bridge Server Launch File for Isaac ROS Integration
This launch file starts the rosbridge_server to enable web-based communication
and integration with Isaac Sim and external systems.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for rosbridge server."""

    # Declare launch arguments
    declare_port_cmd = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Port for rosbridge_websocket to listen on'
    )

    declare_address_cmd = DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description='Address for rosbridge_websocket to bind to'
    )

    declare_ssl_cmd = DeclareLaunchArgument(
        'ssl',
        default_value='False',
        description='Enable SSL for rosbridge_websocket'
    )

    declare_certfile_cmd = DeclareLaunchArgument(
        'certfile',
        default_value='',
        description='Path to certificate file for SSL'
    )

    declare_keyfile_cmd = DeclareLaunchArgument(
        'keyfile',
        default_value='',
        description='Path to key file for SSL'
    )

    declare authenticate_cmd = DeclareLaunchArgument(
        'authenticate',
        default_value='False',
        description='Enable authentication for rosbridge'
    )

    # Get configurations
    port = LaunchConfiguration('port')
    address = LaunchConfiguration('address')
    ssl = LaunchConfiguration('ssl')
    certfile = LaunchConfiguration('certfile')
    keyfile = LaunchConfiguration('keyfile')
    authenticate = LaunchConfiguration('authenticate')

    # Define the rosbridge_websocket node
    rosbridge_websocket_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[
            {'port': port},
            {'address': address},
            {'retry_startup_delay': 1.0},
            {'fragment_timeout': 600},
            {'delay_between_messages': 0},
            {'max_message_size': 10000000},
            {'ssl': ssl},
            {'certfile': certfile},
            {'keyfile': keyfile},
            {'authenticate': authenticate}
        ],
        output='screen'
    )

    # Define the rosapi node
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        parameters=[
            {'topics_glob': ''},
            {'services_glob': ''},
            {'params_glob': ''}
        ],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_port_cmd)
    ld.add_action(declare_address_cmd)
    ld.add_action(declare_ssl_cmd)
    ld.add_action(declare_certfile_cmd)
    ld.add_action(declare_keyfile_cmd)
    ld.add_action(declare authenticate_cmd)

    # Add nodes
    ld.add_action(rosbridge_websocket_node)
    ld.add_action(rosapi_node)

    return ld


def main(args=None):
    """Main function to run the rosbridge launch file."""
    ld = generate_launch_description()
    return ld


if __name__ == '__main__':
    main()