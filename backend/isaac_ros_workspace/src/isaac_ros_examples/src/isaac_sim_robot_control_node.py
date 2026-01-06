#!/usr/bin/env python3

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

"""
Isaac Sim Robot Control Node
ROS 2 node that interfaces between ROS 2 topics/services and Isaac Sim for humanoid robot control
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformStamped

import numpy as np
import math
import time
from typing import List, Optional


class IsaacSimRobotControlNode(Node):
    """
    ROS 2 Node for controlling humanoid robot in Isaac Sim
    Interfaces with Isaac Sim via Isaac ROS bridge
    """

    def __init__(self):
        super().__init__('isaac_sim_robot_control_node')

        # Declare parameters
        self.declare_parameter('robot_name', 'humanoid_robot')
        self.declare_parameter('namespace', 'humanoid')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('control_frequency', 50.0)  # Hz
        self.declare_parameter('odom_publish_frequency', 50.0)  # Hz

        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.namespace = self.get_parameter('namespace').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.odom_publish_frequency = self.get_parameter('odom_publish_frequency').value

        # Initialize robot state
        self.robot_position = [0.0, 0.0, 0.0]  # x, y, z
        self.robot_orientation = [0.0, 0.0, 0.0, 1.0]  # x, y, z, w (quaternion)
        self.robot_linear_vel = [0.0, 0.0, 0.0]  # x, y, z
        self.robot_angular_vel = [0.0, 0.0, 0.0]  # x, y, z
        self.joint_positions = {}
        self.joint_velocities = {}
        self.joint_efforts = {}

        # Define humanoid joint names
        self.joint_names = [
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle',
            'left_shoulder', 'left_elbow',
            'right_shoulder', 'right_elbow'
        ]

        # Initialize joint positions to zero
        for joint_name in self.joint_names:
            self.joint_positions[joint_name] = 0.0
            self.joint_velocities[joint_name] = 0.0
            self.joint_efforts[joint_name] = 0.0

        # Initialize odometry
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.odom_last_time = self.get_clock().now()

        # Create QoS profile for sensors (best effort for camera/IMU data)
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Create QoS profile for commands (reliable for control)
        cmd_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            cmd_qos
        )

        # Create publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            1
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            1
        )

        self.imu_pub = self.create_publisher(
            Imu,
            'imu/data',
            sensor_qos
        )

        # Initialize TF broadcaster if needed
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Create timer for publishing joint states and odometry
        self.joint_state_timer = self.create_timer(
            1.0 / self.odom_publish_frequency,
            self.publish_joint_states
        )

        self.odom_timer = self.create_timer(
            1.0 / self.odom_publish_frequency,
            self.publish_odometry
        )

        self.imu_timer = self.create_timer(
            1.0 / 100.0,  # Publish IMU at 100Hz
            self.publish_imu
        )

        # Initialize timers for Isaac Sim communication
        self.isaac_sim_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.update_isaac_sim
        )

        self.get_logger().info(f'Isaac Sim Robot Control Node initialized for {self.robot_name}')

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback for velocity commands from ROS

        Args:
            msg (Twist): Velocity command message
        """
        # Store the commanded velocities
        self.robot_linear_vel[0] = msg.linear.x
        self.robot_linear_vel[1] = msg.linear.y
        self.robot_linear_vel[2] = msg.linear.z
        self.robot_angular_vel[0] = msg.angular.x
        self.robot_angular_vel[1] = msg.angular.y
        self.robot_angular_vel[2] = msg.angular.z

        # For a humanoid robot, we might interpret this differently
        # For now, we'll use x as forward/backward and z as rotation
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Update joint positions based on commanded velocity
        # This is a simplified model - in reality, this would interface with Isaac Sim
        self.update_joint_positions_from_velocity(linear_x, angular_z)

    def update_joint_positions_from_velocity(self, linear_x: float, angular_z: float):
        """
        Update joint positions based on linear and angular velocity commands
        This is a simplified model for bipedal locomotion

        Args:
            linear_x (float): Linear velocity in x direction
            angular_z (float): Angular velocity around z axis
        """
        # Get current time
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = 1.0 / self.control_frequency  # Time step

        # Simple walking gait based on velocity commands
        if abs(linear_x) > 0.01 or abs(angular_z) > 0.01:
            # Walking pattern generation
            phase = (current_time * 2.0) % (2 * math.pi)  # Walking phase

            # Update leg joints for walking
            left_hip_angle = 0.2 * math.sin(phase)
            right_hip_angle = 0.2 * math.sin(phase + math.pi)
            left_knee_angle = -0.1 * math.sin(phase)
            right_knee_angle = -0.1 * math.sin(phase + math.pi)

            # Update joint positions
            self.joint_positions['left_hip'] = left_hip_angle
            self.joint_positions['right_hip'] = right_hip_angle
            self.joint_positions['left_knee'] = left_knee_angle
            self.joint_positions['right_knee'] = right_knee_angle

            # Update arm joints for balance
            self.joint_positions['left_shoulder'] = -0.1 * math.sin(phase + math.pi/2)
            self.joint_positions['right_shoulder'] = 0.1 * math.sin(phase + math.pi/2)

            # Update position based on velocity
            self.odom_x += linear_x * dt
            self.odom_y += 0.0  # Simplified - only x movement for now
            self.odom_theta += angular_z * dt

    def publish_joint_states(self):
        """
        Publish joint state messages to ROS
        """
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.namespace}/base_link'

        # Set joint names and values
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        msg.velocity = list(self.joint_velocities.values())
        msg.effort = list(self.joint_efforts.values())

        self.joint_state_pub.publish(msg)

    def publish_odometry(self):
        """
        Publish odometry messages to ROS
        """
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.namespace}/odom'
        msg.child_frame_id = f'{self.namespace}/base_link'

        # Set position
        msg.pose.pose.position.x = self.odom_x
        msg.pose.pose.position.y = self.odom_y
        msg.pose.pose.position.z = 0.0  # Simplified - assume flat ground

        # Convert theta to quaternion
        cos_theta = math.cos(self.odom_theta)
        sin_theta = math.sin(self.odom_theta)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sin_theta
        msg.pose.pose.orientation.w = cos_theta

        # Set velocities
        msg.twist.twist.linear.x = self.robot_linear_vel[0]
        msg.twist.twist.linear.y = self.robot_linear_vel[1]
        msg.twist.twist.angular.z = self.robot_angular_vel[2]

        self.odom_pub.publish(msg)

        # Publish TF if enabled
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = f'{self.namespace}/odom'
            t.child_frame_id = f'{self.namespace}/base_link'

            t.transform.translation.x = self.odom_x
            t.transform.translation.y = self.odom_y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = sin_theta
            t.transform.rotation.w = cos_theta

            self.tf_broadcaster.sendTransform(t)

    def publish_imu(self):
        """
        Publish IMU messages to ROS
        """
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.namespace}/imu_link'

        # Set orientation (simplified - just identity for now)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        # Set angular velocity (simplified)
        msg.angular_velocity.x = self.robot_angular_vel[0]
        msg.angular_velocity.y = self.robot_angular_vel[1]
        msg.angular_velocity.z = self.robot_angular_vel[2]

        # Set linear acceleration (simplified - just gravity for now)
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = -9.81

        self.imu_pub.publish(msg)

    def update_isaac_sim(self):
        """
        Update Isaac Sim with current joint positions and get state back
        This is where the actual interface with Isaac Sim would happen
        """
        # In a real implementation, this would:
        # 1. Send joint commands to Isaac Sim
        # 2. Receive updated robot state from Isaac Sim
        # 3. Update the internal state variables

        # For now, this is a simulation of the interface
        self.get_logger().debug('Updating Isaac Sim with joint positions')

        # Simulate receiving updated state from Isaac Sim
        # This would normally come from Isaac Sim via the ROS bridge
        pass

    def get_robot_state(self):
        """
        Get current robot state

        Returns:
            dict: Current robot state including positions, velocities, etc.
        """
        return {
            'position': self.robot_position,
            'orientation': self.robot_orientation,
            'linear_velocity': self.robot_linear_vel,
            'angular_velocity': self.robot_angular_vel,
            'joint_positions': self.joint_positions,
            'joint_velocities': self.joint_velocities,
            'joint_efforts': self.joint_efforts
        }


def main(args=None):
    """
    Main function to run the Isaac Sim Robot Control Node
    """
    rclpy.init(args=args)

    node = IsaacSimRobotControlNode()

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