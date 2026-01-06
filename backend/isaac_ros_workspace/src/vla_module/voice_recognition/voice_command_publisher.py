#!/usr/bin/env python3
"""
Voice command publisher for testing the Vision-Language-Action (VLA) system.
Publishes VoiceCommand messages for testing purposes.
"""

import rclpy
from rclpy.node import Node
from vla_msgs.msg import VoiceCommand
from builtin_interfaces.msg import Time
import time


class VoiceCommandPublisherNode(Node):
    def __init__(self):
        super().__init__('voice_command_publisher')

        # Publisher
        self.publisher = self.create_publisher(VoiceCommand, '/vla/voice_command', 10)

        # Timer to publish test messages
        self.timer = self.create_timer(5.0, self.publish_test_command)
        self.command_counter = 0

        self.get_logger().info('Voice Command Publisher Node initialized')

    def publish_test_command(self):
        """Publish a test voice command"""
        commands = [
            "Move forward",
            "Turn left",
            "Go to the kitchen",
            "Pick up the red cup",
            "Stop moving",
            "Find the blue bottle"
        ]

        if self.command_counter < len(commands):
            cmd = commands[self.command_counter]
            self.command_counter += 1
        else:
            # Cycle back to first command
            cmd = commands[0]
            self.command_counter = 1

        # Create and publish a VoiceCommand message
        msg = VoiceCommand()
        msg.id = f"test_cmd_{self.get_clock().now().nanoseconds}"
        msg.timestamp = self.get_clock().now().to_msg()
        msg.transcript = cmd
        msg.confidence = 0.95  # High confidence for test
        msg.intent = self._determine_intent(cmd)
        msg.parameters = self._extract_parameters(cmd)
        msg.status = "RECEIVED"

        self.publisher.publish(msg)
        self.get_logger().info(f'Published test command: "{cmd}"')


    def _determine_intent(self, command):
        """Determine intent from command text"""
        command_lower = command.lower()

        if any(word in command_lower for word in ['move', 'go', 'forward', 'backward', 'left', 'right', 'turn']):
            return 'NAVIGATION'
        elif any(word in command_lower for word in ['pick', 'grab', 'lift', 'take', 'hold', 'drop']):
            return 'MANIPULATION'
        elif any(word in command_lower for word in ['find', 'look', 'see', 'locate', 'search']):
            return 'PERCEPTION'
        elif any(word in command_lower for word in ['stop', 'pause', 'wait']):
            return 'STOP'
        else:
            return 'UNKNOWN'

    def _extract_parameters(self, command):
        """Extract parameters from command text"""
        parameters = []

        # Extract directions
        directions = ['forward', 'backward', 'left', 'right']
        for direction in directions:
            if direction in command.lower():
                parameters.append(f"direction:{direction}")

        # Extract objects
        objects = ['cup', 'bottle', 'box', 'red', 'blue', 'green']
        for obj in objects:
            if obj in command.lower():
                parameters.append(f"object:{obj}")

        # Extract locations
        locations = ['kitchen', 'bedroom', 'living room']
        for loc in locations:
            if loc in command.lower():
                parameters.append(f"location:{loc}")

        return parameters


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()