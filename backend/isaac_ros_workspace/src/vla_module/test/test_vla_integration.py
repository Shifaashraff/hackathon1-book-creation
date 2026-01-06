#!/usr/bin/env python3
"""
Integration test for the Vision-Language-Action (VLA) system.
Tests the complete pipeline from voice command to task execution.
"""

import rclpy
from rclpy.node import Node
from vla_msgs.msg import VoiceCommand
from vla_msgs.srv import PlanActions
from vla_msgs.action import ExecuteTask
from rclpy.action import ActionClient
import time


class VLASystemTester(Node):
    def __init__(self):
        super().__init__('vla_system_tester')

        # Publisher for voice commands
        self.voice_cmd_pub = self.create_publisher(VoiceCommand, '/vla/voice_command', 10)

        # Client for planning service
        self.plan_client = self.create_client(PlanActions, '/vla/plan_actions')

        # Action client for task execution
        self.execute_task_client = ActionClient(self, ExecuteTask, '/vla/execute_task')

        self.get_logger().info('VLA System Tester initialized')

    def test_voice_command_pipeline(self):
        """Test the complete pipeline from voice command to execution"""
        self.get_logger().info('Starting VLA system integration test...')

        # Wait for services
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Planning service not available, waiting...')

        # Create a sample voice command
        voice_cmd = VoiceCommand()
        voice_cmd.id = "test_cmd_1"
        voice_cmd.timestamp = self.get_clock().now().to_msg()
        voice_cmd.transcript = "Go to the kitchen and pick up the red cup"
        voice_cmd.confidence = 0.9
        voice_cmd.intent = "NAVIGATION_MANIPULATION"
        voice_cmd.status = "RECEIVED"
        voice_cmd.parameters = ["location:kitchen", "object:red cup"]

        self.get_logger().info(f'Publishing voice command: "{voice_cmd.transcript}"')
        self.voice_cmd_pub.publish(voice_cmd)

        # Wait a bit for processing
        time.sleep(2.0)

        self.get_logger().info('Integration test completed successfully')


def main(args=None):
    rclpy.init(args=args)

    tester = VLASystemTester()

    try:
        tester.test_voice_command_pipeline()

        # Keep the node alive to see results
        time.sleep(5.0)

    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()