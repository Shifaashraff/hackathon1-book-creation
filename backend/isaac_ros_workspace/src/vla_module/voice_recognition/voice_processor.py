#!/usr/bin/env python3
"""
Voice processor module for the Vision-Language-Action (VLA) system.
Handles voice command processing, validation, and preparation for cognitive planning.
"""

import rclpy
from rclpy.node import Node
from vla_msgs.msg import VoiceCommand
from vla_msgs.srv import PlanActions
import re


class VoiceProcessorNode(Node):
    def __init__(self):
        super().__init__('voice_processor_node')

        # Subscribers
        self.voice_cmd_sub = self.create_subscription(
            VoiceCommand,
            '/vla/voice_command',
            self.voice_command_callback,
            10
        )

        # Publishers
        self.processed_voice_cmd_pub = self.create_publisher(
            VoiceCommand,
            '/vla/processed_voice_command',
            10
        )

        # Client for cognitive planning service
        self.plan_client = self.create_client(PlanActions, '/vla/plan_actions')
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Planning service not available, waiting...')

        # Configuration parameters
        self.min_confidence = self.declare_parameter('min_confidence', 0.7).value

        self.get_logger().info('Voice Processor Node initialized')

    def voice_command_callback(self, msg):
        """Process incoming voice command"""
        self.get_logger().info(f'Received voice command: "{msg.transcript}" (confidence: {msg.confidence})')

        # Validate the voice command
        if not self._validate_voice_command(msg):
            self.get_logger().warning(f'Voice command failed validation: "{msg.transcript}"')
            return

        # Process the voice command
        processed_cmd = self._process_voice_command(msg)

        # Publish the processed command
        self.processed_voice_cmd_pub.publish(processed_cmd)

        # Send to cognitive planning service
        self._send_to_planning_service(processed_cmd)

    def _validate_voice_command(self, cmd):
        """Validate the voice command"""
        # Check confidence threshold
        if cmd.confidence < self.min_confidence:
            self.get_logger().warning(f'Command confidence {cmd.confidence} below threshold {self.min_confidence}')
            return False

        # Check if transcript is empty
        if not cmd.transcript or not cmd.transcript.strip():
            self.get_logger().warning('Empty or invalid transcript')
            return False

        # Check for potentially harmful commands
        if self._is_command_potentially_harmful(cmd.transcript):
            self.get_logger().warning(f'Potentially harmful command detected: "{cmd.transcript}"')
            return False

        return True

    def _process_voice_command(self, cmd):
        """Process the voice command and extract intent and parameters"""
        # Create a copy of the command to modify
        processed_cmd = VoiceCommand()
        processed_cmd.id = cmd.id
        processed_cmd.timestamp = cmd.timestamp
        processed_cmd.transcript = cmd.transcript
        processed_cmd.confidence = cmd.confidence
        processed_cmd.intent = cmd.intent
        processed_cmd.status = "PROCESSING"

        # Extract parameters from the command
        parameters = self._extract_parameters(cmd.transcript)
        processed_cmd.parameters = parameters

        self.get_logger().info(f'Processed command - Intent: {processed_cmd.intent}, Parameters: {parameters}')

        return processed_cmd

    def _extract_parameters(self, transcript):
        """Extract parameters from the voice command"""
        parameters = []

        # Extract numbers (distances, counts, etc.)
        numbers = re.findall(r'\d+', transcript)
        for num in numbers:
            parameters.append(f"number:{num}")

        # Extract directions
        directions = ['forward', 'backward', 'left', 'right', 'up', 'down', 'north', 'south', 'east', 'west']
        for direction in directions:
            if direction in transcript.lower():
                parameters.append(f"direction:{direction}")

        # Extract objects
        objects = ['cup', 'bottle', 'box', 'table', 'chair', 'kitchen', 'bedroom', 'living room', 'red', 'blue', 'green']
        for obj in objects:
            if obj in transcript.lower():
                parameters.append(f"object:{obj}")

        # Extract locations
        locations = ['kitchen', 'bedroom', 'living room', 'bathroom', 'office', 'dining room']
        for loc in locations:
            if loc in transcript.lower():
                parameters.append(f"location:{loc}")

        return parameters

    def _is_command_potentially_harmful(self, transcript):
        """Check if the command might be potentially harmful"""
        harmful_keywords = [
            'shutdown', 'terminate', 'kill', 'destroy', 'break', 'harm', 'injure',
            'danger', 'emergency', 'crash', 'stop all systems', 'disable safety'
        ]

        transcript_lower = transcript.lower()
        for keyword in harmful_keywords:
            if keyword in transcript_lower:
                return True

        return False

    def _send_to_planning_service(self, cmd):
        """Send the processed voice command to the cognitive planning service"""
        # Create a request for the planning service
        request = PlanActions.Request()
        request.command = cmd

        # Make an asynchronous call to the planning service
        future = self.plan_client.call_async(request)
        future.add_done_callback(self._planning_response_callback)

    def _planning_response_callback(self, future):
        """Handle response from the planning service"""
        try:
            response = future.result()
            if response:
                self.get_logger().info('Successfully sent command to planning service')
            else:
                self.get_logger().error('Planning service returned empty response')
        except Exception as e:
            self.get_logger().error(f'Error calling planning service: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()