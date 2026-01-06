#!/usr/bin/env python3
"""
Command parser module for the Vision-Language-Action (VLA) system.
Parses voice commands into structured intents and parameters for cognitive planning.
"""

import rclpy
from rclpy.node import Node
from vla_msgs.msg import VoiceCommand, CognitivePlan, ActionStep
from vla_msgs.srv import PlanActions
import re
from typing import Dict, List, Tuple


class CommandParserNode(Node):
    def __init__(self):
        super().__init__('command_parser_node')

        # Service for planning actions
        self.plan_service = self.create_service(
            PlanActions,
            '/vla/plan_actions',
            self.plan_actions_callback
        )

        self.get_logger().info('Command Parser Node initialized')

    def plan_actions_callback(self, request, response):
        """Callback for the PlanActions service"""
        self.get_logger().info(f'Received command for planning: "{request.command.transcript}"')

        try:
            # Parse the voice command into a cognitive plan
            cognitive_plan = self.parse_command_to_plan(request.command)

            # Set the plan in the response
            response.result = cognitive_plan

            self.get_logger().info(f'Generated plan with {len(cognitive_plan.actions)} actions')
        except Exception as e:
            self.get_logger().error(f'Error generating plan: {e}')
            # Return an empty plan in case of error
            response.result = self.create_empty_plan(request.command)

        return response

    def parse_command_to_plan(self, voice_command):
        """Parse a voice command into a cognitive plan with action steps"""
        # Create the cognitive plan
        plan = CognitivePlan()
        plan.id = f"plan_{self.get_clock().now().nanoseconds}"
        plan.voice_command_id = voice_command.id
        plan.timestamp = self.get_clock().now().to_msg()
        plan.status = "PLANNING"

        # Parse the command and generate action steps
        action_steps = self._generate_action_steps(voice_command)
        plan.actions = action_steps

        # Set plan status to READY once actions are generated
        plan.status = "READY"

        # Set estimated duration (simplified)
        plan.estimated_duration.sec = len(action_steps) * 5  # 5 seconds per action
        plan.estimated_duration.nanosec = 0

        # Set required capabilities based on actions
        plan.required_capabilities = self._get_required_capabilities(action_steps)

        return plan

    def _generate_action_steps(self, voice_command):
        """Generate action steps based on the voice command"""
        transcript = voice_command.transcript.lower().strip()
        action_steps = []

        # Navigation commands
        if any(word in transcript for word in ['move', 'go', 'forward', 'backward', 'left', 'right', 'turn']):
            action_steps.extend(self._parse_navigation_command(transcript))

        # Manipulation commands
        elif any(word in transcript for word in ['pick', 'grab', 'lift', 'take', 'hold', 'drop']):
            action_steps.extend(self._parse_manipulation_command(transcript))

        # Perception commands
        elif any(word in transcript for word in ['find', 'look', 'see', 'locate', 'search']):
            action_steps.extend(self._parse_perception_command(transcript))

        # Complex multi-step commands
        else:
            action_steps.extend(self._parse_complex_command(transcript))

        # If no specific actions were generated, create a default action
        if not action_steps:
            default_action = self._create_default_action(voice_command.transcript)
            action_steps.append(default_action)

        return action_steps

    def _parse_navigation_command(self, transcript):
        """Parse navigation commands into action steps"""
        actions = []

        # Check for movement commands
        if 'forward' in transcript or 'ahead' in transcript:
            distance = self._extract_distance(transcript) or 1  # Default to 1 meter
            action = self._create_action_step(
                "NAVIGATION",
                f"move_forward_distance:{distance}",
                1,  # Priority
                [],  # Dependencies
                30  # Timeout in seconds
            )
            actions.append(action)

        elif 'backward' in transcript or 'back' in transcript:
            distance = self._extract_distance(transcript) or 1
            action = self._create_action_step(
                "NAVIGATION",
                f"move_backward_distance:{distance}",
                1,
                [],
                30
            )
            actions.append(action)

        elif 'turn left' in transcript or 'left' in transcript:
            action = self._create_action_step(
                "NAVIGATION",
                "turn_left:90",
                1,
                [],
                15
            )
            actions.append(action)

        elif 'turn right' in transcript or 'right' in transcript:
            action = self._create_action_step(
                "NAVIGATION",
                "turn_right:90",
                1,
                [],
                15
            )
            actions.append(action)

        elif 'go to' in transcript or 'move to' in transcript:
            location = self._extract_location(transcript)
            if location:
                action = self._create_action_step(
                    "NAVIGATION",
                    f"navigate_to:{location}",
                    1,
                    [],
                    60
                )
                actions.append(action)

        return actions

    def _parse_manipulation_command(self, transcript):
        """Parse manipulation commands into action steps"""
        actions = []

        # Look for object to manipulate
        obj = self._extract_object(transcript)

        if 'pick' in transcript or 'grab' in transcript or 'take' in transcript:
            action = self._create_action_step(
                "MANIPULATION",
                f"pick_object:{obj}" if obj else "pick_object:unknown",
                1,
                ["NAVIGATION"],  # Need to navigate to object first
                45
            )
            actions.append(action)

        elif 'drop' in transcript or 'put' in transcript:
            action = self._create_action_step(
                "MANIPULATION",
                f"drop_object:{obj}" if obj else "drop_object:unknown",
                1,
                [],
                30
            )
            actions.append(action)

        return actions

    def _parse_perception_command(self, transcript):
        """Parse perception commands into action steps"""
        actions = []

        obj = self._extract_object(transcript)
        location = self._extract_location(transcript)

        if 'find' in transcript or 'locate' in transcript:
            action = self._create_action_step(
                "PERCEPTION",
                f"find_object:{obj}" if obj else f"find_object_in:{location}" if location else "find_object:unknown",
                1,
                [],
                45
            )
            actions.append(action)

        elif 'look' in transcript or 'see' in transcript:
            action = self._create_action_step(
                "PERCEPTION",
                f"look_for:{obj}" if obj else "look_for:unknown",
                1,
                [],
                30
            )
            actions.append(action)

        return actions

    def _parse_complex_command(self, transcript):
        """Parse complex multi-step commands"""
        actions = []

        # Check for sequence of commands (e.g., "go to kitchen and pick up the red cup")
        if 'and' in transcript:
            parts = transcript.split('and')
            for i, part in enumerate(parts):
                part = part.strip()
                if 'go to' in part or 'move to' in part:
                    location = self._extract_location(part)
                    if location:
                        action = self._create_action_step(
                            "NAVIGATION",
                            f"navigate_to:{location}",
                            i + 1,
                            [] if i == 0 else [actions[-1].id],  # Previous action as dependency
                            60
                        )
                        action.id = f"action_{i+1}_nav"
                        actions.append(action)
                elif any(word in part for word in ['pick', 'grab', 'take']):
                    obj = self._extract_object(part)
                    action = self._create_action_step(
                        "MANIPULATION",
                        f"pick_object:{obj}" if obj else "pick_object:unknown",
                        i + 1,
                        [actions[-1].id] if actions else [],  # Previous action as dependency
                        45
                    )
                    action.id = f"action_{i+1}_manip"
                    actions.append(action)

        return actions

    def _create_action_step(self, action_type, parameters_str, priority, dependencies, timeout_sec):
        """Create an ActionStep message"""
        action = ActionStep()
        action.id = f"action_{self.get_clock().now().nanoseconds}"
        action.action_type = action_type
        action.parameters = [parameters_str]
        action.priority = priority
        action.dependencies = dependencies
        action.timeout.sec = timeout_sec
        action.timeout.nanosec = 0
        action.success_criteria = [f"{action_type.lower()}_completed"]

        return action

    def _create_default_action(self, transcript):
        """Create a default action when command parsing fails"""
        action = ActionStep()
        action.id = f"default_action_{self.get_clock().now().nanoseconds}"
        action.action_type = "UNKNOWN"
        action.parameters = [f"command:{transcript}"]
        action.priority = 1
        action.dependencies = []
        action.timeout.sec = 30
        action.timeout.nanosec = 0
        action.success_criteria = ["command_processed"]

        return action

    def _extract_distance(self, transcript):
        """Extract distance value from transcript"""
        # Look for patterns like "2 meters", "3 feet", etc.
        match = re.search(r'(\d+)\s*(meter|m|foot|feet|step|steps)', transcript)
        if match:
            return int(match.group(1))
        return None

    def _extract_object(self, transcript):
        """Extract object name from transcript"""
        # Common objects that might be referenced
        objects = ['cup', 'bottle', 'box', 'ball', 'book', 'chair', 'table', 'red cup', 'blue bottle', 'green object']
        for obj in objects:
            if obj in transcript:
                return obj
        return None

    def _extract_location(self, transcript):
        """Extract location from transcript"""
        # Common locations
        locations = ['kitchen', 'bedroom', 'living room', 'bathroom', 'office', 'dining room', 'hallway']
        for loc in locations:
            if loc in transcript:
                return loc
        return None

    def _get_required_capabilities(self, action_steps):
        """Determine required robot capabilities based on action steps"""
        capabilities = set()

        for action in action_steps:
            if action.action_type == "NAVIGATION":
                capabilities.add("navigation")
            elif action.action_type == "MANIPULATION":
                capabilities.add("manipulation")
            elif action.action_type == "PERCEPTION":
                capabilities.add("perception")

        return list(capabilities)

    def create_empty_plan(self, voice_command):
        """Create an empty plan in case of errors"""
        plan = CognitivePlan()
        plan.id = f"empty_plan_{self.get_clock().now().nanoseconds}"
        plan.voice_command_id = voice_command.id
        plan.timestamp = self.get_clock().now().to_msg()
        plan.status = "FAILED"
        plan.actions = []
        plan.estimated_duration.sec = 0
        plan.estimated_duration.nanosec = 0
        plan.required_capabilities = []

        return plan


def main(args=None):
    rclpy.init(args=args)
    node = CommandParserNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()