#!/usr/bin/env python3
"""
Task planner module for the Vision-Language-Action (VLA) system.
Implements cognitive planning algorithms to convert voice commands into executable action sequences.
"""

import rclpy
from rclpy.node import Node
from vla_msgs.msg import VoiceCommand, CognitivePlan, ActionStep
from vla_msgs.srv import PlanActions
from vla_msgs.srv import ValidateCapability
import time
from typing import List, Dict, Any


class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner_node')

        # Service for planning actions
        self.plan_service = self.create_service(
            PlanActions,
            '/vla/plan_actions',
            self.plan_actions_callback
        )

        # Client for validating capabilities
        self.capability_client = self.create_client(ValidateCapability, '/vla/validate_capability')
        while not self.capability_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Capability validation service not available, waiting...')

        # Configuration parameters
        self.max_plan_steps = self.declare_parameter('max_plan_steps', 10).value
        self.enable_validation = self.declare_parameter('enable_validation', True).value

        self.get_logger().info('Task Planner Node initialized')

    def plan_actions_callback(self, request, response):
        """Callback for the PlanActions service"""
        self.get_logger().info(f'Received command for planning: "{request.command.transcript}"')

        try:
            # Plan the actions using cognitive planning algorithms
            cognitive_plan = self.plan_actions(request.command)

            # Validate the plan
            if self.enable_validation:
                cognitive_plan = self.validate_plan(cognitive_plan)

            # Set the plan in the response
            response.result = cognitive_plan

            self.get_logger().info(f'Planned {len(cognitive_plan.actions)} actions for command: "{request.command.transcript}"')
        except Exception as e:
            self.get_logger().error(f'Error generating plan: {e}')
            # Return an empty plan in case of error
            response.result = self.create_empty_plan(request.command)

        return response

    def plan_actions(self, voice_command):
        """Plan actions using cognitive planning algorithms"""
        # Create the cognitive plan
        plan = CognitivePlan()
        plan.id = f"plan_{self.get_clock().now().nanoseconds}"
        plan.voice_command_id = voice_command.id
        plan.timestamp = self.get_clock().now().to_msg()
        plan.status = "PLANNING"

        # Generate action steps using hierarchical task network (HTN) approach
        action_steps = self._generate_hierarchical_plan(voice_command)
        plan.actions = action_steps[:self.max_plan_steps]  # Limit to max steps

        # Set plan status to READY once actions are generated
        plan.status = "READY"

        # Set estimated duration based on number of actions
        plan.estimated_duration.sec = len(plan.actions) * 10  # 10 seconds per action as estimate
        plan.estimated_duration.nanosec = 0

        # Set required capabilities based on actions
        plan.required_capabilities = self._determine_required_capabilities(plan.actions)

        return plan

    def _generate_hierarchical_plan(self, voice_command):
        """Generate a hierarchical plan using HTN approach"""
        transcript = voice_command.transcript.lower().strip()
        action_steps = []

        # Parse the command and generate subtasks
        subtasks = self._decompose_command(transcript)

        # Convert subtasks to action steps with dependencies
        for i, (action_type, params) in enumerate(subtasks):
            # Determine dependencies (previous action must complete first)
            dependencies = [action_steps[j].id for j in range(i)] if i > 0 else []

            action_step = self._create_action_step(
                action_type=action_type,
                parameters=params,
                priority=i+1,
                dependencies=dependencies,
                timeout_sec=60
            )
            action_steps.append(action_step)

        return action_steps

    def _decompose_command(self, transcript):
        """Decompose a command into subtasks using hierarchical decomposition"""
        subtasks = []

        # Handle complex commands that need decomposition
        if 'and' in transcript or 'then' in transcript:
            # Split command by 'and' or 'then' to create sequence of subtasks
            parts = []
            if 'and' in transcript:
                parts = [part.strip() for part in transcript.split('and')]
            elif 'then' in transcript:
                parts = [part.strip() for part in transcript.split('then')]

            for part in parts:
                subtasks.extend(self._parse_single_command(part))
        else:
            # Single command
            subtasks.extend(self._parse_single_command(transcript))

        return subtasks

    def _parse_single_command(self, command):
        """Parse a single command into one or more action types"""
        command_lower = command.lower()
        subtasks = []

        # Navigation commands
        if any(word in command_lower for word in ['move', 'go', 'navigate', 'walk', 'run', 'drive']):
            # Determine specific navigation action
            if any(word in command_lower for word in ['forward', 'ahead', 'straight']):
                subtasks.append(('NAVIGATION', ['move_forward', 'distance:1m']))
            elif any(word in command_lower for word in ['backward', 'back', 'reverse']):
                subtasks.append(('NAVIGATION', ['move_backward', 'distance:1m']))
            elif any(word in command_lower for word in ['left', 'turn left']):
                subtasks.append(('NAVIGATION', ['turn', 'direction:left', 'angle:90']))
            elif any(word in command_lower for word in ['right', 'turn right']):
                subtasks.append(('NAVIGATION', ['turn', 'direction:right', 'angle:90']))
            elif 'to' in command_lower:
                # Navigate to a location
                location = self._extract_location(command_lower)
                if location:
                    subtasks.append(('NAVIGATION', ['navigate_to', f'location:{location}']))
                else:
                    subtasks.append(('NAVIGATION', ['navigate_to', 'location:unknown']))
            else:
                subtasks.append(('NAVIGATION', ['move_forward', 'distance:1m']))

        # Manipulation commands
        elif any(word in command_lower for word in ['pick', 'grab', 'lift', 'take', 'hold', 'get', 'put', 'drop', 'place']):
            obj = self._extract_object(command_lower)
            if any(word in command_lower for word in ['pick', 'grab', 'take', 'get']):
                subtasks.append(('MANIPULATION', ['pick_object', f'object:{obj}' if obj else 'object:unknown']))
            elif any(word in command_lower for word in ['put', 'drop', 'place']):
                subtasks.append(('MANIPULATION', ['place_object', f'object:{obj}' if obj else 'object:unknown']))

        # Perception commands
        elif any(word in command_lower for word in ['find', 'look', 'see', 'locate', 'search', 'detect', 'identify']):
            obj = self._extract_object(command_lower)
            if obj:
                subtasks.append(('PERCEPTION', ['detect_object', f'object:{obj}']))
            else:
                subtasks.append(('PERCEPTION', ['scan_environment']))

        # Other commands
        else:
            # Default to a simple action
            subtasks.append(('UNKNOWN', ['process_command', f'command:{command}']))

        return subtasks

    def _create_action_step(self, action_type, parameters, priority, dependencies, timeout_sec):
        """Create an ActionStep with the given parameters"""
        action = ActionStep()
        action.id = f"action_{self.get_clock().now().nanoseconds}_{priority}"
        action.action_type = action_type
        action.parameters = parameters
        action.priority = priority
        action.dependencies = dependencies
        action.timeout.sec = timeout_sec
        action.timeout.nanosec = 0
        action.success_criteria = self._determine_success_criteria(action_type, parameters)

        return action

    def _determine_success_criteria(self, action_type, parameters):
        """Determine success criteria for an action"""
        criteria = []

        if action_type == "NAVIGATION":
            criteria.append("navigation_completed")
            criteria.append("goal_reached")
        elif action_type == "MANIPULATION":
            criteria.append("manipulation_completed")
            criteria.append("object_grasped" if any("pick" in p for p in parameters) else "object_released")
        elif action_type == "PERCEPTION":
            criteria.append("perception_completed")
            criteria.append("object_detected" if any("detect" in p for p in parameters) else "environment_scanned")
        else:
            criteria.append("action_completed")

        return criteria

    def _determine_required_capabilities(self, action_steps):
        """Determine required robot capabilities based on action steps"""
        capabilities = set()

        for action in action_steps:
            if action.action_type == "NAVIGATION":
                capabilities.add("navigation")
                capabilities.add("path_planning")
            elif action.action_type == "MANIPULATION":
                capabilities.add("manipulation")
                capabilities.add("gripper_control")
            elif action.action_type == "PERCEPTION":
                capabilities.add("perception")
                capabilities.add("object_detection")
            elif action.action_type == "UNKNOWN":
                # Default capabilities that might be needed
                capabilities.add("navigation")
                capabilities.add("perception")

        return list(capabilities)

    def validate_plan(self, plan):
        """Validate the cognitive plan before execution"""
        self.get_logger().info(f'Validating plan with {len(plan.actions)} actions')

        # Validate each action in the plan
        valid_actions = []
        for action in plan.actions:
            if self._validate_action(action):
                valid_actions.append(action)
            else:
                self.get_logger().warning(f'Invalid action removed: {action.action_type}')

        # Update the plan with valid actions only
        plan.actions = valid_actions

        # Validate required capabilities
        if self.enable_validation:
            valid_capabilities = []
            for capability in plan.required_capabilities:
                if self._validate_capability(capability):
                    valid_capabilities.append(capability)
                else:
                    self.get_logger().warning(f'Capability not available: {capability}')

            plan.required_capabilities = valid_capabilities

        # Update plan status based on validation
        if len(plan.actions) > 0:
            plan.status = "READY"
        else:
            plan.status = "FAILED"

        return plan

    def _validate_action(self, action):
        """Validate a single action"""
        # Check if action type is valid
        valid_types = ["NAVIGATION", "MANIPULATION", "PERCEPTION", "UNKNOWN"]
        if action.action_type not in valid_types:
            return False

        # Check if priority is valid
        if action.priority <= 0:
            return False

        # Check if timeout is valid
        if action.timeout.sec <= 0:
            return False

        # Check if parameters are valid
        if not action.parameters:
            return False

        return True

    def _validate_capability(self, capability):
        """Validate if a robot capability is available"""
        # Create a request for the capability validation service
        request = ValidateCapability.Request()
        request.capability_name = capability

        # Make a synchronous call to the validation service
        future = self.capability_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
            return response.available
        except Exception as e:
            self.get_logger().error(f'Error validating capability {capability}: {e}')
            return False  # Assume capability is not available if validation fails

    def _extract_object(self, command):
        """Extract object name from command"""
        # Common objects that might be referenced
        objects = [
            'cup', 'bottle', 'box', 'ball', 'book', 'chair', 'table', 'red cup', 'blue bottle',
            'green object', 'red ball', 'blue book', 'object', 'item', 'thing'
        ]

        command_lower = command.lower()
        for obj in objects:
            if obj in command_lower:
                return obj

        # Extract any color + object pattern
        import re
        color_obj_match = re.search(r'(red|blue|green|yellow|black|white)\s+(\w+)', command_lower)
        if color_obj_match:
            return f"{color_obj_match.group(1)} {color_obj_match.group(2)}"

        return None

    def _extract_location(self, command):
        """Extract location from command"""
        # Common locations
        locations = [
            'kitchen', 'bedroom', 'living room', 'bathroom', 'office', 'dining room',
            'hallway', 'garage', 'garden', 'dining room', 'study', 'playroom'
        ]

        command_lower = command.lower()
        for loc in locations:
            if loc in command_lower:
                return loc

        return None

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
    node = TaskPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()