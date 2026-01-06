#!/usr/bin/env python3
"""
Cognitive planning server for the Vision-Language-Action (VLA) system.
Implements the ROS 2 action server for planning cognitive actions from voice commands.
"""

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from vla_msgs.action import PlanActions
from vla_msgs.msg import VoiceCommand, CognitivePlan
import threading
import time


class CognitivePlanningServer(Node):
    def __init__(self):
        super().__init__('cognitive_planning_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            PlanActions,
            'vla/plan_actions',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # Publishers
        self.plan_pub = self.create_publisher(CognitivePlan, '/vla/cognitive_plan', 10)

        # Internal state
        self._goal_handle = None
        self._is_executing = False

        self.get_logger().info('Cognitive Planning Server initialized')

    def goal_callback(self, goal_request):
        """Accept or reject a goal request"""
        self.get_logger().info('Received goal request to plan actions')

        # Check if we can accept the goal
        if self._is_executing:
            self.get_logger().info('Rejecting new goal, currently executing')
            return GoalResponse.REJECT

        # Accept the goal
        self.get_logger().info('Accepting goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal and provide feedback"""
        self.get_logger().info('Executing planning goal')

        # Set execution state
        self._is_executing = True
        self._goal_handle = goal_handle

        try:
            # Get the voice command from the goal
            voice_command = goal_handle.request.command

            # Create feedback message
            feedback_msg = PlanActions.Feedback()
            feedback_msg.feedback = "Starting cognitive planning process"

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            # Plan the actions using cognitive planning algorithms
            cognitive_plan = self.plan_actions(voice_command, feedback_msg, goal_handle)

            # Create result
            result = PlanActions.Result()
            result.result = cognitive_plan

            # Publish the plan
            self.plan_pub.publish(cognitive_plan)

            # Update feedback before completing
            feedback_msg.feedback = "Planning completed successfully"
            goal_handle.publish_feedback(feedback_msg)

            # Succeed the goal
            goal_handle.succeed()

            self.get_logger().info('Planning goal completed successfully')

            return result

        except Exception as e:
            self.get_logger().error(f'Error during planning: {e}')

            # Create result with empty plan
            result = PlanActions.Result()
            result.result = self.create_empty_plan(goal_handle.request.command)

            # Abort the goal
            goal_handle.abort()

            return result
        finally:
            # Reset execution state
            self._is_executing = False
            self._goal_handle = None

    def plan_actions(self, voice_command, feedback_msg, goal_handle):
        """Plan actions using cognitive planning algorithms"""
        self.get_logger().info(f'Planning actions for command: "{voice_command.transcript}"')

        # Update feedback
        feedback_msg.feedback = "Analyzing voice command"
        goal_handle.publish_feedback(feedback_msg)

        # Simulate processing time
        time.sleep(0.5)

        # Create the cognitive plan
        plan = CognitivePlan()
        plan.id = f"plan_{self.get_clock().now().nanoseconds}"
        plan.voice_command_id = voice_command.id
        plan.timestamp = self.get_clock().now().to_msg()
        plan.status = "PLANNING"

        # Update feedback
        feedback_msg.feedback = "Generating action sequence"
        goal_handle.publish_feedback(feedback_msg)

        # Generate action steps based on the voice command
        action_steps = self._generate_action_steps(voice_command, feedback_msg, goal_handle)
        plan.actions = action_steps

        # Set plan status to READY
        plan.status = "READY"

        # Set estimated duration
        plan.estimated_duration.sec = len(action_steps) * 5  # 5 seconds per action
        plan.estimated_duration.nanosec = 0

        # Set required capabilities
        plan.required_capabilities = self._determine_required_capabilities(action_steps)

        # Update feedback
        feedback_msg.feedback = f"Plan generated with {len(action_steps)} actions"
        goal_handle.publish_feedback(feedback_msg)

        return plan

    def _generate_action_steps(self, voice_command, feedback_msg, goal_handle):
        """Generate action steps based on the voice command"""
        import sys
        sys.path.append('../cognitive_planning')  # Add cognitive_planning to path
        from command_parser import CommandParserNode

        # Create a temporary parser to generate actions
        # In a real implementation, this would be a shared service
        parser = CommandParserNode()
        # Since we can't directly call the method, we'll implement basic parsing here
        return self._basic_command_parsing(voice_command)

    def _basic_command_parsing(self, voice_command):
        """Basic command parsing to generate action steps"""
        from vla_msgs.msg import ActionStep
        transcript = voice_command.transcript.lower().strip()
        action_steps = []

        # Navigation commands
        if any(word in transcript for word in ['move', 'go', 'forward', 'backward', 'left', 'right', 'turn']):
            action = ActionStep()
            action.id = f"nav_action_{self.get_clock().now().nanoseconds}"
            action.action_type = "NAVIGATION"
            action.parameters = ["move_forward", "distance:1m"]
            action.priority = 1
            action.dependencies = []
            action.timeout.sec = 30
            action.timeout.nanosec = 0
            action.success_criteria = ["navigation_completed"]
            action_steps.append(action)

        # Manipulation commands
        elif any(word in transcript for word in ['pick', 'grab', 'lift', 'take', 'hold', 'drop']):
            action = ActionStep()
            action.id = f"manip_action_{self.get_clock().now().nanoseconds}"
            action.action_type = "MANIPULATION"
            action.parameters = ["grasp_object", "object:unknown"]
            action.priority = 1
            action.dependencies = []
            action.timeout.sec = 45
            action.timeout.nanosec = 0
            action.success_criteria = ["manipulation_completed"]
            action_steps.append(action)

        # Perception commands
        elif any(word in transcript for word in ['find', 'look', 'see', 'locate', 'search']):
            action = ActionStep()
            action.id = f"percept_action_{self.get_clock().now().nanoseconds}"
            action.action_type = "PERCEPTION"
            action.parameters = ["detect_object", "target:unknown"]
            action.priority = 1
            action.dependencies = []
            action.timeout.sec = 30
            action.timeout.nanosec = 0
            action.success_criteria = ["perception_completed"]
            action_steps.append(action)

        # Default: create a simple action
        if not action_steps:
            action = ActionStep()
            action.id = f"default_action_{self.get_clock().now().nanoseconds}"
            action.action_type = "UNKNOWN"
            action.parameters = [f"command:{voice_command.transcript}"]
            action.priority = 1
            action.dependencies = []
            action.timeout.sec = 15
            action.timeout.nanosec = 0
            action.success_criteria = ["command_processed"]
            action_steps.append(action)

        return action_steps

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

        return list(capabilities)

    def create_empty_plan(self, voice_command):
        """Create an empty plan in case of errors"""
        from vla_msgs.msg import CognitivePlan
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

    node = CognitivePlanningServer()

    # Use a multi-threaded executor to handle callbacks properly
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Cognitive Planning Server spinning...')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()