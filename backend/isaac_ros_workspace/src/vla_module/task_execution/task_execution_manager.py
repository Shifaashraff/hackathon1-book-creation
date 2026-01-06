#!/usr/bin/env python3
"""
Task execution manager for the Vision-Language-Action (VLA) system.
Manages the execution of cognitive plans by interfacing with ROS 2 action servers.
"""

import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action.server import ServerGoalHandle

from vla_msgs.msg import CognitivePlan, TaskExecutionState
from vla_msgs.action import ExecuteTask
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import threading
import time
from typing import Dict, List, Optional


class TaskExecutionManager(Node):
    def __init__(self):
        super().__init__('task_execution_manager')

        # Create action server for executing tasks
        self._action_server = ActionServer(
            self,
            ExecuteTask,
            '/vla/execute_task',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # Publishers
        self.execution_state_pub = self.create_publisher(
            TaskExecutionState,
            '/vla/task_execution_state',
            10
        )

        # Action clients for navigation and manipulation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manip_client = ActionClient(self, FollowJointTrajectory, 'follow_joint_trajectory')

        # Internal state
        self.current_plan = None
        self.execution_state = TaskExecutionState()
        self.is_executing = False
        self._goal_handle = None

        # Initialize execution state
        self._initialize_execution_state()

        self.get_logger().info('Task Execution Manager initialized')

    def goal_callback(self, goal_request):
        """Accept or reject a goal request"""
        self.get_logger().info('Received task execution request')

        # Check if we can accept the goal
        if self.is_executing:
            self.get_logger().info('Rejecting new goal, currently executing')
            return GoalResponse.REJECT

        # Accept the goal
        self.get_logger().info('Accepting task execution goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def _initialize_execution_state(self):
        """Initialize the execution state"""
        self.execution_state.id = f"execution_{self.get_clock().now().nanoseconds}"
        self.execution_state.cognitive_plan_id = ""
        self.execution_state.current_step = ""
        self.execution_state.completed_steps = []
        self.execution_state.failed_steps = []
        self.execution_state.progress = 0.0
        self.execution_state.status = "IDLE"
        self.execution_state.error_message = ""
        self.execution_state.feedback = "Ready to execute tasks"
        self.execution_state.start_time = self.get_clock().now().to_msg()
        self.execution_state.current_time = self.get_clock().now().to_msg()

    def execute_callback(self, goal_handle):
        """Execute the goal and provide feedback"""
        self.get_logger().info('Executing task execution goal')

        # Set execution state
        self.is_executing = True
        self._goal_handle = goal_handle
        self.current_plan = goal_handle.request.plan

        try:
            # Create feedback message
            feedback_msg = ExecuteTask.Feedback()
            feedback_msg.feedback = self.execution_state

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            # Execute the plan
            final_state = self._execute_plan(self.current_plan, feedback_msg, goal_handle)

            # Create result
            result = ExecuteTask.Result()
            result.result = final_state

            # Update feedback before completing
            feedback_msg.feedback = final_state
            goal_handle.publish_feedback(feedback_msg)

            # Succeed the goal
            goal_handle.succeed()

            self.get_logger().info('Task execution goal completed successfully')

            return result

        except Exception as e:
            self.get_logger().error(f'Error during task execution: {e}')

            # Create result with final state
            result = ExecuteTask.Result()
            self.execution_state.status = "FAILED"
            self.execution_state.error_message = str(e)
            self.execution_state.feedback = f"Execution error: {e}"
            result.result = self.execution_state

            # Abort the goal
            goal_handle.abort()

            return result

        finally:
            # Reset execution state
            self.is_executing = False
            self._goal_handle = None

    def _execute_plan(self, plan, feedback_msg, goal_handle):
        """Execute the cognitive plan"""
        try:
            self.current_plan = plan
            self.execution_state.cognitive_plan_id = plan.id
            self.execution_state.status = "EXECUTING"
            self.execution_state.progress = 0.0
            self.execution_state.completed_steps = []
            self.execution_state.failed_steps = []
            self.execution_state.start_time = self.get_clock().now().to_msg()

            # Publish initial state
            feedback_msg.feedback = self.execution_state
            goal_handle.publish_feedback(feedback_msg)

            for i, action_step in enumerate(plan.actions):
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Execution cancelled by user')
                    self.execution_state.status = "CANCELLED"
                    self.execution_state.feedback = "Execution cancelled by user"
                    break

                self.get_logger().info(f'Executing action {i+1}/{len(plan.actions)}: {action_step.action_type}')

                # Update current step
                self.execution_state.current_step = action_step.id
                self.execution_state.feedback = f'Executing: {action_step.action_type} - {action_step.action_type} action'

                # Publish feedback
                feedback_msg.feedback = self.execution_state
                goal_handle.publish_feedback(feedback_msg)

                # Execute the action
                success = self._execute_action_step(action_step)

                if success:
                    # Mark as completed
                    self.execution_state.completed_steps.append(action_step.id)
                    self.get_logger().info(f'Action {action_step.id} completed successfully')
                else:
                    # Mark as failed
                    self.execution_state.failed_steps.append(action_step.id)
                    self.get_logger().error(f'Action {action_step.id} failed')
                    break  # Stop execution on failure for now

                # Update progress
                self.execution_state.progress = len(self.execution_state.completed_steps) / len(plan.actions)

                # Publish feedback
                feedback_msg.feedback = self.execution_state
                goal_handle.publish_feedback(feedback_msg)

            # Plan execution completed
            if len(self.execution_state.failed_steps) == 0 and self.execution_state.status != "CANCELLED":
                self.execution_state.status = "COMPLETED"
                self.execution_state.feedback = f"Plan completed successfully with {len(self.execution_state.completed_steps)} actions"
            elif self.execution_state.status != "CANCELLED":
                self.execution_state.status = "FAILED"
                self.execution_state.feedback = f"Plan failed with {len(self.execution_state.failed_steps)} failed actions"

            # Update final state
            self.execution_state.current_time = self.get_clock().now().to_msg()
            self._publish_execution_state()

            return self.execution_state

        except Exception as e:
            self.get_logger().error(f'Error during plan execution: {e}')
            self.execution_state.status = "FAILED"
            self.execution_state.error_message = str(e)
            self.execution_state.feedback = f"Execution error: {e}"
            self.execution_state.current_time = self.get_clock().now().to_msg()
            self._publish_execution_state()
            return self.execution_state

    def _execute_action_step(self, action_step):
        """Execute a single action step"""
        try:
            self.get_logger().info(f'Executing action: {action_step.action_type} with parameters: {action_step.parameters}')

            if action_step.action_type == "NAVIGATION":
                return self._execute_navigation_action(action_step)
            elif action_step.action_type == "MANIPULATION":
                return self._execute_manipulation_action(action_step)
            elif action_step.action_type == "PERCEPTION":
                return self._execute_perception_action(action_step)
            else:
                self.get_logger().warning(f'Unknown action type: {action_step.action_type}')
                return self._execute_generic_action(action_step)

        except Exception as e:
            self.get_logger().error(f'Error executing action {action_step.action_type}: {e}')
            return False

    def _execute_navigation_action(self, action_step):
        """Execute navigation action"""
        self.get_logger().info('Executing navigation action')

        # Extract navigation parameters
        goal_x = 0.0
        goal_y = 0.0
        goal_z = 0.0
        goal_w = 1.0

        for param in action_step.parameters:
            if param.startswith('goal_position_x:'):
                goal_x = float(param.split(':')[1])
            elif param.startswith('goal_position_y:'):
                goal_y = float(param.split(':')[1])
            elif param.startswith('goal_position_z:'):
                goal_z = float(param.split(':')[1])
            elif param.startswith('goal_orientation_w:'):
                goal_w = float(param.split(':')[1])

        # Create navigation goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.position.z = goal_z
        goal.pose.pose.orientation.w = goal_w

        # Send navigation goal (with timeout)
        if self.nav_client.wait_for_server(timeout_sec=1.0):
            future = self.nav_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=action_step.timeout.sec)

            # Check if goal was accepted and completed
            if future.result() is not None:
                return True
            else:
                self.get_logger().error('Navigation goal was not completed in time')
                return False
        else:
            self.get_logger().error('Navigation action server not available')
            return False

    def _execute_manipulation_action(self, action_step):
        """Execute manipulation action"""
        self.get_logger().info('Executing manipulation action')

        # For now, simulate successful execution
        # In a real system, this would interface with manipulator action servers
        time.sleep(2.0)  # Simulate execution time

        return True

    def _execute_perception_action(self, action_step):
        """Execute perception action"""
        self.get_logger().info('Executing perception action')

        # For now, simulate successful execution
        # In a real system, this would interface with perception action servers
        time.sleep(1.0)  # Simulate execution time

        return True

    def _execute_generic_action(self, action_step):
        """Execute generic action"""
        self.get_logger().info(f'Executing generic action: {action_step.action_type}')

        # For unknown action types, simulate execution
        time.sleep(1.0)  # Simulate execution time

        return True

    def _publish_execution_state(self):
        """Publish the current execution state"""
        self.execution_state.current_time = self.get_clock().now().to_msg()
        self.execution_state_pub.publish(self.execution_state)

    def cancel_execution(self):
        """Cancel current execution"""
        self.get_logger().info('Cancelling current execution')
        self.is_executing = False
        self.execution_state.status = "CANCELLED"
        self.execution_state.feedback = "Execution cancelled by user"
        self._publish_execution_state()

    def reset_execution(self):
        """Reset execution state"""
        self.get_logger().info('Resetting execution state')
        self.current_plan = None
        self.is_executing = False
        self._initialize_execution_state()
        self._publish_execution_state()


def main(args=None):
    rclpy.init(args=args)

    node = TaskExecutionManager()

    # Use a multi-threaded executor to handle callbacks properly
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Task Execution Manager spinning...')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, cancelling execution and shutting down...')
        node.cancel_execution()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()