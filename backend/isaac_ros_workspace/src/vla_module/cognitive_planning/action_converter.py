#!/usr/bin/env python3
"""
Action converter module for the Vision-Language-Action (VLA) system.
Converts cognitive plans into ROS 2 action messages for robot execution.
"""

import rclpy
from rclpy.node import Node
from vla_msgs.msg import CognitivePlan, ActionStep, TaskExecutionState
from vla_msgs.srv import PlanActions
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
import math


class ActionConverterNode(Node):
    def __init__(self):
        super().__init__('action_converter_node')

        # Service for planning actions (to integrate with the planning flow)
        self.plan_service = self.create_service(
            PlanActions,
            '/vla/plan_actions',
            self.plan_actions_callback
        )

        # Publisher for execution state updates
        self.execution_state_pub = self.create_publisher(
            TaskExecutionState,
            '/vla/task_execution_state',
            10
        )

        # Configuration parameters
        self.enable_detailed_conversion = self.declare_parameter('enable_detailed_conversion', True).value

        self.get_logger().info('Action Converter Node initialized')

    def plan_actions_callback(self, request, response):
        """Callback for the PlanActions service"""
        self.get_logger().info(f'Received command for planning: "{request.command.transcript}"')

        try:
            # For this implementation, we'll create a basic plan structure
            # In a real implementation, this would integrate with the command parser
            cognitive_plan = self._create_basic_plan(request.command)

            # If detailed conversion is enabled, enhance the plan with ROS 2 specific actions
            if self.enable_detailed_conversion:
                cognitive_plan = self._enhance_plan_with_ros2_actions(cognitive_plan)

            # Set the plan in the response
            response.result = cognitive_plan

            self.get_logger().info(f'Generated enhanced plan with {len(cognitive_plan.actions)} actions')
        except Exception as e:
            self.get_logger().error(f'Error generating plan: {e}')
            # Return an empty plan in case of error
            response.result = self._create_empty_plan(request.command)

        return response

    def _enhance_plan_with_ros2_actions(self, cognitive_plan):
        """Enhance the cognitive plan with detailed ROS 2 action mappings"""
        enhanced_actions = []

        for action_step in cognitive_plan.actions:
            # Convert each action step to a more detailed ROS 2 action
            enhanced_action = self._convert_to_ros2_action(action_step)
            enhanced_actions.append(enhanced_action)

        cognitive_plan.actions = enhanced_actions
        return cognitive_plan

    def _convert_to_ros2_action(self, action_step):
        """Convert an action step to a detailed ROS 2 action with specific parameters"""
        # Create a copy of the action step
        converted_action = ActionStep()
        converted_action.id = action_step.id
        converted_action.action_type = action_step.action_type
        converted_action.priority = action_step.priority
        converted_action.dependencies = action_step.dependencies
        converted_action.timeout = action_step.timeout
        converted_action.success_criteria = action_step.success_criteria

        # Convert parameters based on action type
        if action_step.action_type == "NAVIGATION":
            converted_action.parameters = self._convert_navigation_params(action_step.parameters)
        elif action_step.action_type == "MANIPULATION":
            converted_action.parameters = self._convert_manipulation_params(action_step.parameters)
        elif action_step.action_type == "PERCEPTION":
            converted_action.parameters = self._convert_perception_params(action_step.parameters)
        else:
            # For unknown or other types, keep original parameters
            converted_action.parameters = action_step.parameters

        return converted_action

    def _convert_navigation_params(self, original_params):
        """Convert navigation parameters to detailed ROS 2 navigation parameters"""
        converted_params = []

        for param in original_params:
            if param.startswith("move_forward"):
                # Extract distance if specified
                if "distance:" in param:
                    distance_str = param.split("distance:")[1]
                    distance = self._parse_distance(distance_str)
                    # Convert to Twist message parameters for forward movement
                    linear_x = 0.5  # m/s
                    duration = distance / linear_x if distance > 0 else 1.0
                    converted_params.extend([
                        f"linear_x:{linear_x}",
                        f"duration:{duration}",
                        f"angular_z:0.0"
                    ])
                else:
                    # Default forward movement
                    converted_params.extend([
                        "linear_x:0.5",
                        "duration:2.0",  # 2 seconds at 0.5 m/s = 1 meter
                        "angular_z:0.0"
                    ])

            elif param.startswith("move_backward"):
                if "distance:" in param:
                    distance_str = param.split("distance:")[1]
                    distance = self._parse_distance(distance_str)
                    linear_x = -0.5  # negative for backward
                    duration = distance / abs(linear_x) if distance > 0 else 1.0
                    converted_params.extend([
                        f"linear_x:{linear_x}",
                        f"duration:{duration}",
                        f"angular_z:0.0"
                    ])
                else:
                    converted_params.extend([
                        "linear_x:-0.5",
                        "duration:2.0",
                        "angular_z:0.0"
                    ])

            elif param.startswith("turn"):
                # Parse turn parameters
                if "angle:" in param:
                    angle_str = param.split("angle:")[1]
                    try:
                        angle = float(angle_str)
                        # Convert angle to angular velocity and duration
                        angular_z = 0.5  # rad/s
                        duration = math.radians(angle) / angular_z if angle != 0 else 1.0
                        direction = "left" if "left" in param else "right"
                        sign = 1.0 if direction == "left" else -1.0
                        converted_params.extend([
                            f"linear_x:0.0",
                            f"duration:{duration}",
                            f"angular_z:{sign * angular_z}"
                        ])
                    except ValueError:
                        # Default turn
                        converted_params.extend([
                            "linear_x:0.0",
                            "duration:3.14",  # 180 degrees at 0.5 rad/s
                            "angular_z:0.5"
                        ])
                else:
                    converted_params.extend([
                        "linear_x:0.0",
                        "duration:3.14",
                        "angular_z:0.5"
                    ])

            elif param.startswith("navigate_to"):
                # Convert location-based navigation to specific coordinates
                if "location:" in param:
                    location = param.split("location:")[1]
                    pose = self._get_pose_for_location(location)
                    converted_params.extend([
                        f"goal_position_x:{pose.position.x}",
                        f"goal_position_y:{pose.position.y}",
                        f"goal_position_z:{pose.position.z}",
                        f"goal_orientation_x:{pose.orientation.x}",
                        f"goal_orientation_y:{pose.orientation.y}",
                        f"goal_orientation_z:{pose.orientation.z}",
                        f"goal_orientation_w:{pose.orientation.w}",
                        f"location:{location}"
                    ])
                else:
                    # Default pose if location is unknown
                    converted_params.extend([
                        "goal_position_x:1.0",
                        "goal_position_y:0.0",
                        "goal_position_z:0.0",
                        "goal_orientation_x:0.0",
                        "goal_orientation_y:0.0",
                        "goal_orientation_z:0.0",
                        "goal_orientation_w:1.0",
                        "location:unknown"
                    ])

            else:
                # Add original parameter if not specifically handled
                converted_params.append(param)

        return converted_params

    def _convert_manipulation_params(self, original_params):
        """Convert manipulation parameters to detailed ROS 2 manipulation parameters"""
        converted_params = []

        for param in original_params:
            if param.startswith("pick_object"):
                obj_name = param.split("object:")[1] if "object:" in param else "unknown"
                converted_params.extend([
                    f"object_name:{obj_name}",
                    "action_type:grasp",
                    "gripper_position:0.8",  # Open gripper
                    "approach_distance:0.1",  # 10 cm approach
                    "grasp_height:0.2"       # Grasp at 20 cm height
                ])

            elif param.startswith("place_object"):
                obj_name = param.split("object:")[1] if "object:" in param else "unknown"
                converted_params.extend([
                    f"object_name:{obj_name}",
                    "action_type:place",
                    "gripper_position:0.8",  # Open gripper to release
                    "place_height:0.1"       # Place at 10 cm height
                ])

            else:
                # Add original parameter if not specifically handled
                converted_params.append(param)

        return converted_params

    def _convert_perception_params(self, original_params):
        """Convert perception parameters to detailed ROS 2 perception parameters"""
        converted_params = []

        for param in original_params:
            if param.startswith("detect_object"):
                obj_name = param.split("object:")[1] if "object:" in param else "unknown"
                converted_params.extend([
                    f"target_object:{obj_name}",
                    "action_type:detection",
                    "detection_method:template_matching",
                    "confidence_threshold:0.7",
                    "max_detection_range:2.0"  # meters
                ])

            elif param.startswith("look_for"):
                obj_name = param.split("look_for:")[1] if "look_for:" in param else "unknown"
                converted_params.extend([
                    f"target_object:{obj_name}",
                    "action_type:search",
                    "search_method:pan_tilt",
                    "search_area:360_degree",
                    "confidence_threshold:0.7"
                ])

            elif param == "scan_environment":
                converted_params.extend([
                    "action_type:scan",
                    "scan_method:lidar_vision_fusion",
                    "scan_resolution:0.1",  # meters
                    "scan_range:3.0"        # meters
                ])

            else:
                # Add original parameter if not specifically handled
                converted_params.append(param)

        return converted_params

    def _parse_distance(self, distance_str):
        """Parse distance string and convert to meters"""
        try:
            # Handle different units
            if "m" in distance_str or "meter" in distance_str:
                # Extract number before 'm' or 'meter'
                import re
                number_match = re.search(r'[\d.]+', distance_str)
                if number_match:
                    return float(number_match.group())
            elif "cm" in distance_str or "centimeter" in distance_str:
                import re
                number_match = re.search(r'[\d.]+', distance_str)
                if number_match:
                    return float(number_match.group()) / 100.0  # Convert cm to m
            elif "ft" in distance_str or "foot" in distance_str:
                import re
                number_match = re.search(r'[\d.]+', distance_str)
                if number_match:
                    return float(number_match.group()) * 0.3048  # Convert ft to m
            else:
                # Assume it's just a number in meters
                return float(distance_str)
        except ValueError:
            # Default to 1 meter if parsing fails
            return 1.0

    def _get_pose_for_location(self, location):
        """Get a predefined pose for a known location"""
        # Predefined locations and their poses (in a real system, these would come from a map)
        location_poses = {
            "kitchen": Pose(
                position=Point(x=3.0, y=1.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ),
            "bedroom": Pose(
                position=Point(x=-2.0, y=2.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)  # 90 degree rotation
            ),
            "living room": Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ),
            "bathroom": Pose(
                position=Point(x=2.5, y=-1.5, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)  # 180 degree rotation
            )
        }

        return location_poses.get(location, Pose(
            position=Point(x=1.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        ))

    def convert_plan_to_ros2_actions(self, cognitive_plan):
        """Convert an entire cognitive plan to ROS 2 action format"""
        # This method would convert the cognitive plan to actual ROS 2 action messages
        # that can be sent to the robot's action servers
        self.get_logger().info(f'Converting plan with {len(cognitive_plan.actions)} actions to ROS 2 format')

        # In a real implementation, this would create actual ROS 2 action goals
        # and prepare them for execution by the appropriate action servers
        ros2_action_goals = []

        for i, action in enumerate(cognitive_plan.actions):
            # Create a ROS 2 action goal based on the action type
            goal = self._create_ros2_action_goal(action)
            ros2_action_goals.append(goal)

        return ros2_action_goals

    def _create_ros2_action_goal(self, action_step):
        """Create a ROS 2 action goal for a specific action step"""
        # This would create actual action goals for navigation, manipulation, etc.
        # In this simplified version, we'll return a dictionary representation
        goal = {
            'action_id': action_step.id,
            'action_type': action_step.action_type,
            'parameters': action_step.parameters,
            'timeout': action_step.timeout.sec + action_step.timeout.nanosec / 1e9
        }

        return goal

    def _create_basic_plan(self, voice_command):
        """Create a basic cognitive plan from a voice command"""
        # This is a simplified implementation
        # In a real system, this would call the command parser module
        from vla_msgs.msg import CognitivePlan
        plan = CognitivePlan()
        plan.id = f"basic_plan_{self.get_clock().now().nanoseconds}"
        plan.voice_command_id = voice_command.id
        plan.timestamp = self.get_clock().now().to_msg()
        plan.status = "READY"
        plan.actions = []  # This will be populated by the enhancement function
        plan.estimated_duration.sec = 30  # Default estimate
        plan.estimated_duration.nanosec = 0
        plan.required_capabilities = ["navigation", "perception"]  # Default capabilities

        return plan

    def _create_empty_plan(self, voice_command):
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
    node = ActionConverterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()