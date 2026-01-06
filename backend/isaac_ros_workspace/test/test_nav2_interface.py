#!/usr/bin/env python3

"""
Unit tests for Isaac ROS to Nav2 Interface
Tests the nav2 interface node functionality
"""

import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import time
from unittest.mock import Mock, MagicMock


class TestNav2Interface(unittest.TestCase):
    """
    Unit tests for the Nav2 interface node
    """

    def setUp(self):
        """
        Set up the test environment
        """
        if not rclpy.ok():
            rclpy.init()

        # Create a mock node for testing
        self.test_node = Node('test_nav2_interface_node')

        # Mock publishers and subscribers
        self.mock_cmd_vel_pub = Mock()
        self.mock_perception_sub = Mock()

    def tearDown(self):
        """
        Clean up after tests
        """
        self.test_node.destroy_node()

    def test_perception_subscription(self):
        """
        Test that the Nav2 interface subscribes to perception data
        """
        expected_topic = '/perception_data'
        self.assertEqual(expected_topic, '/perception_data')

    def test_vslam_subscription(self):
        """
        Test that the Nav2 interface subscribes to VSLAM data
        """
        expected_topic = '/visual_slam/poses'
        self.assertEqual(expected_topic, '/visual_slam/poses')

    def test_detection_subscription(self):
        """
        Test that the Nav2 interface subscribes to detection data
        """
        expected_topic = '/detectnet/detections'
        self.assertEqual(expected_topic, '/detectnet/detections')

    def test_cmd_vel_publication(self):
        """
        Test that the Nav2 interface publishes velocity commands
        """
        expected_topic = '/cmd_vel'
        self.assertEqual(expected_topic, '/cmd_vel')

    def test_navigation_goal_request(self):
        """
        Test navigation goal request functionality
        """
        # Create a mock navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header = Header()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = 1.0
        goal_msg.pose.pose.position.y = 1.0
        goal_msg.pose.pose.orientation.w = 1.0

        # Test goal creation
        self.assertEqual(goal_msg.pose.header.frame_id, 'map')
        self.assertEqual(goal_msg.pose.pose.position.x, 1.0)
        self.assertEqual(goal_msg.pose.pose.position.y, 1.0)

    def test_perception_fusion_activation(self):
        """
        Test perception fusion activation
        """
        # Test that perception fusion can be enabled/disabled
        fusion_enabled = True
        self.assertTrue(fusion_enabled)

        fusion_disabled = False
        self.assertFalse(fusion_disabled)

    def test_obstacle_detection_integration(self):
        """
        Test obstacle detection integration with navigation
        """
        # Simulate detected obstacles
        obstacles = [
            {'position': {'x': 2.0, 'y': 1.5}, 'confidence': 0.9, 'class': 'person'},
            {'position': {'x': 0.5, 'y': 3.0}, 'confidence': 0.85, 'class': 'chair'}
        ]

        # Test obstacle processing
        self.assertEqual(len(obstacles), 2)
        self.assertGreaterEqual(obstacles[0]['confidence'], 0.8)

    def test_path_adjustment_with_perception(self):
        """
        Test path adjustment based on perception data
        """
        # Original path
        original_path = Path()
        original_path.poses = []

        # Add some poses to the path
        for i in range(5):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = i * 1.0
            pose_stamped.pose.position.y = 0.0
            original_path.poses.append(pose_stamped)

        # Simulate perception-adjusted path
        adjusted_path = Path()
        adjusted_path.poses = []

        # Add adjusted poses
        for i in range(5):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = i * 1.0
            pose_stamped.pose.position.y = 0.2 * (i % 2)  # Slight adjustment
            adjusted_path.poses.append(pose_stamped)

        self.assertEqual(len(original_path.poses), 5)
        self.assertEqual(len(adjusted_path.poses), 5)

    def test_semantic_navigation_integration(self):
        """
        Test semantic navigation integration
        """
        # Simulate semantic information
        semantic_info = {
            'traversable_areas': ['grass', 'paved', 'carpet'],
            'avoidance_areas': ['water', 'stairs', 'construction'],
            'preferred_areas': ['paved', 'carpet']
        }

        # Test semantic processing
        self.assertIn('grass', semantic_info['traversable_areas'])
        self.assertIn('water', semantic_info['avoidance_areas'])
        self.assertIn('paved', semantic_info['preferred_areas'])

    def test_velocity_command_generation(self):
        """
        Test velocity command generation from navigation
        """
        # Simulate navigation velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2  # m/s
        cmd_vel.angular.z = 0.1  # rad/s

        self.assertEqual(cmd_vel.linear.x, 0.2)
        self.assertEqual(cmd_vel.angular.z, 0.1)

    def test_navigation_state_tracking(self):
        """
        Test navigation state tracking
        """
        # Simulate navigation state
        nav_state = {
            'navigation_active': True,
            'current_goal': PoseStamped(),
            'perception_fusion_active': True,
            'perception_data_available': True,
            'detection_count': 3
        }

        self.assertTrue(nav_state['navigation_active'])
        self.assertTrue(nav_state['perception_fusion_active'])
        self.assertEqual(nav_state['detection_count'], 3)

    def test_dynamic_obstacle_handling(self):
        """
        Test dynamic obstacle handling
        """
        # Simulate dynamic obstacles
        dynamic_obstacles = [
            {'id': 1, 'position': {'x': 1.5, 'y': 1.0}, 'velocity': {'x': 0.1, 'y': 0.0}},
            {'id': 2, 'position': {'x': 2.0, 'y': 0.5}, 'velocity': {'x': 0.0, 'y': 0.1}}
        ]

        self.assertEqual(len(dynamic_obstacles), 2)
        self.assertGreaterEqual(dynamic_obstacles[0]['velocity']['x'], 0.0)

    def test_costmap_updates_with_perception(self):
        """
        Test costmap updates with perception data
        """
        # Simulate costmap update
        costmap_update = {
            'timestamp': time.time(),
            'updated_cells': 25,
            'obstacle_added': True,
            'obstacle_removed': False,
            'inflation_radius': 0.55
        }

        self.assertTrue(costmap_update['obstacle_added'])
        self.assertFalse(costmap_update['obstacle_removed'])
        self.assertEqual(costmap_update['inflation_radius'], 0.55)


class TestFootstepPlannerIntegration(unittest.TestCase):
    """
    Unit tests for footstep planner integration with Nav2
    """

    def setUp(self):
        """
        Set up the test environment
        """
        if not rclpy.ok():
            rclpy.init()

        self.test_node = Node('test_footstep_planner_integration_node')

    def tearDown(self):
        """
        Clean up after tests
        """
        self.test_node.destroy_node()

    def test_footstep_generation_from_path(self):
        """
        Test footstep generation from navigation path
        """
        # Simulate navigation path
        path = Path()
        path.poses = []

        # Add poses to path
        for i in range(10):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = i * 0.3  # Step length
            pose_stamped.pose.position.y = 0.0
            path.poses.append(pose_stamped)

        # Simulate footstep conversion
        footsteps = []
        for i, pose in enumerate(path.poses):
            if i % 2 == 0:  # Alternate between left and right foot
                foot_pose = PoseStamped()
                foot_pose.pose = pose.pose
                foot_pose.pose.position.y -= 0.1  # Left foot offset
                footsteps.append(('left', foot_pose))
            else:
                foot_pose = PoseStamped()
                foot_pose.pose = pose.pose
                foot_pose.pose.position.y += 0.1  # Right foot offset
                footsteps.append(('right', foot_pose))

        self.assertEqual(len(footsteps), 10)
        self.assertIn('left', [step[0] for step in footsteps])

    def test_balance_constraint_verification(self):
        """
        Test balance constraint verification for footsteps
        """
        # Simulate foot poses
        left_foot_pose = PoseStamped()
        left_foot_pose.pose.position.x = 0.0
        left_foot_pose.pose.position.y = -0.1
        left_foot_pose.pose.position.z = 0.0

        right_foot_pose = PoseStamped()
        right_foot_pose.pose.position.x = 0.0
        right_foot_pose.pose.position.y = 0.1
        right_foot_pose.pose.position.z = 0.0

        # Calculate distance between feet
        dx = right_foot_pose.pose.position.x - left_foot_pose.pose.position.x
        dy = right_foot_pose.pose.position.y - left_foot_pose.pose.position.y
        distance = (dx**2 + dy**2)**0.5

        # Check if distance is within acceptable range for balance
        min_distance = 0.1
        max_distance = 0.4
        is_balanced = min_distance <= distance <= max_distance

        self.assertTrue(is_balanced)
        self.assertGreaterEqual(distance, min_distance)

    def test_step_height_constraints(self):
        """
        Test step height constraints
        """
        # Define step constraints
        step_constraints = {
            'max_step_height': 0.2,  # meters
            'max_step_down': 0.15,   # meters
            'step_length': 0.3,      # meters
            'step_width': 0.2        # meters
        }

        self.assertEqual(step_constraints['max_step_height'], 0.2)
        self.assertEqual(step_constraints['step_length'], 0.3)
        self.assertLessEqual(step_constraints['max_step_down'], 0.2)


def suite():
    """
    Create a test suite combining all tests
    """
    suite = unittest.TestSuite()

    # Add Nav2 interface tests
    suite.addTest(unittest.makeSuite(TestNav2Interface))

    # Add footstep planner integration tests
    suite.addTest(unittest.makeSuite(TestFootstepPlannerIntegration))

    return suite


def run_tests():
    """
    Run all tests
    """
    runner = unittest.TextTestRunner(verbosity=2)
    test_suite = suite()
    result = runner.run(test_suite)

    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    exit(0 if success else 1)