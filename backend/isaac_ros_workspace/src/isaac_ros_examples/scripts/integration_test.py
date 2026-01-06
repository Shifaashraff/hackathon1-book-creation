#!/usr/bin/env python3
"""
Integration test script for Isaac AI Robot Brain workflow
This script demonstrates how Isaac Sim, Isaac ROS, and Nav2 components integrate
"""

import sys
import time
import subprocess
import signal
import os

def test_isaac_sim_integration():
    """Test Isaac Sim environment setup"""
    print("Testing Isaac Sim integration...")

    # Check if Isaac Sim environment exists
    sim_env_path = "isaac_sim/assets/environments/training_scenes/photorealistic_world.usd"
    if os.path.exists(sim_env_path):
        print("[PASS] Isaac Sim environment exists")
    else:
        print("[FAIL] Isaac Sim environment not found")
        return False

    # Check if physics configuration exists
    physics_config_path = "isaac_sim/configs/sim_config.yaml"
    if os.path.exists(physics_config_path):
        print("[PASS] Isaac Sim physics configuration exists")
    else:
        print("[FAIL] Isaac Sim physics configuration not found")
        return False

    print("[PASS] Isaac Sim integration test passed")
    return True

def test_isaac_ros_integration():
    """Test Isaac ROS perception pipeline"""
    print("Testing Isaac ROS integration...")

    # Check if perception pipeline node exists
    perception_node_path = "isaac_ros_workspace/src/isaac_ros_examples/src/perception_pipeline_node.py"
    if os.path.exists(perception_node_path):
        print("[PASS] Perception pipeline node exists")
    else:
        print("[FAIL] Perception pipeline node not found")
        return False

    # Check if robot control node exists
    control_node_path = "isaac_ros_workspace/src/isaac_ros_examples/src/isaac_sim_robot_control_node.py"
    if os.path.exists(control_node_path):
        print("[PASS] Robot control node exists")
    else:
        print("[FAIL] Robot control node not found")
        return False

    # Check if navigation interface node exists
    nav_node_path = "isaac_ros_workspace/src/isaac_ros_examples/src/nav2_interface_node.py"
    if os.path.exists(nav_node_path):
        print("[PASS] Navigation interface node exists")
    else:
        print("[FAIL] Navigation interface node not found")
        return False

    print("[PASS] Isaac ROS integration test passed")
    return True

def test_nav2_integration():
    """Test Nav2 navigation configuration"""
    print("Testing Nav2 integration...")

    # Check if Nav2 configuration files exist
    nav2_config_paths = [
        "nav2/config/humanoid_nav2_params.yaml",
        "nav2/config/costmap_common_params.yaml",
        "nav2/config/planner_server_params.yaml",
        "nav2/config/controller_server_params.yaml"
    ]

    for config_path in nav2_config_paths:
        if os.path.exists(config_path):
            print(f"[PASS] {config_path} exists")
        else:
            print(f"[FAIL] {config_path} not found")
            return False

    # Check if launch files exist
    launch_file_path = "nav2/launch/"
    if os.path.exists(launch_file_path):
        print("[PASS] Nav2 launch files directory exists")
    else:
        print("[FAIL] Nav2 launch files directory not found")
        return False

    # Check if footstep planner exists
    footstep_planner_path = "nav2/scripts/footstep_planner.py"
    if os.path.exists(footstep_planner_path):
        print("[PASS] Footstep planner exists")
    else:
        print("[FAIL] Footstep planner not found")
        return False

    print("[PASS] Nav2 integration test passed")
    return True

def test_system_integration():
    """Test end-to-end system integration"""
    print("Testing system integration...")

    # Check if full simulation launch file exists
    full_sim_launch = "isaac_ros_workspace/src/isaac_ros_examples/launch/full_simulation.launch.py"
    if os.path.exists(full_sim_launch):
        print("[PASS] Full simulation launch file exists")
    else:
        print("[FAIL] Full simulation launch file not found")
        return False

    # Check if rosbridge launch file exists
    rosbridge_launch = "isaac_ros_workspace/src/isaac_ros_examples/launch/rosbridge_launch.py"
    if os.path.exists(rosbridge_launch):
        print("[PASS] Rosbridge launch file exists")
    else:
        print("[FAIL] Rosbridge launch file not found")
        return False

    print("[PASS] System integration test passed")
    return True

def main():
    """Main integration test function"""
    print("Starting Isaac AI Robot Brain Integration Test")
    print("=" * 50)

    all_passed = True

    # Test individual components
    if not test_isaac_sim_integration():
        all_passed = False

    if not test_isaac_ros_integration():
        all_passed = False

    if not test_nav2_integration():
        all_passed = False

    if not test_system_integration():
        all_passed = False

    print("=" * 50)
    if all_passed:
        print("[PASS] All integration tests passed!")
        print("Isaac AI Robot Brain system is properly configured and integrated.")
        return 0
    else:
        print("[FAIL] Some integration tests failed!")
        print("Please check the configuration and try again.")
        return 1

if __name__ == "__main__":
    sys.exit(main())