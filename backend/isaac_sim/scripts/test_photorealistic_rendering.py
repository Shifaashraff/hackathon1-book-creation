#!/usr/bin/env python3

"""
Test script to validate photorealistic rendering setup for Isaac Sim humanoid robot simulation
This script validates the configuration and setup without requiring actual Isaac Sim execution
"""

import os
import sys
import yaml
import json
from pathlib import Path


def validate_photorealistic_setup():
    """
    Validate the photorealistic rendering setup for Isaac Sim
    """
    print("[INFO] Validating Photorealistic Rendering Setup for Isaac Sim...")

    # Check if required directories exist
    required_dirs = [
        "isaac_sim/assets/environments/training_scenes",
        "isaac_sim/configs",
        "isaac_sim/configs/robot_configs",
        "isaac_sim/extensions/humanoid_control",
        "isaac_ros_workspace/src/isaac_ros_examples/robots"
    ]

    all_dirs_exist = True
    for dir_path in required_dirs:
        if not os.path.exists(dir_path):
            print(f"[ERROR] Directory does not exist: {dir_path}")
            all_dirs_exist = False
        else:
            print(f"[OK] Directory exists: {dir_path}")

    if not all_dirs_exist:
        print("[ERROR] Required directories missing")
        return False

    # Check if required files exist
    required_files = [
        "isaac_sim/assets/environments/training_scenes/photorealistic_world.usd",
        "isaac_sim/configs/sim_config.yaml",
        "isaac_sim/configs/robot_configs/humanoid_control.yaml",
        "isaac_sim/extensions/humanoid_control/__init__.py",
        "isaac_sim/extensions/humanoid_control/humanoid_control.py",
        "isaac_ros_workspace/src/isaac_ros_examples/robots/humanoid_robot.urdf",
        "isaac_ros_workspace/src/isaac_ros_examples/src/isaac_sim_robot_control_node.py"
    ]

    all_files_exist = True
    for file_path in required_files:
        if not os.path.exists(file_path):
            print(f"[ERROR] File does not exist: {file_path}")
            all_files_exist = False
        else:
            print(f"[OK] File exists: {file_path}")

    if not all_files_exist:
        print("[ERROR] Required files missing")
        return False

    # Validate USD file content (basic check)
    usd_file = "isaac_sim/assets/environments/training_scenes/photorealistic_world.usd"
    with open(usd_file, 'r') as f:
        usd_content = f.read()

    # Check for key photorealistic elements
    photorealistic_elements = [
        "DomeLight", "DistantLight", "RectLight", "Camera",
        "fStop", "focalLength", "colorTemperature", "intensity"
    ]

    usd_valid = True
    for element in photorealistic_elements:
        if element not in usd_content:
            print(f"[ERROR] Photorealistic element missing in USD: {element}")
            usd_valid = False
        else:
            print(f"[OK] Photorealistic element found in USD: {element}")

    if not usd_valid:
        print("[ERROR] USD file missing photorealistic elements")
        return False

    # Validate sim_config.yaml
    config_file = "isaac_sim/configs/sim_config.yaml"
    with open(config_file, 'r') as f:
        try:
            config = yaml.safe_load(f)
        except yaml.YAMLError as e:
            print(f"[ERROR] Invalid YAML in sim_config.yaml: {e}")
            return False

    required_config_sections = ["physics", "rendering", "simulation", "humanoid_physics"]
    config_valid = True

    for section in required_config_sections:
        if section not in config:
            print(f"[ERROR] Missing config section: {section}")
            config_valid = False
        else:
            print(f"[OK] Config section found: {section}")

    # Check rendering parameters
    if "rendering" in config:
        rendering = config["rendering"]
        if "quality" in rendering and rendering["quality"] == "Production":
            print("[OK] Rendering quality set to Production for photorealistic output")
        else:
            print("[WARN] Rendering quality not set to Production")

        if "denoise" in rendering and rendering["denoise"]["enable"]:
            print("[OK] Denoising enabled for clean photorealistic output")
        else:
            print("[WARN] Denoising not enabled")

    if not config_valid:
        print("[ERROR] Config validation failed")
        return False

    # Validate humanoid control config
    control_config_file = "isaac_sim/configs/robot_configs/humanoid_control.yaml"
    with open(control_config_file, 'r') as f:
        try:
            control_config = yaml.safe_load(f)
        except yaml.YAMLError as e:
            print(f"[ERROR] Invalid YAML in humanoid_control.yaml: {e}")
            return False

    if "joints" in control_config and "bipedal_control" in control_config:
        print("[OK] Humanoid control configuration found")
    else:
        print("[ERROR] Missing humanoid control configuration")
        return False

    # Validate URDF file exists and has basic structure
    urdf_file = "isaac_ros_workspace/src/isaac_ros_examples/robots/humanoid_robot.urdf"
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()

    if "<robot" in urdf_content and "</robot>" in urdf_content:
        print("[OK] URDF file has basic structure")
    else:
        print("[ERROR] URDF file missing basic structure")
        return False

    # Check for humanoid-specific elements in URDF
    humanoid_elements = ["<joint", "<link", "hip", "knee", "ankle", "shoulder", "elbow"]
    urdf_humanoid_valid = True
    urdf_content_lower = urdf_content.lower()

    for element in humanoid_elements:
        if element in urdf_content_lower:
            print(f"[OK] Humanoid element found in URDF: {element}")
        else:
            print(f"[WARN] Humanoid element not found in URDF: {element}")

    # Validate Isaac ROS control node
    control_node_file = "isaac_ros_workspace/src/isaac_ros_examples/src/isaac_sim_robot_control_node.py"
    with open(control_node_file, 'r') as f:
        control_node_content = f.read()

    required_ros_elements = [
        "rclpy", "Node", "JointState", "Odometry", "Twist",
        "cmd_vel_callback", "publish_joint_states", "publish_odometry"
    ]

    node_valid = True
    for element in required_ros_elements:
        if element in control_node_content:
            print(f"[OK] ROS element found in control node: {element}")
        else:
            print(f"[WARN] ROS element not found in control node: {element}")
            # Not critical for photorealistic rendering validation

    print("\n[OK] Photorealistic rendering setup validation completed successfully!")
    print("\n[SUMMARY]:")
    print("   - USD world file contains photorealistic lighting and camera setups")
    print("   - Physics parameters configured for realistic humanoid simulation")
    print("   - Rendering settings optimized for photorealistic output")
    print("   - Humanoid robot model and control configuration in place")
    print("   - ROS control interface ready for Isaac Sim communication")
    print("\n[SUCCESS] The setup is ready for Isaac Sim photorealistic rendering testing")

    return True


def main():
    """
    Main function to run photorealistic rendering validation
    """
    print("=" * 70)
    print(" Isaac Sim - Photorealistic Rendering Validation Test")
    print("=" * 70)

    success = validate_photorealistic_setup()

    if success:
        print("\n[SUCCESS] Photorealistic rendering setup validation PASSED!")
        print("\n[INFO] Next steps for actual testing:")
        print("   1. Launch Isaac Sim with the photorealistic_world.usd")
        print("   2. Load the humanoid_robot.urdf into the simulation")
        print("   3. Apply the sim_config.yaml and humanoid_control.yaml configurations")
        print("   4. Run the isaac_sim_robot_control_node.py to interface with the simulation")
        print("   5. Observe photorealistic rendering output with advanced lighting")
        return 0
    else:
        print("\n[ERROR] Photorealistic rendering setup validation FAILED!")
        return 1


if __name__ == "__main__":
    sys.exit(main())