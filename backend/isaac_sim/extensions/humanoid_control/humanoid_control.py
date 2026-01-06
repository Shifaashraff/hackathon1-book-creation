# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Humanoid Robot Joint Control Extension for Isaac Sim
Provides specialized control algorithms for bipedal humanoid robots
"""

import omni
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.core import World
from omni.isaac.core.utils import rotations
from omni.isaac.core.utils import stage
import numpy as np
import carb
import math


class HumanoidJointController:
    """
    Specialized controller for humanoid robot joint control with bipedal locomotion support
    """

    def __init__(self, robot_name="/World/humanoid_robot", config_path=None):
        """
        Initialize the humanoid joint controller

        Args:
            robot_name (str): Path to the robot in the USD stage
            config_path (str): Path to configuration file with joint parameters
        """
        self._robot_name = robot_name
        self._config_path = config_path
        self._articulation = None
        self._joint_names = []
        self._joint_indices = {}
        self._control_mode = "position"  # position, velocity, or effort
        self._default_dt = 1.0/60.0  # Default time step

        # Bipedal locomotion parameters
        self._step_height = 0.1
        self._step_length = 0.3
        self._step_duration = 0.8
        self._stance_duration = 0.4
        self._swing_duration = 0.4

        # Balance control parameters
        self._com_height = 0.8
        self._com_tolerance = 0.05
        self._zmp_margin = 0.05

        # Initialize joint control parameters
        self._init_joint_parameters()

    def _init_joint_parameters(self):
        """
        Initialize joint parameters based on humanoid robot structure
        """
        # Define humanoid joint names and their indices
        self._joint_names = [
            "left_hip", "left_knee", "left_ankle",
            "right_hip", "right_knee", "right_ankle",
            "left_shoulder", "left_elbow",
            "right_shoulder", "right_elbow"
        ]

        for i, name in enumerate(self._joint_names):
            self._joint_indices[name] = i

        # Initialize joint limits and control parameters
        self._joint_limits = {
            "left_hip": (-1.57, 1.57),
            "left_knee": (-1.57, 0.0),
            "left_ankle": (-0.5, 0.5),
            "right_hip": (-1.57, 1.57),
            "right_knee": (-1.57, 0.0),
            "right_ankle": (-0.5, 0.5),
            "left_shoulder": (-1.57, 1.57),
            "left_elbow": (-1.57, 1.57),
            "right_shoulder": (-1.57, 1.57),
            "right_elbow": (-1.57, 1.57)
        }

        # Initialize default joint positions (standing pose)
        self._default_positions = np.array([
            0.0,  # left_hip
            0.0,  # left_knee
            0.0,  # left_ankle
            0.0,  # right_hip
            0.0,  # right_knee
            0.0,  # right_ankle
            0.0,  # left_shoulder
            0.0,  # left_elbow
            0.0,  # right_shoulder
            0.0   # right_elbow
        ])

    def attach_to_robot(self, articulation: Articulation):
        """
        Attach the controller to a robot articulation

        Args:
            articulation (Articulation): The robot articulation to control
        """
        self._articulation = articulation

        # Set default joint positions
        self.set_joint_positions(self._default_positions)

    def set_joint_positions(self, positions, joint_indices=None):
        """
        Set joint positions for the humanoid robot

        Args:
            positions (np.array or list): Target joint positions
            joint_indices (list): Indices of joints to control (None for all)
        """
        if self._articulation is None:
            carb.log_error("No articulation attached to controller")
            return

        positions = np.array(positions)

        if joint_indices is None:
            # Control all joints
            self._articulation.set_joint_positions(positions)
        else:
            # Control specific joints
            current_positions = self._articulation.get_joint_positions()
            for i, idx in enumerate(joint_indices):
                if idx < len(current_positions):
                    current_positions[idx] = positions[i]
            self._articulation.set_joint_positions(current_positions)

    def set_joint_velocities(self, velocities, joint_indices=None):
        """
        Set joint velocities for the humanoid robot

        Args:
            velocities (np.array or list): Target joint velocities
            joint_indices (list): Indices of joints to control (None for all)
        """
        if self._articulation is None:
            carb.log_error("No articulation attached to controller")
            return

        velocities = np.array(velocities)

        if joint_indices is None:
            # Control all joints
            self._articulation.set_joint_velocities(velocities)
        else:
            # Control specific joints
            current_velocities = self._articulation.get_joint_velocities()
            for i, idx in enumerate(joint_indices):
                if idx < len(current_velocities):
                    current_velocities[idx] = velocities[i]
            self._articulation.set_joint_velocities(current_velocities)

    def set_joint_efforts(self, efforts, joint_indices=None):
        """
        Set joint efforts (torques) for the humanoid robot

        Args:
            efforts (np.array or list): Target joint efforts
            joint_indices (list): Indices of joints to control (None for all)
        """
        if self._articulation is None:
            carb.log_error("No articulation attached to controller")
            return

        efforts = np.array(efforts)

        if joint_indices is None:
            # Control all joints
            self._articulation.set_joint_efforts(efforts)
        else:
            # Control specific joints
            current_efforts = self._articulation.get_joint_efforts()
            for i, idx in enumerate(joint_indices):
                if idx < len(current_efforts):
                    current_efforts[idx] = efforts[i]
            self._articulation.set_joint_efforts(current_efforts)

    def get_joint_positions(self):
        """
        Get current joint positions

        Returns:
            np.array: Current joint positions
        """
        if self._articulation is None:
            return np.zeros(len(self._joint_names))
        return self._articulation.get_joint_positions()

    def get_joint_velocities(self):
        """
        Get current joint velocities

        Returns:
            np.array: Current joint velocities
        """
        if self._articulation is None:
            return np.zeros(len(self._joint_names))
        return self._articulation.get_joint_velocities()

    def get_joint_efforts(self):
        """
        Get current joint efforts

        Returns:
            np.array: Current joint efforts
        """
        if self._articulation is None:
            return np.zeros(len(self._joint_names))
        return self._articulation.get_joint_efforts()

    def move_to_standing_pose(self):
        """
        Move the humanoid robot to a stable standing pose
        """
        self.set_joint_positions(self._default_positions)

    def execute_walk_step(self, step_direction="forward", step_size=0.3):
        """
        Execute a single walking step for bipedal locomotion

        Args:
            step_direction (str): Direction of step ("forward", "backward", "left", "right")
            step_size (float): Size of the step in meters
        """
        if self._articulation is None:
            carb.log_error("No articulation attached to controller")
            return

        # Get current joint positions
        current_positions = self.get_joint_positions()

        # Calculate target positions for walking gait
        target_positions = current_positions.copy()

        # Simplified walking gait - in a real implementation, this would be more complex
        if step_direction == "forward":
            # Lift one foot, move forward, place down
            # This is a simplified version - real walking would require complex gait planning
            target_positions[self._joint_indices["left_hip"]] = 0.1
            target_positions[self._joint_indices["left_knee"]] = -0.2
            target_positions[self._joint_indices["right_hip"]] = -0.05
            target_positions[self._joint_indices["right_knee"]] = 0.1

        elif step_direction == "left":
            # Lateral movement
            target_positions[self._joint_indices["left_hip"]] = 0.1
            target_positions[self._joint_indices["right_hip"]] = -0.1

        # Apply the target positions
        self.set_joint_positions(target_positions)

    def balance_control(self):
        """
        Apply balance control to maintain humanoid stability
        """
        if self._articulation is None:
            carb.log_error("No articulation attached to controller")
            return

        # Get current robot state
        current_positions = self.get_joint_positions()

        # Apply simple balance control adjustments
        # In a real implementation, this would use sensor feedback and complex algorithms
        balance_adjustments = np.zeros(len(current_positions))

        # Apply balance adjustments to ankle joints for stability
        balance_adjustments[self._joint_indices["left_ankle"]] = 0.02
        balance_adjustments[self._joint_indices["right_ankle"]] = -0.02

        # Calculate new positions with balance adjustments
        new_positions = current_positions + balance_adjustments

        # Apply position limits
        for i, joint_name in enumerate(self._joint_names):
            min_limit, max_limit = self._joint_limits[joint_name]
            new_positions[i] = np.clip(new_positions[i], min_limit, max_limit)

        # Set the adjusted positions
        self.set_joint_positions(new_positions)

    def get_robot_state(self):
        """
        Get comprehensive robot state including joint positions, velocities, and efforts

        Returns:
            dict: Dictionary containing robot state information
        """
        if self._articulation is None:
            return {}

        return {
            "joint_positions": self.get_joint_positions(),
            "joint_velocities": self.get_joint_velocities(),
            "joint_efforts": self.get_joint_efforts(),
            "end_effectors": self._get_end_effector_positions(),
            "center_of_mass": self._get_center_of_mass()
        }

    def _get_end_effector_positions(self):
        """
        Get positions of end effectors (hands and feet)

        Returns:
            dict: Positions of end effectors
        """
        # This would require more complex implementation to get actual end effector positions
        # For now, return placeholder values
        return {
            "left_hand": [0.0, 0.0, 0.0],
            "right_hand": [0.0, 0.0, 0.0],
            "left_foot": [0.0, 0.0, 0.0],
            "right_foot": [0.0, 0.0, 0.0]
        }

    def _get_center_of_mass(self):
        """
        Get estimated center of mass position

        Returns:
            list: [x, y, z] position of center of mass
        """
        # Simplified COM calculation - in reality this would be more complex
        return [0.0, 0.0, self._com_height]


class HumanoidControlExtension:
    """
    Main extension class for the humanoid control system
    """

    def __init__(self):
        self._humanoid_controller = None
        self._world = None
        self._task = None
        carb.log_info("Humanoid Control Extension initialized")

    def setup_humanoid_robot(self, robot_path="/World/humanoid_robot", usd_path=None):
        """
        Set up a humanoid robot in the simulation environment

        Args:
            robot_path (str): Path where the robot will be placed in the stage
            usd_path (str): Path to the robot USD file
        """
        # If no USD path provided, use a default humanoid robot
        if usd_path is None:
            # In a real implementation, this would load a specific humanoid robot USD
            carb.log_info(f"Setting up humanoid robot at {robot_path}")
        else:
            add_reference_to_stage(usd_path, robot_path)

        # Create the humanoid controller
        self._humanoid_controller = HumanoidJointController(robot_path)

        # Get the articulation from the world
        if self._world is not None:
            articulation = self._world.scene.get_object(robot_path)
            if articulation is not None:
                self._humanoid_controller.attach_to_robot(articulation)
                carb.log_info(f"Attached controller to robot at {robot_path}")
            else:
                carb.log_error(f"Could not find articulation at {robot_path}")

    def set_world(self, world: World):
        """
        Set the Isaac Sim world for the extension

        Args:
            world (World): Isaac Sim world object
        """
        self._world = world

    def get_controller(self):
        """
        Get the humanoid controller instance

        Returns:
            HumanoidJointController: The humanoid controller
        """
        return self._humanoid_controller

    def cleanup(self):
        """
        Clean up the extension resources
        """
        if self._humanoid_controller:
            self._humanoid_controller = None
        carb.log_info("Humanoid Control Extension cleaned up")


# Global instance of the extension
_humanoid_control_extension = None


def get_humanoid_control_extension():
    """
    Get the singleton instance of the humanoid control extension

    Returns:
        HumanoidControlExtension: The humanoid control extension instance
    """
    global _humanoid_control_extension
    if _humanoid_control_extension is None:
        _humanoid_control_extension = HumanoidControlExtension()
    return _humanoid_control_extension


def setup_humanoid_robot(robot_path="/World/humanoid_robot", usd_path=None):
    """
    Convenience function to set up a humanoid robot

    Args:
        robot_path (str): Path where the robot will be placed in the stage
        usd_path (str): Path to the robot USD file
    """
    extension = get_humanoid_control_extension()
    extension.setup_humanoid_robot(robot_path, usd_path)


def get_humanoid_controller():
    """
    Get the humanoid controller instance

    Returns:
        HumanoidJointController: The humanoid controller
    """
    extension = get_humanoid_control_extension()
    return extension.get_controller()