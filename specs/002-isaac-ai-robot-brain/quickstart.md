# Quickstart Guide: Isaac AI Robot Brain (NVIDIA Isaac)

**Feature**: Isaac AI Robot Brain (NVIDIA Isaac)
**Date**: 2026-01-03
**Status**: Draft

## Overview

This guide provides a quick start for setting up the Isaac AI Robot Brain curriculum with NVIDIA Isaac ecosystem. Students will learn to create photorealistic humanoid simulations in Isaac Sim, implement VSLAM and perception pipelines with Isaac ROS, and configure Nav2 for bipedal humanoid navigation.

## Prerequisites

- Ubuntu 22.04 LTS
- NVIDIA GPU with RTX capabilities (RTX 3080 or better recommended)
- CUDA 12.x installed
- ROS 2 Humble Hawksbill installed
- Isaac Sim and Isaac ROS installed
- Isaac Sim license or evaluation access
- Basic knowledge of ROS 2 and simulation (Modules 1 & 2)

## Installation Steps

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set up Isaac Sim Environment

```bash
# Navigate to Isaac Sim directory
cd isaac_sim

# Follow NVIDIA Isaac Sim installation instructions
# This typically involves:
# - Downloading Isaac Sim from NVIDIA developer portal
# - Extracting and configuring the simulation environment
# - Setting up necessary environment variables
```

### 3. Install Isaac ROS Dependencies

```bash
# Navigate to Isaac ROS workspace
cd isaac_ros_workspace

# Install Isaac ROS packages
sudo apt update
sudo apt install ros-humble-isaac-ros-* ros-humble-nav2-*

# Build the workspace
colcon build --symlink-install
source install/setup.bash
```

### 4. Install Docusaurus Dependencies

```bash
# Navigate to the docusaurus directory
cd docusaurus
npm install
```

### 5. Set up Nav2 Configuration

```bash
# Navigate to Nav2 configuration directory
cd nav2

# Verify Nav2 installation
ros2 launch nav2_bringup tb3_simulation_launch.py
```

## Curriculum Navigation

The curriculum is organized into three progressive chapters:

### Chapter 1: Isaac Sim - Photorealistic Simulation
- Navigate to `docusaurus/docs/module-3-ai-robot-brain/isaac-sim/`
- Start with `setup.md` and follow the sequence
- Complete hands-on exercises with Isaac Sim humanoid robot models

### Chapter 2: Isaac ROS - VSLAM and Perception
- Navigate to `docusaurus/docs/module-3-ai-robot-brain/isaac-ros/`
- Follow the sequence from `vslam-setup.md` onwards
- Integrate perception pipelines with the simulation

### Chapter 3: Nav2 - Bipedal Path Planning
- Navigate to `docusaurus/docs/module-3-ai-robot-brain/nav2/`
- Follow the sequence from `path-planning.md` onwards
- Configure navigation for humanoid-specific constraints

## Running the Simulation Environment

### Isaac Sim Simulation

```bash
# From the isaac_sim directory
cd isaac_sim

# Launch Isaac Sim with humanoid robot
./python.sh -m omni.isaac.sim.python_app --config=humanoid_robot_config.yaml
```

### Isaac ROS Perception Pipeline

```bash
# Source the Isaac ROS workspace
cd isaac_ros_workspace
source install/setup.bash

# Launch perception pipeline
ros2 launch isaac_ros_examples perception_pipeline_launch.py
```

### Nav2 Navigation

```bash
# Source the workspace
cd isaac_ros_workspace
source install/setup.bash

# Launch Nav2 with humanoid-specific configuration
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=./config/humanoid_nav2_params.yaml
```

## Running the Documentation Site

```bash
cd docusaurus
npm start
```

This will start the Docusaurus documentation site locally at `http://localhost:3000`.

## Key ROS 2 Topics for Monitoring

### Isaac ROS Topics
- `/isaac_ros/vslam/pose` - VSLAM estimated pose
- `/isaac_ros/perception/detections` - Object detection results
- `/isaac_ros/perception/segmentation` - Semantic segmentation output

### Navigation Topics
- `/humanoid/cmd_vel` - Velocity commands for humanoid robot
- `/humanoid/odom` - Odometry information
- `/humanoid/goal_pose` - Navigation goal commands

## Troubleshooting

### Common Issues

1. **Isaac Sim won't launch**: Verify NVIDIA GPU drivers and Isaac Sim installation
2. **Perception pipeline not working**: Check Isaac ROS package installation and dependencies
3. **Navigation fails**: Verify Nav2 configuration for humanoid-specific constraints
4. **Documentation site not loading**: Run `npm install` again in docusaurus directory

### Useful Commands

```bash
# Check Isaac ROS packages
ros2 pkg list | grep isaac

# Check running Isaac ROS nodes
ros2 node list | grep isaac

# Echo Isaac ROS topics
ros2 topic echo /isaac_ros/perception/detections

# Check Isaac Sim logs
# Check ~/.nvidia-isaac/logs/ directory
```

## Next Steps

After completing the quickstart:
1. Work through Chapter 1 exercises to understand Isaac Sim
2. Proceed to perception pipeline implementation in Chapter 2
3. Complete navigation configuration in Chapter 3
4. Experiment with custom humanoid models and environments
5. Generate your own synthetic datasets for AI training