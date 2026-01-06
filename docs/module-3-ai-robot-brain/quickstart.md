# Quickstart Guide: Isaac AI Robot Brain (NVIDIA Isaac)

This quickstart guide provides a step-by-step introduction to setting up and running the Isaac AI Robot Brain system with photorealistic humanoid simulation, perception pipelines, and navigation.

## Prerequisites

Before starting, ensure you have:
- NVIDIA Isaac Sim installed with GPU support
- ROS 2 Humble Hawksbill or newer
- Isaac ROS packages installed
- Nav2 packages installed
- Compatible NVIDIA GPU (RTX series recommended)
- Python 3.8 or newer

## Installation

### 1. Clone the Repository

```bash
# Clone the main repository
git clone <repository-url>
cd hackathon1
```

### 2. Set up Isaac Sim Environment

```bash
# Navigate to Isaac Sim directory
cd isaac_sim

# Create virtual environment
python -m venv isaac_sim_env
source isaac_sim_env/bin/activate  # On Windows: isaac_sim_env\Scripts\activate

# Install Isaac Sim Python dependencies
pip install -r requirements.txt
```

### 3. Set up Isaac ROS Workspace

```bash
# Navigate to Isaac ROS workspace
cd ../isaac_ros_workspace

# Source ROS 2 environment
source /opt/ros/humble/setup.bash  # Adjust path as needed

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 4. Set up Nav2 Configuration

```bash
# Nav2 configurations are already prepared in the nav2 directory
cd ../nav2/config
# All necessary configuration files are in place
```

## Launching the System

### 1. Start Isaac Sim

```bash
# Launch Isaac Sim with the humanoid robot environment
cd ../isaac_sim
python -m omni.isaac.sim.python_app --config-path configs/sim_config.yaml
```

### 2. Launch Isaac ROS Perception Pipeline

```bash
# In a new terminal, source the workspace
cd ../isaac_ros_workspace
source install/setup.bash

# Launch the perception pipeline
ros2 launch isaac_ros_examples perception_pipeline.launch.py
```

### 3. Launch Nav2 Navigation Stack

```bash
# In another terminal, source the workspace
cd ../isaac_ros_workspace
source install/setup.bash

# Launch the humanoid navigation stack
ros2 launch nav2 humanoid_navigation.launch.py
```

## Basic Operations

### 1. Test Photorealistic Rendering

1. Open Isaac Sim GUI
2. Load the `photorealistic_world.usd` from `isaac_sim/assets/environments/training_scenes/`
3. Add the humanoid robot from `isaac_ros_workspace/src/isaac_ros_examples/robots/humanoid_robot.urdf`
4. Enable photorealistic rendering in the viewport settings

### 2. Test Synthetic Data Generation

```bash
# Verify synthetic data generation is working
ros2 topic echo /front_stereo_camera/left/image_rect_color
ros2 topic echo /front_stereo_camera/right/image_rect
ros2 topic echo /imu/data
```

### 3. Test Perception Pipeline

```bash
# Check perception pipeline outputs
ros2 topic echo /perception_data
ros2 topic echo /visual_slam/poses
ros2 topic echo /detectnet/detections
```

### 4. Test Navigation

```bash
# Send a simple navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}, header: {frame_id: 'map'}}}"
```

## Key Components Overview

### Isaac Sim Environment
- Located in `isaac_sim/` directory
- Contains photorealistic world in `assets/environments/training_scenes/photorealistic_world.usd`
- Physics configuration in `configs/sim_config.yaml`
- Robot assets in `assets/robots/humanoid_robot/`

### Isaac ROS Perception
- Perception pipeline node in `isaac_ros_workspace/src/isaac_ros_examples/src/perception_pipeline_node.py`
- Message definitions in `isaac_ros_workspace/src/isaac_ros_messages/`
- Launch files in `isaac_ros_workspace/src/isaac_ros_examples/launch/`

### Nav2 Navigation
- Configuration files in `nav2/config/`
- Launch files in `nav2/launch/`
- Footstep planner in `nav2/scripts/footstep_planner.py`

## Troubleshooting Common Issues

### Isaac Sim Won't Launch
- Check GPU compatibility and drivers
- Verify Isaac Sim installation
- Ensure proper environment variables are set

### Perception Pipeline Not Working
- Verify camera topics are publishing
- Check sensor configurations in URDF
- Ensure Isaac ROS packages are built and sourced

### Navigation Not Responding
- Verify costmap is updating
- Check TF tree for proper transforms
- Ensure proper frame IDs in configurations

## Next Steps

1. Complete the [Isaac Sim exercises](./isaac-sim/exercises.md)
2. Proceed to [Isaac ROS tutorials](./isaac-ros/)
3. Practice [Nav2 navigation](./nav2/)
4. Experiment with the full AI robot brain integration

## Video Tutorials

For visual learners, check out the video tutorials in `docs/module-3-ai-robot-brain/assets/`:
- Quick Setup Tutorial
- Basic Navigation Demo
- Perception Pipeline Walkthrough

## Support

For additional help:
- Check the [troubleshooting guide](./troubleshooting.md)
- Review the [FAQ](./faq.md)
- Join our community forums