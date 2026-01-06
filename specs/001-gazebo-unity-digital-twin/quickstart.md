# Quickstart Guide: Digital Twin Curriculum - Gazebo & Unity Integration

**Feature**: Digital Twin Curriculum - Gazebo & Unity Integration
**Date**: 2026-01-02
**Status**: Draft

## Overview

This guide provides a quick start for setting up and running the Digital Twin simulation environment with Gazebo physics, sensor integration, and Unity visualization. This setup enables students to learn humanoid robot simulation with ROS 2 integration.

## Prerequisites

- Ubuntu 22.04 LTS (or Windows 10+ for Unity development)
- ROS 2 Humble Hawksbill installed
- Gazebo Garden installed
- Unity 2022.3 LTS installed
- Node.js 18+ and npm installed
- Git installed

## Installation Steps

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set up ROS 2 Workspace

```bash
# Navigate to the ROS 2 workspace
cd ros2_ws

# Build the workspace
colcon build --packages-select digital_twin_msgs gazebo_ros_integration unity_ros_bridge digital_twin_examples

# Source the workspace
source install/setup.bash
```

### 3. Install Docusaurus Dependencies

```bash
# Navigate to the docusaurus directory
cd docusaurus
npm install
```

### 4. Start the Simulation Environment

#### Physics Simulation (Gazebo)

```bash
# From the ros2_ws directory
cd ros2_ws
source install/setup.bash

# Launch the simulation
ros2 launch digital_twin_examples simulation.launch.py
```

This will start:
- Gazebo with the humanoid robot model
- Physics simulation with gravity and collisions
- ROS 2 nodes for robot control

#### Sensor Integration

With the simulation running, in a new terminal:

```bash
# Source the workspace
cd ros2_ws
source install/setup.bash

# Launch sensor nodes
ros2 run digital_twin_examples sensor_publisher_node
```

This will start publishing sensor data (LiDAR, camera, IMU) to ROS 2 topics.

### 5. Set up Unity Visualization

1. Open Unity Hub
2. Open the project from the `unity/` directory in the repository
3. In the Unity project, configure the ROS connection settings:
   - Set ROS bridge IP address (usually localhost:9090)
   - Ensure rosbridge_suite is running

### 6. Start ROS Bridge for Unity Integration

```bash
# Source ROS 2 workspace
cd ros2_ws
source install/setup.bash

# Start rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 7. Run the Complete Digital Twin

1. Start Gazebo simulation (Step 4)
2. Start sensor integration (Step 4)
3. Start ROS bridge (Step 6)
4. In Unity, run the synchronization scene to visualize the robot

## Curriculum Navigation

The curriculum is organized into three progressive chapters:

### Chapter 1: Physics Simulation in Gazebo
- Navigate to `docusaurus/docs/module-2-digital-twin/physics-gazebo/`
- Start with `setup.md` and follow the sequence
- Complete hands-on exercises with the simulation

### Chapter 2: Sensor Integration and ROS 2 Communication
- Navigate to `docusaurus/docs/module-2-digital-twin/sensors/`
- Follow the sequence from `lidar-setup.md` onwards
- Integrate sensors with the physics simulation

### Chapter 3: Unity Visualization and Rendering
- Navigate to `docusaurus/docs/module-2-digital-twin/rendering-unity/`
- Follow the sequence from `unity-setup.md` onwards
- Connect Unity visualization to the simulation

## Running the Documentation Site

```bash
cd docusaurus
npm start
```

This will start the Docusaurus documentation site locally at `http://localhost:3000`.

## Key ROS 2 Topics for Monitoring

- `/joint_states` - Robot joint positions
- `/scan` - LiDAR sensor data
- `/camera/image_raw` - Camera sensor data
- `/imu/data` - IMU sensor data
- `/tf` and `/tf_static` - Coordinate transforms

## Troubleshooting

### Common Issues

1. **Simulation not starting**: Ensure Gazebo and ROS 2 are properly installed and sourced
2. **Sensor data not publishing**: Check that sensor nodes are running and topics are connected
3. **Unity visualization not updating**: Verify ROS bridge connection and topic synchronization
4. **Documentation site not loading**: Check Node.js version and run `npm install` again

### Useful Commands

```bash
# Check running ROS 2 nodes
ros2 node list

# Check ROS 2 topics
ros2 topic list

# Echo a specific topic
ros2 topic echo /joint_states

# Check available services
ros2 service list
```

## Next Steps

After completing the quickstart:
1. Work through Chapter 1 exercises to understand physics simulation
2. Proceed to sensor integration in Chapter 2
3. Complete Unity visualization in Chapter 3
4. Experiment with custom robot models and environments