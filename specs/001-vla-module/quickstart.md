# Quickstart Guide: Vision-Language-Action (VLA) Module

## Overview
This guide will help you set up and run the Vision-Language-Action system that enables humanoid robots to receive voice commands via OpenAI Whisper, plan actions using cognitive planning, and execute tasks autonomously.

## Prerequisites

### Hardware Requirements
- NVIDIA GPU (RTX 3080 or better recommended)
- Ubuntu 22.04 LTS
- Humanoid robot with ROS 2 compatibility (simulated via Isaac Sim for development)

### Software Requirements
- ROS 2 Humble Hawksbill
- Isaac Sim 2023.1+
- Isaac ROS packages
- Nav2 navigation stack
- Python 3.8+
- OpenAI API key

### Environment Setup
1. Install ROS 2 Humble following the official guide
2. Set up Isaac Sim with humanoid robot assets
3. Configure OpenAI API key in environment variables:
   ```bash
   export OPENAI_API_KEY="your-api-key-here"
   ```

## Installation

### 1. Clone the Repository
```bash
cd ~/isaac_ros_ws/src
git clone <repository-url>
```

### 2. Install Python Dependencies
```bash
pip3 install openai numpy scipy pyaudio
```

### 3. Build ROS 2 Packages
```bash
cd ~/isaac_ros_ws
colcon build --packages-select vla_module
source install/setup.bash
```

## Running the System

### 1. Launch Isaac Sim Environment
```bash
# Start Isaac Sim with humanoid robot in a test environment
```

### 2. Launch VLA System
```bash
# Launch the complete VLA system
ros2 launch vla_module vla_system.launch.py
```

### 3. Test Voice Commands
Once the system is running, you can issue voice commands like:
- "Move forward"
- "Turn left"
- "Go to the kitchen"
- "Pick up the red cup"

## Architecture Overview

The VLA system consists of three main components:

1. **Voice Recognition**: Uses OpenAI Whisper to convert speech to text
2. **Cognitive Planning**: Transforms voice commands into executable action sequences
3. **Task Execution**: Executes the planned actions using ROS 2 interfaces

## Key ROS 2 Interfaces

### Published Topics
- `/vla/voice_command` - Voice commands with transcription and confidence
- `/vla/cognitive_plan` - Generated plans with action sequences
- `/vla/task_execution_state` - Current state of task execution

### Subscribed Topics
- `/audio_input` - Raw audio data from robot's microphones
- Various ROS 2 topics for navigation, manipulation, and perception

### Actions
- `/vla/execute_task` - Action server for executing cognitive plans
- `/vla/plan_actions` - Action server for cognitive planning

## Basic Usage Example

1. Ensure Isaac Sim is running with a humanoid robot
2. Launch the VLA system: `ros2 launch vla_module vla_system.launch.py`
3. Speak a command like "Move forward 1 meter"
4. Observe the robot executing the command
5. Check the console for processing status and feedback

## Troubleshooting

### Voice Recognition Issues
- Ensure microphone is properly connected and configured
- Check that the OpenAI API key is correctly set
- Verify network connectivity to OpenAI services

### Task Execution Problems
- Confirm robot is properly simulated in Isaac Sim
- Verify ROS 2 network connectivity between components
- Check that required robot capabilities are available

## Next Steps

- Explore the cognitive planning algorithms in detail
- Customize the voice command vocabulary
- Extend the system with additional robot capabilities
- Complete the capstone project implementing complex multi-step tasks