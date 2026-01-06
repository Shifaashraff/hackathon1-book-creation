# Vision-Language-Action (VLA) Module

This module implements a complete system for voice-command-based humanoid robot control. The system integrates OpenAI Whisper for voice recognition, cognitive planning for converting commands to action sequences, and task execution for controlling the robot.

## Architecture

The VLA system consists of the following components:

### Voice Recognition
- `audio_capture`: Captures audio from microphone with noise filtering and voice activity detection
- `whisper_interface`: Integrates with OpenAI Whisper API for speech-to-text conversion
- `voice_processor`: Processes and validates voice commands before planning

### Cognitive Planning
- `command_parser`: Parses voice commands into structured intents and parameters
- `task_planner`: Generates cognitive plans using hierarchical task network (HTN) approach
- `action_converter`: Converts cognitive plans to detailed ROS 2 action parameters

### Task Execution
- `task_execution_manager`: Executes cognitive plans by interfacing with ROS 2 action servers

## Message Types

- `VoiceCommand`: Represents a recognized voice command with transcript, confidence, intent, and parameters
- `CognitivePlan`: Represents a planned sequence of actions with dependencies and requirements
- `ActionStep`: Represents a single action step in a cognitive plan
- `TaskExecutionState`: Tracks the current state of task execution

## Action Types

- `ExecuteTask`: Action for executing cognitive plans
- `PlanActions`: Action for generating cognitive plans from voice commands
- `ExecuteActionStep`: Action for executing individual action steps

## Service Types

- `ValidateCapability`: Service for validating robot capabilities

## Launching the System

The complete system can be launched using the provided launch file:

```bash
ros2 launch vla_module vla_system.launch.py
```

This will start all necessary nodes for voice recognition, cognitive planning, and task execution.

## Testing

A test publisher is available to send sample voice commands:

```bash
ros2 run vla_module voice_command_publisher
```

## Configuration

The system can be configured through ROS 2 parameters:

- `audio_rate`: Audio sampling rate (default: 16000 Hz)
- `noise_threshold`: Threshold for voice activity detection (default: 0.01)
- `min_confidence`: Minimum confidence for voice commands (default: 0.7)
- `max_plan_steps`: Maximum number of steps in a plan (default: 10)
- `enable_validation`: Enable plan validation (default: True)

## Environment Variables

- `OPENAI_API_KEY`: Required for Whisper API access

## ROS 2 Topics

- `/audio_input`: Raw audio data from microphone
- `/vla/voice_command`: Recognized voice commands
- `/vla/processed_voice_command`: Processed voice commands ready for planning
- `/vla/cognitive_plan`: Generated cognitive plans
- `/vla/task_execution_state`: Current task execution state

## ROS 2 Services/Actions

- `/vla/plan_actions`: Service for planning actions from voice commands
- `/vla/execute_task`: Action for executing cognitive plans
- `/vla/validate_capability`: Service for capability validation

## Dependencies

- ROS 2 Humble Hawksbill
- OpenAI Python library
- PyAudio
- NumPy
- Isaac ROS packages
- Navigation2 (Nav2)