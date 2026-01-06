# VLA API Contract

## Overview
This document defines the API contracts for the Vision-Language-Action (VLA) system that enables humanoid robots to receive voice commands via OpenAI Whisper, plan actions using cognitive planning, and execute tasks autonomously.

## ROS 2 Message Definitions

### VoiceCommand.msg
```
# Represents a voice command captured from user input
string id                    # Unique identifier for the command
builtin_interfaces/Time timestamp  # When the command was received
string transcript           # Transcribed text from speech
float32 confidence          # Confidence score from speech recognition (0.0-1.0)
string intent               # Parsed intent from the command
KeyValue[] parameters       # Parsed parameters from the command
string status               # Status of command processing (RECEIVED, PROCESSING, COMPLETED, FAILED)
```

### CognitivePlan.msg
```
# Represents a structured plan derived from a voice command
string id                    # Unique identifier for the plan
string voice_command_id      # Reference to the original voice command
builtin_interfaces/Time timestamp  # When the plan was created
ActionStep[] actions         # Sequence of actions to execute
string status                # Current status (PLANNING, READY, EXECUTING, COMPLETED, FAILED)
duration estimated_duration  # Estimated time to complete the plan
string[] required_capabilities  # Robot capabilities needed
```

### ActionStep.msg
```
# A single step in a cognitive plan
string id                    # Unique identifier for the action step
string action_type           # Type of action (NAVIGATION, MANIPULATION, PERCEPTION, etc.)
KeyValue[] parameters        # Parameters for the action
int32 priority               # Priority level for execution
string[] dependencies        # IDs of actions that must complete first
duration timeout             # Maximum time allowed for this step
string[] success_criteria    # Conditions that define success
```

### TaskExecutionState.msg
```
# Current state of task execution
string id                    # Unique identifier for the execution state
string cognitive_plan_id     # Reference to the plan being executed
string current_step          # ID of the currently executing step
string[] completed_steps     # IDs of completed steps
string[] failed_steps        # IDs of failed steps
float32 progress             # Overall progress (0.0-1.0)
string status                # Current execution status (IDLE, EXECUTING, PAUSED, COMPLETED, FAILED, CANCELLED)
string error_message         # Error details if status is FAILED
string feedback              # Human-readable feedback about execution
```

## ROS 2 Action Definitions

### ExecuteTask.action
```
# Action to execute a cognitive plan
CognitivePlan plan          # The plan to execute
---
TaskExecutionState result   # The final state of execution
---
TaskExecutionState feedback # Updates on execution progress
```

### PlanActions.action
```
# Action to generate a cognitive plan from a voice command
VoiceCommand command        # The voice command to plan for
---
CognitivePlan result        # The generated cognitive plan
---
CognitivePlan feedback      # Updates on planning progress
```

## Service Definitions

### ValidateCapability.srv
```
# Service to check if a robot capability is available
string capability_name      # Name of the capability to check
---
bool available              # Whether the capability is available
string error_message        # Error details if not available
```

## Topic Interfaces

### Published Topics
- `/vla/voice_command` (VoiceCommand)
  - Publishes recognized voice commands
  - Rate: On voice command detection
  - Purpose: Notify other components of new voice commands

- `/vla/cognitive_plan` (CognitivePlan)
  - Publishes generated cognitive plans
  - Rate: When plans are generated
  - Purpose: Notify execution system of new plans

- `/vla/task_execution_state` (TaskExecutionState)
  - Publishes current state of task execution
  - Rate: When state changes
  - Purpose: Provide real-time status updates

- `/vla/system_status` (String)
  - Publishes overall system status
  - Rate: Periodic updates (1 Hz)
  - Purpose: Monitor system health

### Subscribed Topics
- `/audio_input` (sensor_msgs/AudioData)
  - Receives raw audio data from robot microphones
  - Rate: Continuous audio stream
  - Purpose: Capture voice commands

- `/robot_status` (String)
  - Receives robot status updates
  - Rate: When robot status changes
  - Purpose: Monitor robot availability for tasks

## Action Interfaces

### `/vla/execute_task` (ExecuteTask)
- **Purpose**: Execute a cognitive plan on the robot
- **Goal**: Submit a cognitive plan for execution
- **Feedback**: Receive updates on execution progress
- **Result**: Get final execution state when complete
- **Usage**: When a cognitive plan is ready for execution

### `/vla/plan_actions` (PlanActions)
- **Purpose**: Generate a cognitive plan from a voice command
- **Goal**: Submit a voice command for planning
- **Feedback**: Receive updates on planning progress
- **Result**: Get the generated cognitive plan
- **Usage**: When a voice command needs to be converted to actions

## Service Interfaces

### `/vla/validate_capability` (ValidateCapability)
- **Purpose**: Check if the robot has required capabilities
- **Request**: Capability name to validate
- **Response**: Availability status and error details
- **Usage**: Before executing actions to ensure robot capability

## Error Handling

### Common Error Codes
- `VOICE_RECOGNITION_FAILED` (400): Voice command could not be recognized
- `PLAN_GENERATION_FAILED` (500): Cognitive plan could not be generated
- `EXECUTION_FAILED` (500): Task execution failed during runtime
- `CAPABILITY_UNAVAILABLE` (409): Required robot capability is not available
- `TIMEOUT_ERROR` (408): Operation timed out

### Error Response Format
All errors follow the ROS 2 standard error response pattern with appropriate status codes and descriptive messages.

## Validation Rules

### VoiceCommand Validation
- `transcript` must not be empty
- `confidence` must be between 0.0 and 1.0
- `status` must be one of the defined enum values

### CognitivePlan Validation
- `actions` array must not be empty
- `estimated_duration` must be positive
- `status` must be one of the defined enum values

### ActionStep Validation
- `action_type` must be a valid action type
- `priority` must be non-negative
- `timeout` must be positive

### TaskExecutionState Validation
- `progress` must be between 0.0 and 1.0
- `status` must be one of the defined enum values