# Data Model: Vision-Language-Action (VLA) Module

## Overview
Data structures for the VLA system that enables humanoid robots to receive voice commands, plan actions cognitively, and execute tasks autonomously.

## Core Entities

### VoiceCommand
**Description**: Represents a voice command captured from user input
**Fields**:
- `id`: String - Unique identifier for the command
- `timestamp`: DateTime - When the command was received
- `raw_audio`: Binary - Raw audio data (optional, for debugging)
- `transcript`: String - Transcribed text from speech
- `confidence`: Float - Confidence score from speech recognition (0.0-1.0)
- `intent`: String - Parsed intent from the command
- `parameters`: Map<String, Any> - Parsed parameters from the command
- `status`: Enum - Status of command processing (RECEIVED, PROCESSING, COMPLETED, FAILED)

### CognitivePlan
**Description**: Represents a structured plan derived from a voice command
**Fields**:
- `id`: String - Unique identifier for the plan
- `voice_command_id`: String - Reference to the original voice command
- `timestamp`: DateTime - When the plan was created
- `actions`: List<ActionStep> - Sequence of actions to execute
- `status`: Enum - Current status (PLANNING, READY, EXECUTING, COMPLETED, FAILED)
- `estimated_duration`: Duration - Estimated time to complete the plan
- `required_capabilities`: List<String> - Robot capabilities needed

### ActionStep
**Description**: A single step in a cognitive plan
**Fields**:
- `id`: String - Unique identifier for the action step
- `action_type`: String - Type of action (NAVIGATION, MANIPULATION, PERCEPTION, etc.)
- `parameters`: Map<String, Any> - Parameters for the action
- `priority`: Int - Priority level for execution
- `dependencies`: List<String> - IDs of actions that must complete first
- `timeout`: Duration - Maximum time allowed for this step
- `success_criteria`: List<String> - Conditions that define success

### TaskExecutionState
**Description**: Current state of task execution
**Fields**:
- `id`: String - Unique identifier for the execution state
- `cognitive_plan_id`: String - Reference to the plan being executed
- `current_step`: String - ID of the currently executing step
- `completed_steps`: List<String> - IDs of completed steps
- `failed_steps`: List<String> - IDs of failed steps
- `progress`: Float - Overall progress (0.0-1.0)
- `status`: Enum - Current execution status (IDLE, EXECUTING, PAUSED, COMPLETED, FAILED, CANCELLED)
- `error_message`: String - Error details if status is FAILED
- `feedback`: String - Human-readable feedback about execution

### RobotCapability
**Description**: Represents a capability of the humanoid robot
**Fields**:
- `id`: String - Unique identifier for the capability
- `name`: String - Name of the capability (e.g., "navigation", "object_manipulation")
- `description`: String - Human-readable description
- `available`: Boolean - Whether the capability is currently available
- `requires_calibration`: Boolean - Whether the capability requires calibration

## ROS 2 Message Definitions

### VoiceCommand.msg
```
string id
builtin_interfaces/Time timestamp
string transcript
float32 confidence
string intent
KeyValue[] parameters
string status
```

### CognitivePlan.msg
```
string id
string voice_command_id
builtin_interfaces/Time timestamp
ActionStep[] actions
string status
duration estimated_duration
string[] required_capabilities
```

### ActionStep.msg
```
string id
string action_type
KeyValue[] parameters
int32 priority
string[] dependencies
duration timeout
string[] success_criteria
```

### TaskExecutionState.msg
```
string id
string cognitive_plan_id
string current_step
string[] completed_steps
string[] failed_steps
float32 progress
string status
string error_message
string feedback
```

## Relationships
- VoiceCommand → CognitivePlan (1 to 1)
- CognitivePlan → ActionStep (1 to many)
- CognitivePlan → TaskExecutionState (1 to 1)
- TaskExecutionState → ActionStep (many to many - through completed_steps, failed_steps)

## State Transitions

### VoiceCommand States
- RECEIVED → PROCESSING (when cognitive planning begins)
- PROCESSING → COMPLETED (when plan is ready)
- PROCESSING → FAILED (when planning fails)

### CognitivePlan States
- PLANNING → READY (when plan is complete)
- READY → EXECUTING (when execution starts)
- EXECUTING → COMPLETED (when all steps complete successfully)
- EXECUTING → FAILED (when execution fails)

### TaskExecutionState States
- IDLE → EXECUTING (when task execution begins)
- EXECUTING → COMPLETED (when all actions complete)
- EXECUTING → FAILED (when action fails)
- EXECUTING → PAUSED (when execution is paused)
- PAUSED → EXECUTING (when execution resumes)
- Any state → CANCELLED (when execution is cancelled)