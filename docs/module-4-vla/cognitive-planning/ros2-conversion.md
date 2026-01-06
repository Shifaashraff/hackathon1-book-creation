# ROS 2 Conversion Guide

## Overview
This guide explains how cognitive plans are converted to ROS 2 action messages and topics for execution on the humanoid robot.

## Conversion Architecture

The ROS 2 conversion system maps high-level cognitive plans to specific ROS 2 interfaces:

1. **Cognitive Plan**: High-level plan with abstract actions
2. **Action Conversion**: Mapping to specific ROS 2 actions
3. **Message Generation**: Creating ROS 2 messages for execution
4. **Topic Publication**: Sending messages to robot systems

## ROS 2 Message Types

### Voice Command Message
The `VoiceCommand` message carries voice command information:

```yaml
# vla_msgs/msg/VoiceCommand.msg
string id                    # Unique identifier for the command
builtin_interfaces/Time timestamp  # When the command was received
string transcript           # Transcribed text from speech
float32 confidence          # Confidence score from speech recognition (0.0-1.0)
string intent               # Parsed intent from the command
string[] parameters       # Parsed parameters from the command
string status               # Status of command processing (RECEIVED, PROCESSING, COMPLETED, FAILED)
```

### Cognitive Plan Message
The `CognitivePlan` message represents the structured plan:

```yaml
# vla_msgs/msg/CognitivePlan.msg
string id                    # Unique identifier for the plan
string voice_command_id      # Reference to the original voice command
builtin_interfaces/Time timestamp  # When the plan was created
ActionStep[] actions         # Sequence of actions to execute
string status                # Current status (PLANNING, READY, EXECUTING, COMPLETED, FAILED)
duration estimated_duration  # Estimated time to complete the plan
string[] required_capabilities  # Robot capabilities needed
```

### Action Step Message
The `ActionStep` message defines individual actions:

```yaml
# vla_msgs/msg/ActionStep.msg
string id                    # Unique identifier for the action step
string action_type           # Type of action (NAVIGATION, MANIPULATION, PERCEPTION, etc.)
string[] parameters        # Parameters for the action
int32 priority               # Priority level for execution
string[] dependencies        # IDs of actions that must complete first
duration timeout             # Maximum time allowed for this step
string[] success_criteria    # Conditions that define success
```

### Task Execution State Message
The `TaskExecutionState` message tracks execution progress:

```yaml
# vla_msgs/msg/TaskExecutionState.msg
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

## Conversion Process

### 1. Action Type Mapping

Each cognitive action type maps to specific ROS 2 interfaces:

#### Navigation Actions
- **Action Type**: `NAVIGATION`
- **ROS 2 Action**: `nav2_msgs/action/NavigateToPose`
- **Parameters Conversion**:
  - `navigate_to:location` → `NavigateToPose.Goal.pose`
  - `move_forward_distance:2m` → Custom velocity commands
  - `turn_left:90` → Custom rotation commands

#### Manipulation Actions
- **Action Type**: `MANIPULATION`
- **ROS 2 Action**: `control_msgs/action/FollowJointTrajectory`
- **Parameters Conversion**:
  - `pick_object:cup` → Gripper control sequences
  - `place_object:table` → Placement trajectory planning

#### Perception Actions
- **Action Type**: `PERCEPTION`
- **ROS 2 Action**: Custom perception actions
- **Parameters Conversion**:
  - `detect_object:red_ball` → Object detection parameters
  - `look_for:cup` → Camera control parameters

### 2. Parameter Conversion

Parameters in cognitive plans are converted to ROS 2 message fields:

```python
def convert_navigation_params(cognitive_params):
    """Convert cognitive navigation parameters to ROS 2 navigation parameters"""
    ros2_params = {}

    for param in cognitive_params:
        if param.startswith("navigate_to:"):
            location = param.split("navigate_to:")[1]
            # Convert location to specific coordinates
            pose = get_pose_for_location(location)
            ros2_params['pose'] = pose

        elif param.startswith("move_forward_distance:"):
            distance_str = param.split("move_forward_distance:")[1]
            distance = float(distance_str.replace("m", ""))
            ros2_params['linear_x'] = 0.5  # m/s
            ros2_params['duration'] = distance / 0.5

    return ros2_params
```

### 3. Dependency Resolution

Dependencies in cognitive plans are mapped to ROS 2 action sequencing:

```python
def resolve_action_dependencies(actions):
    """Resolve dependencies and create execution sequence"""
    # Build dependency graph
    dependency_graph = {}
    for action in actions:
        dependency_graph[action.id] = action.dependencies

    # Create execution sequence
    execution_sequence = topological_sort(dependency_graph)

    return execution_sequence
```

## ROS 2 Action Servers

### ExecuteTask Action Server
The `/vla/execute_task` action server executes cognitive plans:

- **Action Type**: `vla_msgs/action/ExecuteTask`
- **Goal**: `CognitivePlan` to execute
- **Feedback**: `TaskExecutionState` updates
- **Result**: Final `TaskExecutionState`

### PlanActions Action Server
The `/vla/plan_actions` action server generates cognitive plans:

- **Action Type**: `vla_msgs/action/PlanActions`
- **Goal**: `VoiceCommand` to plan for
- **Feedback**: Planning progress updates
- **Result**: Generated `CognitivePlan`

## Topic Interfaces

### Published Topics
- `/vla/voice_command`: Recognized voice commands
- `/vla/cognitive_plan`: Generated cognitive plans
- `/vla/task_execution_state`: Execution state updates
- `/vla/system_status`: Overall system status

### Subscribed Topics
- `/audio_input`: Raw audio data from microphones
- `/robot_status`: Robot status updates
- Various navigation and manipulation topics

## Service Interfaces

### ValidateCapability Service
The `/vla/validate_capability` service validates robot capabilities:

- **Service Type**: `vla_msgs/srv/ValidateCapability`
- **Request**: Capability name to check
- **Response**: Availability and error details

## Conversion Parameters

### Configuration Options
| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_detailed_conversion` | True | Enable detailed ROS 2 action mapping |
| `navigation_action_server` | "/navigate_to_pose" | Navigation action server name |
| `manipulation_action_server` | "/follow_joint_trajectory" | Manipulation action server name |
| `conversion_timeout` | 30.0 | Maximum time for conversion in seconds |

### Performance Tuning
- **enable_detailed_conversion**: Set to False for faster but less detailed conversion
- **conversion_timeout**: Adjust based on system complexity
- **action_server_names**: Configure based on available robot systems

## Error Handling

### Conversion Errors
Common conversion errors and handling:

1. **Unknown Action Type**
   - **Cause**: Action type not supported
   - **Solution**: Log error and skip action or use default mapping

2. **Invalid Parameters**
   - **Cause**: Parameters don't match expected format
   - **Solution**: Validate parameters before conversion

3. **Missing Dependencies**
   - **Cause**: Action depends on non-existent actions
   - **Solution**: Validate dependency graph before execution

### Error Recovery
```python
def safe_convert_action(action):
    """Safely convert an action with error handling"""
    try:
        # Validate action type
        if action.action_type not in SUPPORTED_ACTION_TYPES:
            raise ValueError(f"Unsupported action type: {action.action_type}")

        # Convert parameters
        ros2_action = convert_action_parameters(action)

        return ros2_action
    except Exception as e:
        # Log error and return fallback action
        logger.error(f"Conversion failed for action {action.id}: {e}")
        return create_fallback_action(action)
```

## Performance Optimization

### Conversion Efficiency
- **Caching**: Cache conversion results for repeated patterns
- **Batching**: Convert multiple actions together when possible
- **Asynchronous**: Perform conversion in background threads
- **Preprocessing**: Pre-process common action patterns

### Memory Management
- **Message Pooling**: Reuse message objects where possible
- **Cleanup**: Remove completed conversion data
- **Streaming**: Process large plans in chunks

## Integration Examples

### Navigation Integration
```python
def convert_navigation_action(cognitive_action):
    """Convert cognitive navigation action to ROS 2 NavigateToPose action"""
    from nav2_msgs.action import NavigateToPose

    goal = NavigateToPose.Goal()

    # Convert cognitive parameters to navigation pose
    for param in cognitive_action.parameters:
        if param.startswith("navigate_to:"):
            location = param.split("navigate_to:")[1]
            goal.pose = get_pose_for_location(location)
        elif param.startswith("move_forward_distance:"):
            # Convert to velocity-based movement
            pass

    return goal
```

### Manipulation Integration
```python
def convert_manipulation_action(cognitive_action):
    """Convert cognitive manipulation action to ROS 2 FollowJointTrajectory action"""
    from control_msgs.action import FollowJointTrajectory

    goal = FollowJointTrajectory.Goal()

    # Convert cognitive parameters to joint trajectories
    for param in cognitive_action.parameters:
        if param.startswith("pick_object:"):
            object_name = param.split("pick_object:")[1]
            goal.trajectory = create_grasp_trajectory(object_name)

    return goal
```

## Testing and Validation

### Conversion Testing
Test conversion with various command types:

1. **Simple Commands**: "Move forward"
2. **Complex Commands**: "Go to kitchen and pick up red cup"
3. **Parameter Variations**: Different distances, objects, locations
4. **Error Conditions**: Invalid commands, missing parameters

### Validation Techniques
- **Unit Tests**: Test individual conversion functions
- **Integration Tests**: Test end-to-end conversion pipeline
- **Simulation Tests**: Validate converted actions in simulation
- **Regression Tests**: Ensure changes don't break existing functionality

## Best Practices

### Conversion Design
- **Modularity**: Keep conversion logic modular and testable
- **Extensibility**: Design for easy addition of new action types
- **Validation**: Validate inputs before conversion
- **Logging**: Log conversion decisions for debugging

### Performance
- **Efficiency**: Optimize conversion for real-time performance
- **Resource Management**: Manage memory and computation resources
- **Error Handling**: Gracefully handle conversion failures
- **Monitoring**: Monitor conversion performance and errors

## Advanced Topics

### Custom Action Types
For specialized applications, implement custom action types:

```python
def register_custom_action_type(action_type, converter_function):
    """Register a custom action type with its converter"""
    CUSTOM_CONVERTERS[action_type] = converter_function
```

### Dynamic Parameter Conversion
For flexible parameter handling:

```python
def convert_parameters_dynamically(params, action_type):
    """Dynamically convert parameters based on action type"""
    # Use action type-specific conversion rules
    converter = PARAMETER_CONVERTERS.get(action_type, default_converter)
    return converter(params)
```

## Troubleshooting

### Common Issues

#### Parameter Mapping Errors
- **Problem**: Parameters not mapping correctly
- **Solution**: Verify parameter format and mapping rules

#### Action Server Connection Issues
- **Problem**: Cannot connect to action servers
- **Solution**: Verify action server names and availability

#### Conversion Performance
- **Problem**: Slow conversion times
- **Solution**: Optimize conversion algorithms and parameters

## Next Steps

- Complete the [Cognitive Planning Exercises](./exercises.md) for hands-on practice
- Review [Planning Algorithms Guide](./planning-algorithms.md) for algorithm details
- Explore [Task Execution Guide](../autonomous-execution/task-execution.md) for task execution