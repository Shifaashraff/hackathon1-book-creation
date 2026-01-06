# Planning Algorithms Guide

## Overview
This guide covers the cognitive planning algorithms used in the Vision-Language-Action (VLA) system to convert voice commands into executable action sequences for humanoid robots.

## Planning Architecture

The cognitive planning system uses a hierarchical task network (HTN) approach to break down high-level voice commands into executable ROS 2 actions. The system consists of:

1. **Command Parser**: Interprets voice commands and extracts intent
2. **Task Planner**: Creates hierarchical action plans
3. **Action Converter**: Maps plans to specific ROS 2 actions
4. **Plan Validator**: Ensures plans are executable with available capabilities

## Hierarchical Task Network (HTN) Planning

### Concept
HTN planning decomposes complex tasks into smaller, manageable subtasks. Each high-level task is broken down into more specific subtasks until primitive actions are reached.

### Implementation in VLA
```
High-level Command: "Go to kitchen and pick up red cup"
├── Navigation Task: "Navigate to kitchen"
│   ├── Path Planning: "Calculate path to kitchen"
│   ├── Obstacle Avoidance: "Avoid obstacles during navigation"
│   └── Localization: "Confirm arrival at kitchen"
└── Manipulation Task: "Pick up red cup"
    ├── Object Detection: "Locate red cup"
    ├── Approach: "Move gripper to cup position"
    └── Grasp: "Grasp the cup"
```

### Algorithm Steps
1. **Parse Command**: Extract intent and parameters from voice command
2. **Decompose Task**: Break down into subtasks using predefined methods
3. **Order Actions**: Determine execution order and dependencies
4. **Validate Plan**: Check feasibility with robot capabilities
5. **Optimize**: Improve efficiency where possible

## Planning Algorithms

### 1. Command Decomposition Algorithm

#### Purpose
Breaks down complex voice commands into individual action steps.

#### Process
1. Analyze command structure
2. Identify action types (navigation, manipulation, perception)
3. Extract parameters and constraints
4. Create dependency graph

#### Example
```python
def decompose_command(command):
    # Parse command into subtasks
    subtasks = []

    if "go to" in command:
        location = extract_location(command)
        subtasks.append(("NAVIGATION", f"navigate_to:{location}"))

    if "pick up" in command:
        object = extract_object(command)
        subtasks.append(("MANIPULATION", f"pick_object:{object}"))

    return subtasks
```

### 2. Dependency Resolution Algorithm

#### Purpose
Determines the correct order of action execution based on dependencies.

#### Process
1. Build dependency graph
2. Identify independent tasks
3. Order tasks respecting dependencies
4. Group parallelizable tasks

#### Example
```python
def resolve_dependencies(actions):
    # Build dependency graph
    graph = build_dependency_graph(actions)

    # Topological sort to get execution order
    execution_order = topological_sort(graph)

    return execution_order
```

### 3. Capability Validation Algorithm

#### Purpose
Ensures the robot has necessary capabilities to execute the plan.

#### Process
1. Extract required capabilities from plan
2. Check robot's available capabilities
3. Validate capability availability
4. Adjust plan if necessary

#### Example
```python
def validate_capabilities(plan, robot_capabilities):
    required_caps = extract_required_capabilities(plan)

    for cap in required_caps:
        if cap not in robot_capabilities:
            raise CapabilityNotAvailableError(f"Capability {cap} not available")

    return True
```

## Action Types and Planning

### Navigation Planning
For navigation commands, the system plans:

- **Path Planning**: Calculate optimal route to destination
- **Obstacle Avoidance**: Handle dynamic and static obstacles
- **Localization**: Maintain awareness of position
- **Recovery**: Handle navigation failures

### Manipulation Planning
For manipulation commands, the system plans:

- **Grasp Planning**: Calculate gripper position and orientation
- **Approach Planning**: Plan safe approach trajectory
- **Placement Planning**: Determine object placement location
- **Force Control**: Apply appropriate grip forces

### Perception Planning
For perception commands, the system plans:

- **Object Detection**: Locate specific objects
- **Scene Understanding**: Interpret environment
- **Feature Extraction**: Identify relevant features
- **Tracking**: Follow moving objects

## Plan Representation

### Cognitive Plan Structure
A cognitive plan contains:

- **ID**: Unique identifier
- **Voice Command ID**: Reference to original command
- **Timestamp**: When plan was created
- **Actions**: List of action steps
- **Status**: Current planning/execution status
- **Estimated Duration**: Expected time to complete
- **Required Capabilities**: Capabilities needed for execution

### Action Step Structure
Each action step contains:

- **ID**: Unique identifier
- **Action Type**: Navigation, manipulation, perception, etc.
- **Parameters**: Specific parameters for the action
- **Priority**: Execution priority
- **Dependencies**: Actions that must complete first
- **Timeout**: Maximum allowed execution time
- **Success Criteria**: Conditions for successful completion

## Planning Parameters

### Configuration Options
| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_plan_steps` | 10 | Maximum number of steps in a plan |
| `enable_validation` | True | Whether to validate plans |
| `planning_timeout` | 30.0 | Maximum time for planning in seconds |
| `dependency_resolution_strategy` | "topological" | Method for resolving dependencies |

### Performance Tuning
- **max_plan_steps**: Adjust based on command complexity
- **planning_timeout**: Balance between thoroughness and responsiveness
- **validation**: Enable for safety-critical applications

## Plan Validation

### Pre-execution Validation
Before execution, plans are validated for:

- **Completeness**: All required actions present
- **Feasibility**: Robot has necessary capabilities
- **Consistency**: Actions are logically consistent
- **Safety**: Plan doesn't violate safety constraints

### Runtime Validation
During execution, the system validates:

- **Preconditions**: Conditions required for action execution
- **Resource Availability**: Capabilities still available
- **Progress**: Plan making expected progress
- **Safety**: Maintaining safe operation

## Error Handling and Recovery

### Planning Failures
If planning fails, the system:
1. Logs the error with context
2. Attempts alternative planning strategies
3. Falls back to simple actions if needed
4. Reports error to user interface

### Execution Failures
If execution fails:
1. Identify which action failed
2. Determine cause of failure
3. Attempt recovery if possible
4. Update plan as needed
5. Continue or terminate based on failure type

## Performance Optimization

### Planning Efficiency
To optimize planning performance:

- **Caching**: Cache frequently used plans
- **Abstraction**: Use abstract planning for common patterns
- **Parallelization**: Plan independent subtasks in parallel
- **Pruning**: Remove infeasible branches early

### Memory Management
- **Plan Size**: Limit plan complexity to manageable levels
- **Cleanup**: Remove completed plans from memory
- **Streaming**: Process large plans in chunks if needed

## Integration with ROS 2

### Action Server Interface
The planning system uses the `/vla/plan_actions` action server:
- **Goal**: Voice command to be planned
- **Feedback**: Progress updates during planning
- **Result**: Generated cognitive plan

### Message Types
- **VoiceCommand**: Input from voice recognition
- **CognitivePlan**: Output of planning process
- **ActionStep**: Individual action within a plan
- **TaskExecutionState**: Current execution status

## Testing and Validation

### Plan Quality Metrics
- **Completeness**: Does the plan achieve the goal?
- **Optimality**: Is the plan efficient?
- **Robustness**: How does it handle failures?
- **Safety**: Does it maintain safety constraints?

### Validation Techniques
- **Simulation Testing**: Test plans in simulation first
- **Unit Testing**: Test individual planning components
- **Integration Testing**: Test complete planning pipeline
- **Regression Testing**: Ensure changes don't break existing functionality

## Advanced Topics

### Learning-Based Planning
Future enhancements could include:
- Learning from execution failures
- Adapting to specific environments
- Improving efficiency through experience

### Multi-Robot Planning
For systems with multiple robots:
- Coordinating actions between robots
- Avoiding conflicts in shared spaces
- Distributing tasks effectively

## Best Practices

### Plan Design
- Keep plans modular and reusable
- Handle edge cases explicitly
- Validate assumptions before execution
- Design for graceful degradation

### Performance
- Profile planning performance regularly
- Optimize frequently executed paths
- Balance planning thoroughness with response time
- Monitor resource usage during planning

## Next Steps

- Review [ROS 2 Conversion Guide](./ros2-conversion.md) for action mapping
- Complete the [Cognitive Planning Exercises](./exercises.md) for hands-on practice
- Explore [Task Execution Guide](../autonomous-execution/task-execution.md) for task execution