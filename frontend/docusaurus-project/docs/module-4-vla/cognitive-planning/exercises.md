# Cognitive Planning Exercises

## Overview
This document contains hands-on exercises for the Cognitive Planning chapter, focusing on converting voice commands to executable ROS 2 action sequences.

## Exercise 1: Basic Plan Generation

### Objective
Generate and examine a basic cognitive plan from a simple voice command.

### Steps
1. Launch the cognitive planning system:
   ```bash
   ros2 run vla_module command_parser
   ```

2. Send a simple voice command via command line:
   ```bash
   ros2 service call /vla/plan_actions vla_msgs/action/PlanActions "{command: {transcript: 'Move forward', confidence: 0.9, intent: 'NAVIGATION'}}"
   ```

3. Monitor the generated cognitive plan:
   ```bash
   ros2 topic echo /vla/cognitive_plan
   ```

4. Analyze the plan structure:
   - Number of action steps
   - Action types
   - Parameters
   - Dependencies

### Expected Results
- Cognitive plan should contain 1-2 action steps
- Action type should be NAVIGATION
- Parameters should include movement details
- No dependencies for simple command

### Verification
- Check that plan status is "READY"
- Verify action steps are executable
- Confirm estimated duration is reasonable

## Exercise 2: Complex Command Planning

### Objective
Plan actions for a complex multi-step command.

### Steps
1. Send a complex command:
   ```bash
   ros2 service call /vla/plan_actions vla_msgs/action/PlanActions "{command: {transcript: 'Go to kitchen and pick up red cup', confidence: 0.9, intent: 'COMPLEX'}}"
   ```

2. Examine the cognitive plan:
   ```bash
   ros2 topic echo /vla/cognitive_plan
   ```

3. Analyze the dependency structure:
   - Identify navigation vs manipulation steps
   - Check dependency relationships
   - Verify action sequencing

4. Monitor execution state:
   ```bash
   ros2 topic echo /vla/task_execution_state
   ```

### Expected Results
- Plan should contain multiple action steps
- Steps should include both navigation and manipulation
- Dependencies should sequence navigation before manipulation
- Estimated duration should reflect multiple steps

## Exercise 3: Parameter Extraction and Conversion

### Objective
Test the parameter extraction and conversion process.

### Steps
1. Test command with numerical parameters:
   ```bash
   ros2 service call /vla/plan_actions vla_msgs/action/PlanActions "{command: {transcript: 'Move forward 3 meters', confidence: 0.9, intent: 'NAVIGATION'}}"
   ```

2. Examine the converted parameters:
   - Check cognitive plan parameters
   - Verify distance parameter extraction
   - Confirm ROS 2 action parameter conversion

3. Test different parameter types:
   - Distances: "Move forward 2 meters", "Move backward 1.5 meters"
   - Angles: "Turn left 90 degrees", "Turn right 45 degrees"
   - Objects: "Pick up blue cup", "Find red ball"

### Expected Results
- Numerical values should be correctly extracted
- Distance units should be converted to meters
- Object names should be identified
- Parameters should be properly formatted for ROS 2 actions

## Exercise 4: Dependency Resolution

### Objective
Test the dependency resolution in cognitive planning.

### Steps
1. Plan a complex command with clear dependencies:
   ```bash
   ros2 service call /vla/plan_actions vla_msgs/action/PlanActions "{command: {transcript: 'Go to kitchen then pick up the red cup', confidence: 0.9, intent: 'COMPLEX'}}"
   ```

2. Analyze the dependency graph:
   - Check that navigation happens before manipulation
   - Verify dependency IDs match action IDs
   - Confirm execution order

3. Test dependency validation:
   - Attempt to execute manipulation before navigation
   - Verify system prevents invalid execution order

### Expected Results
- Navigation action should have no dependencies
- Manipulation action should depend on navigation completion
- Dependency IDs should match action IDs exactly
- Execution should follow dependency order

## Exercise 5: Plan Validation

### Objective
Test the cognitive plan validation process.

### Steps
1. Test with valid plan:
   ```bash
   ros2 service call /vla/validate_capability vla_msgs/srv/ValidateCapability "{capability_name: 'navigation'}"
   ```

2. Examine plan validation results:
   - Check required capabilities
   - Verify capability availability
   - Confirm plan status

3. Test with potentially invalid commands:
   - Commands requiring unavailable capabilities
   - Commands with impossible parameters
   - Commands with conflicting requirements

### Expected Results
- Valid plans should have status "READY"
- Invalid plans should have status "FAILED"
- Required capabilities should match action types
- System should handle invalid plans gracefully

## Exercise 6: Action Type Mapping

### Objective
Test the mapping of cognitive action types to ROS 2 actions.

### Steps
1. Generate navigation plan:
   ```bash
   ros2 service call /vla/plan_actions vla_msgs/action/PlanActions "{command: {transcript: 'Turn left', confidence: 0.9, intent: 'NAVIGATION'}}"
   ```

2. Generate manipulation plan:
   ```bash
   ros2 service call /vla/plan_actions vla_msgs/action/PlanActions "{command: {transcript: 'Pick up object', confidence: 0.9, intent: 'MANIPULATION'}}"
   ```

3. Generate perception plan:
   ```bash
   ros2 service call /vla/plan_actions vla_msgs/action/PlanActions "{command: {transcript: 'Find the ball', confidence: 0.9, intent: 'PERCEPTION'}}"
   ```

4. Examine each plan's action types and parameters.

### Expected Results
- Navigation commands should create NAVIGATION action types
- Manipulation commands should create MANIPULATION action types
- Perception commands should create PERCEPTION action types
- Parameters should be appropriate for each action type

## Exercise 7: Error Handling and Recovery

### Objective
Test how the planning system handles errors and edge cases.

### Steps
1. Test with unclear command:
   ```bash
   ros2 service call /vla/plan_actions vla_msgs/action/PlanActions "{command: {transcript: 'Do something', confidence: 0.5, intent: 'UNKNOWN'}}"
   ```

2. Test with command requiring unavailable capability:
   - Simulate capability unavailability
   - Observe planning behavior

3. Test with command with invalid parameters:
   - "Move forward -5 meters" (negative distance)
   - "Turn left 500 degrees" (impossible angle)

4. Monitor system logs for error handling.

### Expected Results
- Low-confidence commands should be handled appropriately
- Unavailable capabilities should be detected
- Invalid parameters should be rejected
- System should maintain stability

## Exercise 8: Performance Testing

### Objective
Evaluate the performance of the cognitive planning system.

### Steps
1. Measure planning time:
   - Time from command receipt to plan completion
   - Record multiple samples
   - Calculate average planning time

2. Monitor resource usage:
   - CPU usage during planning
   - Memory usage
   - Network usage (if applicable)

3. Test with varying command complexity:
   - Simple commands: 1-2 actions
   - Medium commands: 3-5 actions
   - Complex commands: 6+ actions

4. Record performance metrics for each category.

### Expected Results
- Planning time should be under 2 seconds for simple commands
- Memory usage should remain stable
- CPU usage should be reasonable during planning
- Performance should scale appropriately with complexity

## Exercise 9: Integration Testing

### Objective
Test the complete cognitive planning pipeline.

### Steps
1. Launch the full VLA system:
   ```bash
   ros2 launch vla_module vla_system.launch.py
   ```

2. Test the complete pipeline:
   - Voice command input
   - Cognitive planning
   - Action conversion
   - Task execution

3. Monitor all relevant topics:
   ```bash
   # In separate terminals:
   ros2 topic echo /vla/voice_command
   ros2 topic echo /vla/cognitive_plan
   ros2 topic echo /vla/task_execution_state
   ```

4. Issue various commands and observe the complete flow.

### Expected Results
- Complete pipeline should function end-to-end
- All topics should show appropriate messages
- Plans should be generated and executed correctly
- System should handle errors gracefully

## Exercise 10: Custom Command Planning

### Objective
Test the system with custom or unusual voice commands.

### Steps
1. Create custom commands:
   - "Move in a circle"
   - "Find the tallest object and go to it"
   - "Wait for my signal then move forward"

2. Plan each custom command:
   ```bash
   ros2 service call /vla/plan_actions vla_msgs/action/PlanActions "{command: {transcript: 'your_custom_command', confidence: 0.9, intent: 'CUSTOM'}}"
   ```

3. Analyze how the system handles custom commands:
   - Action decomposition
   - Parameter extraction
   - Plan validity

### Expected Results
- System should attempt to decompose custom commands
- Unknown commands should be handled gracefully
- Plan should be generated even for complex custom commands

## Assessment Criteria

### Success Metrics
- Plan generation success rate: >90%
- Planning time: \<2 seconds for simple commands
- Correct action type mapping: >95%
- Proper dependency resolution: >95%
- Appropriate error handling: 100%

### Deliverables
For each exercise, document:
1. Commands tested
2. Plans generated
3. Performance metrics
4. Any issues encountered
5. Parameters adjusted
6. Results observed

## Advanced Challenges

### Challenge 1: Optimized Planning
- Implement plan optimization techniques
- Reduce plan execution time
- Minimize redundant actions
- Measure improvement over baseline

### Challenge 2: Learning-Based Planning
- Analyze patterns in successful plans
- Identify common action sequences
- Propose improvements to planning algorithms
- Test enhanced planning approaches

### Challenge 3: Multi-Robot Planning
- Extend planning for multiple robots
- Coordinate actions between robots
- Handle resource conflicts
- Test coordination mechanisms

## Troubleshooting Guide

### Common Issues

#### Plan Generation Failures
- **Problem**: Planning service not responding
- **Solution**: Check that command_parser node is running

#### Incorrect Action Types
- **Problem**: Wrong action type generated
- **Solution**: Review intent extraction and action mapping

#### Dependency Issues
- **Problem**: Actions executing out of order
- **Solution**: Verify dependency resolution logic

#### Performance Problems
- **Problem**: Slow planning times
- **Solution**: Profile and optimize planning algorithms