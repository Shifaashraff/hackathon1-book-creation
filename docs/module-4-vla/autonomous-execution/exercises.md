# Autonomous Execution Exercises

## Overview
This document contains hands-on exercises for the Autonomous Execution chapter, focusing on executing cognitive plans on humanoid robots using perception, navigation, and manipulation capabilities.

## Exercise 1: Basic Task Execution Setup

### Objective
Set up and verify the basic task execution system.

### Steps
- Launch the complete VLA system:
   ```bash
   ros2 launch vla_module vla_system.launch.py
   ```

- Verify all components are running:
   ```bash
   ros2 node list
   ```

- Check for the task execution manager:
   ```bash
   ros2 node info /task_execution_manager
   ```

- Verify required topics are available:
   ```bash
   ros2 topic list | grep vla
   ```

- Test the execution state publisher:
   ```bash
   ros2 topic echo /vla/task_execution_state
   ```

### Expected Results
- All VLA nodes should be running
- Task execution manager should be available
- VLA topics should be listed
- Execution state topic should publish regularly

### Troubleshooting
- If nodes are not running, check the launch file
- Verify all dependencies are installed
- Check ROS 2 network configuration

## Exercise 2: Simple Task Execution

### Objective
Execute a simple navigation task using the autonomous execution system.

### Steps
- Send a simple navigation command:
   ```bash
   ros2 action send_goal /vla/execute_task vla_msgs/action/ExecuteTask "{plan: {id: 'simple_nav_plan', voice_command_id: 'cmd1', actions: [{id: 'nav1', action_type: 'NAVIGATION', parameters: ['move_forward_distance:1m'], priority: 1, dependencies: [], timeout: {sec: 30, nanosec: 0}, success_criteria: ['navigation_completed']}], status: 'READY', estimated_duration: {sec: 10, nanosec: 0}, required_capabilities: ['navigation']}}"
   ```

- Monitor execution progress:
   ```bash
   ros2 topic echo /vla/task_execution_state
   ```

- Observe the robot's behavior in Isaac Sim

- Check for successful completion in the execution state

### Expected Results
- Task execution should start
- Navigation action should execute
- Robot should move forward approximately 1 meter
- Execution state should show completion

### Verification
- Check that progress increases over time
- Verify final state is "COMPLETED"
- Confirm no errors in the logs

## Exercise 3: Complex Multi-Step Task

### Objective
Execute a complex task with multiple action steps and dependencies.

### Steps
- Create a multi-step cognitive plan with dependencies:
   ```bash
   ros2 action send_goal /vla/execute_task vla_msgs/action/ExecuteTask "{plan: {id: 'complex_plan', voice_command_id: 'cmd2', actions: [{id: 'nav1', action_type: 'NAVIGATION', parameters: ['navigate_to:kitchen'], priority: 1, dependencies: [], timeout: {sec: 60, nanosec: 0}, success_criteria: ['navigation_completed']}, {id: 'manip1', action_type: 'MANIPULATION', parameters: ['pick_object:red_cup'], priority: 2, dependencies: ['nav1'], timeout: {sec: 45, nanosec: 0}, success_criteria: ['manipulation_completed']}], status: 'READY', estimated_duration: {sec: 120, nanosec: 0}, required_capabilities: ['navigation', 'manipulation']}}"
   ```

- Monitor the execution state:
   ```bash
   ros2 topic echo /vla/task_execution_state
   ```

- Observe the dependency handling:
   - Navigation should complete before manipulation
   - Check that manipulation starts after navigation

- Verify the complete task execution

### Expected Results
- Navigation action should execute first
- Manipulation should start only after navigation completion
- Both actions should complete successfully
- Overall task should finish with "COMPLETED" status

### Verification
- Check dependency sequence in execution state
- Verify action completion order
- Confirm both actions succeed

## Exercise 4: Error Handling and Recovery

### Objective
Test the system's ability to handle and recover from execution errors.

### Steps
- Create a plan that may fail:
   ```bash
   ros2 action send_goal /vla/execute_task vla_msgs/action/ExecuteTask "{plan: {id: 'error_plan', voice_command_id: 'cmd3', actions: [{id: 'nav1', action_type: 'NAVIGATION', parameters: ['navigate_to:impossible_location'], priority: 1, dependencies: [], timeout: {sec: 10, nanosec: 0}, success_criteria: ['navigation_completed']}], status: 'READY', estimated_duration: {sec: 15, nanosec: 0}, required_capabilities: ['navigation']}}"
   ```

- Monitor execution state for errors:
   ```bash
   ros2 topic echo /vla/task_execution_state
   ```

- Observe the system's error handling:
   - Check for error detection
   - Verify error reporting
   - Observe any recovery attempts

- Test the system with timeout scenarios

### Expected Results
- System should detect execution failures
- Error should be reported in execution state
- System should handle errors gracefully
- Execution state should reflect failure status

### Verification
- Check error messages in execution state
- Verify system stability after errors
- Confirm proper error reporting

## Exercise 5: Capability Validation

### Objective
Test the system's capability validation before execution.

### Steps
- Test with available capabilities:
   ```bash
   ros2 service call /vla/validate_capability vla_msgs/srv/ValidateCapability "{capability_name: 'navigation'}"
   ```

- Execute a task requiring navigation:
   - Verify navigation capability is checked
   - Confirm task executes only if capability is available

- Simulate unavailable capability:
   - Temporarily disable a capability
   - Attempt to execute a task requiring it
   - Observe the system's response

- Test capability recovery:
   - Restore the capability
   - Attempt execution again
   - Verify system resumes normal operation

### Expected Results
- Available capabilities should validate successfully
- Unavailable capabilities should be detected
- System should prevent execution without required capabilities
- Recovery should work when capabilities are restored

### Verification
- Check capability validation responses
- Verify execution prevention for missing capabilities
- Confirm system recovery after capability restoration

## Exercise 6: Performance Monitoring

### Objective
Monitor and analyze the performance of the task execution system.

### Steps
- Execute a series of tasks with monitoring:
   ```bash
   # In separate terminals:
   ros2 topic echo /vla/task_execution_state --field progress &
   ros2 topic hz /vla/task_execution_state &
   ```

- Measure execution times for different task types:
   - Simple navigation tasks
   - Complex multi-step tasks
   - Manipulation tasks

- Monitor resource usage:
   ```bash
   htop
   ```

- Record performance metrics:
   - Execution time per action
   - Overall task completion time
   - CPU and memory usage
   - Success/failure rates

### Expected Results
- Simple tasks should complete quickly
- Complex tasks should take longer but remain reasonable
- Resource usage should remain stable
- Success rates should be high

### Verification
- Compare performance across different task types
- Identify any performance bottlenecks
- Verify resource usage is acceptable

## Exercise 7: State Management

### Objective
Test the execution state tracking and management system.

### Steps
- Execute a long-running task:
   ```bash
   ros2 action send_goal /vla/execute_task vla_msgs/action/ExecuteTask "{plan: {id: 'long_plan', voice_command_id: 'cmd4', actions: [{id: 'nav1', action_type: 'NAVIGATION', parameters: ['navigate_to:kitchen'], priority: 1, dependencies: [], timeout: {sec: 60, nanosec: 0}, success_criteria: ['navigation_completed']}, {id: 'wait1', action_type: 'WAIT', parameters: ['duration:5s'], priority: 2, dependencies: ['nav1'], timeout: {sec: 10, nanosec: 0}, success_criteria: ['wait_completed']}, {id: 'nav2', action_type: 'NAVIGATION', parameters: ['navigate_to:living_room'], priority: 3, dependencies: ['wait1'], timeout: {sec: 60, nanosec: 0}, success_criteria: ['navigation_completed']}], status: 'READY', estimated_duration: {sec: 150, nanosec: 0}, required_capabilities: ['navigation']}}"
   ```

- Monitor state changes in real-time:
   ```bash
   ros2 topic echo /vla/task_execution_state
   ```

- Observe state transitions:
   - IDLE → EXECUTING
   - EXECUTING → various sub-states
   - EXECUTING → COMPLETED

- Test state consistency across different execution scenarios

### Expected Results
- State should transition correctly
- Progress should update appropriately
- Current step should track actual execution
- Completed steps should be recorded

### Verification
- Check state transition accuracy
- Verify progress tracking
- Confirm step tracking correctness

## Exercise 8: Parallel Execution (Optional)

### Objective
Test the system's ability to execute independent actions in parallel.

### Steps
- Enable parallel execution in configuration:
   ```bash
   ros2 param set /task_execution_manager enable_parallel_execution true
   ```

- Create a plan with independent actions:
   ```bash
   ros2 action send_goal /vla/execute_task vla_msgs/action/ExecuteTask "{plan: {id: 'parallel_plan', voice_command_id: 'cmd5', actions: [{id: 'nav1', action_type: 'NAVIGATION', parameters: ['navigate_to:kitchen'], priority: 1, dependencies: [], timeout: {sec: 60, nanosec: 0}, success_criteria: ['navigation_completed']}, {id: 'percept1', action_type: 'PERCEPTION', parameters: ['detect_objects'], priority: 1, dependencies: [], timeout: {sec: 30, nanosec: 0}, success_criteria: ['perception_completed']}], status: 'READY', estimated_duration: {sec: 70, nanosec: 0}, required_capabilities: ['navigation', 'perception']}}"
   ```

- Monitor concurrent execution:
   ```bash
   ros2 topic echo /vla/task_execution_state
   ```

- Observe parallel vs sequential execution differences

### Expected Results
- Independent actions should execute concurrently
- Execution time should be reduced for parallelizable tasks
- System should manage resource allocation
- State tracking should handle concurrent updates

### Verification
- Compare execution times with/without parallelization
- Verify concurrent action tracking
- Check resource management

## Exercise 9: Integration with Isaac Sim

### Objective
Test the complete integration with Isaac Sim humanoid robot.

### Steps
- Launch Isaac Sim with humanoid robot model

- Start the VLA system:
   ```bash
   ros2 launch vla_module vla_system.launch.py
   ```

- Execute navigation tasks and observe in simulation:
   - Robot should navigate to specified locations
   - Movement should be smooth and controlled
   - Obstacle avoidance should work if applicable

- Execute manipulation tasks:
   - Robot should attempt to manipulate objects
   - Gripper control should function properly
   - Object interaction should be appropriate

- Monitor simulation and execution state simultaneously

### Expected Results
- Robot should respond to navigation commands in simulation
- Manipulation should work with virtual objects
- Simulation should reflect execution state
- No collisions or errors in simulation

### Verification
- Check robot movement accuracy
- Verify manipulation success rates
- Confirm simulation-ROS integration

## Exercise 10: Comprehensive Task Execution

### Objective
Execute a complete, realistic task combining all VLA capabilities.

### Steps
- Create a realistic scenario:
   - "Go to the kitchen, find the red cup, pick it up, and bring it to the dining table"

- Plan the task with appropriate actions:
   ```bash
   ros2 action send_goal /vla/execute_task vla_msgs/action/ExecuteTask "{plan: {id: 'realistic_plan', voice_command_id: 'cmd6', actions: [{id: 'nav_kitchen', action_type: 'NAVIGATION', parameters: ['navigate_to:kitchen'], priority: 1, dependencies: [], timeout: {sec: 90, nanosec: 0}, success_criteria: ['navigation_completed']}, {id: 'find_cup', action_type: 'PERCEPTION', parameters: ['detect_object:red_cup'], priority: 2, dependencies: ['nav_kitchen'], timeout: {sec: 45, nanosec: 0}, success_criteria: ['object_detected']}, {id: 'pick_cup', action_type: 'MANIPULATION', parameters: ['grasp_object:red_cup'], priority: 3, dependencies: ['find_cup'], timeout: {sec: 60, nanosec: 0}, success_criteria: ['grasp_completed']}, {id: 'nav_table', action_type: 'NAVIGATION', parameters: ['navigate_to:dining_table'], priority: 4, dependencies: ['pick_cup'], timeout: {sec: 90, nanosec: 0}, success_criteria: ['navigation_completed']}, {id: 'place_cup', action_type: 'MANIPULATION', parameters: ['place_object:dining_table'], priority: 5, dependencies: ['nav_table'], timeout: {sec: 60, nanosec: 0}, success_criteria: ['placement_completed']}], status: 'READY', estimated_duration: {sec: 400, nanosec: 0}, required_capabilities: ['navigation', 'perception', 'manipulation']}}"
   ```

- Execute and monitor the complete task:
   ```bash
   ros2 topic echo /vla/task_execution_state
   ```

- Observe the complete task execution in simulation

### Expected Results
- All task steps should execute in correct order
- Robot should navigate, perceive, and manipulate
- Task should complete successfully
- Execution state should accurately reflect progress

### Verification
- Check end-to-end task completion
- Verify all action types function correctly
- Confirm realistic scenario execution

## Assessment Criteria

### Success Metrics
- **Task Completion Rate**: >80% of tasks complete successfully
- **Execution Accuracy**: >90% of actions perform as expected
- **Response Time**: <2 seconds for task initiation
- **Error Handling**: Graceful handling of failures
- **State Tracking**: Accurate progress monitoring

### Performance Benchmarks
- **Simple Tasks**: <30 seconds average completion
- **Complex Tasks**: <3 minutes average completion
- **Resource Usage**: <20% CPU during execution
- **Memory Usage**: Stable during extended operation

### Deliverables
For each exercise, document:
1. Steps performed
2. Results observed
3. Issues encountered
4. Parameters adjusted
5. Performance metrics
6. Verification steps

## Advanced Challenges

### Challenge 1: Dynamic Environment Adaptation
- Execute tasks in environments with moving obstacles
- Adapt navigation plans dynamically
- Handle changing object positions

### Challenge 2: Multi-Task Execution
- Execute multiple tasks concurrently
- Manage resource conflicts
- Prioritize tasks appropriately

### Challenge 3: Learning and Adaptation
- Analyze failed executions
- Adapt execution strategies
- Improve success rates over time

## Troubleshooting Guide

### Common Issues

#### Execution Not Starting
- **Problem**: Task execution doesn't begin
- **Check**: Task execution manager status
- **Verify**: Plan validity and required capabilities

#### Actions Failing
- **Problem**: Individual actions not completing
- **Check**: Action server availability
- **Verify**: Action parameters and robot capabilities

#### State Tracking Issues
- **Problem**: Execution state not updating correctly
- **Check**: State publisher functionality
- **Verify**: Message publishing rates and connections