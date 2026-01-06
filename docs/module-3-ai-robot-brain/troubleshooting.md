# Troubleshooting Guide for Isaac AI Robot Brain

This guide covers common issues and their solutions for the Isaac AI Robot Brain curriculum, including Isaac Sim, Isaac ROS, and Nav2 components.

## Isaac Sim Troubleshooting

### Rendering Issues

#### Problem: No rendering output or black screen
**Symptoms**: Isaac Sim window is black or shows no content
**Solutions**:
1. Check GPU compatibility: Ensure you have an NVIDIA GPU with RTX or GTX series
2. Update GPU drivers to the latest version
3. Verify CUDA compatibility: Isaac Sim requires CUDA 11.0 or higher
4. Check Isaac Sim logs for specific error messages
5. Try running Isaac Sim with lower graphics settings

#### Problem: Slow rendering performance
**Symptoms**: Low frame rates or laggy interaction
**Solutions**:
1. Reduce rendering quality settings in Isaac Sim
2. Close other GPU-intensive applications
3. Increase GPU memory allocation if possible
4. Check if Isaac Sim is using dedicated GPU (not integrated graphics)
5. Reduce scene complexity temporarily for testing

### Physics Simulation Issues

#### Problem: Robot falls through the ground or exhibits unstable physics
**Symptoms**: Robot sinks into ground, unrealistic movements, explosions in simulation
**Solutions**:
1. Verify physics parameters in `isaac_sim/configs/sim_config.yaml`
2. Check that gravity is set to `-9.81` m/s²
3. Adjust solver parameters: increase iterations or change solver type
4. Verify that robot has proper collision geometry in URDF
5. Check that robot is positioned above the ground initially

#### Problem: Robot joints behave erratically
**Symptoms**: Joint positions jump unexpectedly, oscillations, unstable movements
**Solutions**:
1. Check joint limits in robot URDF
2. Verify PID controller parameters in `isaac_sim/configs/robot_configs/humanoid_control.yaml`
3. Adjust joint damping and friction parameters
4. Check that control frequency matches simulation frequency
5. Verify that joint names in controller match those in URDF

### Sensor Simulation Issues

#### Problem: No sensor data or empty messages
**Symptoms**: Sensor topics show no data or messages with zero values
**Solutions**:
1. Verify sensor plugins are correctly configured in USD files
2. Check that sensor topics are being published: `ros2 topic list`
3. Verify sensor frame IDs are correct
4. Check that Isaac Sim and ROS times are synchronized
5. Look for sensor plugin loading errors in Isaac Sim logs

#### Problem: Incorrect sensor readings
**Symptoms**: Unexpected values, wrong units, incorrect coordinate frames
**Solutions**:
1. Verify sensor calibration parameters
2. Check coordinate frame transformations (ROS uses right-handed, Isaac Sim uses left-handed)
3. Verify sensor mounting positions in robot URDF
4. Check that sensor noise parameters are reasonable
5. Validate sensor message formats and units

## Isaac ROS Troubleshooting

### Perception Pipeline Issues

#### Problem: Perception pipeline node not starting
**Symptoms**: Node fails to launch or crashes immediately
**Solutions**:
1. Check dependencies: Ensure all required Isaac ROS packages are installed
2. Verify Python environment: Check Python version and installed packages
3. Check ROS 2 environment: Ensure workspace is sourced
4. Look for import errors in the launch logs
5. Verify all required configuration files exist

#### Problem: No perception output or poor detection quality
**Symptoms**: Detection topics show no results or low-confidence detections
**Solutions**:
1. Verify camera topics are publishing data: `ros2 topic echo /camera/image_raw`
2. Check camera calibration parameters
3. Verify lighting conditions in the simulation
4. Check model paths for perception networks
5. Adjust confidence thresholds in perception configuration

### VSLAM Issues

#### Problem: VSLAM fails to initialize or track
**Symptoms**: No pose estimates published, tracking lost frequently
**Solutions**:
1. Verify stereo camera calibration
2. Check that both left and right camera topics are publishing
3. Ensure sufficient visual features in the environment
4. Verify that cameras have overlapping fields of view
5. Check VSLAM parameter configuration

#### Problem: Drifting pose estimates
**Symptoms**: Position estimates drift significantly over time
**Solutions**:
1. Check for camera calibration errors
2. Verify camera synchronization
3. Adjust VSLAM parameters for more conservative tracking
4. Check for consistent lighting conditions
5. Verify that robot is moving sufficiently for feature tracking

### Communication Issues

#### Problem: Topics not connecting between nodes
**Symptoms**: Publishers and subscribers show no data exchange
**Solutions**:
1. Check topic names: Ensure publisher and subscriber use identical names
2. Verify message types: Ensure publisher and subscriber use same message types
3. Check QoS settings: Ensure compatible QoS policies
4. Verify network connectivity if using multi-machine setup
5. Use `ros2 topic info` to verify connections

## Nav2 Troubleshooting

### Navigation Issues

#### Problem: Navigation fails to start or plan paths
**Symptoms**: Navigation action fails, no path planning, errors in nav2 logs
**Solutions**:
1. Verify Nav2 configuration files (`nav2/config/humanoid_nav2_params.yaml`)
2. Check that all required Nav2 nodes are running
3. Verify costmap parameters for humanoid dimensions
4. Check TF tree: Ensure all required transforms exist
5. Verify map server is running if using static map

#### Problem: Robot gets stuck or fails to navigate
**Symptoms**: Robot stops mid-navigation, oscillates, or fails to reach goal
**Solutions**:
1. Check costmap inflation parameters for humanoid size
2. Verify local planner parameters for stability
3. Check that robot can fit through planned path
4. Verify obstacle detection is working properly
5. Adjust controller parameters for humanoid stability

### Bipedal-Specific Issues

#### Problem: Unstable bipedal navigation
**Symptoms**: Robot loses balance during navigation, falls over
**Solutions**:
1. Check footstep planner parameters in `nav2/scripts/footstep_planner.py`
2. Verify step constraints (length, width, height) are appropriate
3. Adjust controller parameters for more stable movement
4. Check balance control parameters in humanoid configuration
5. Verify that robot model has proper center of mass

#### Problem: Poor obstacle avoidance for bipedal robot
**Symptoms**: Robot collides with obstacles or takes inefficient paths
**Solutions**:
1. Adjust costmap inflation for humanoid foot size
2. Verify perception integration is working
3. Check that obstacle detection is accurate
4. Adjust path planning parameters for bipedal constraints
5. Verify that obstacle clearance is sufficient for foot placement

## Integration Issues

### Isaac Sim and ROS Connection Issues

#### Problem: Isaac Sim bridge not connecting
**Symptoms**: No communication between Isaac Sim and ROS nodes
**Solutions**:
1. Check Isaac ROS bridge configuration
2. Verify Isaac Sim extension is properly loaded
3. Check that Isaac Sim is using correct ROS domain
4. Verify network settings if running on different machines
5. Look for bridge connection errors in logs

#### Problem: TF synchronization issues
**Symptoms**: TF lookup failures, timing issues between frames
**Solutions**:
1. Verify `use_sim_time` is set consistently across nodes
2. Check that Isaac Sim is publishing clock messages
3. Verify TF publishing rates are appropriate
4. Check that frame names are consistent between Isaac Sim and ROS
5. Use `tf2_tools` to debug TF issues

## Performance Issues

### Resource Usage Problems

#### Problem: High CPU or GPU usage
**Symptoms**: System slowdown, thermal throttling, dropped frames
**Solutions**:
1. Reduce simulation frequency
2. Lower rendering quality settings
3. Decrease sensor update rates
4. Optimize perception pipeline parameters
5. Close unnecessary applications

#### Problem: Memory leaks or growing memory usage
**Symptoms**: Gradual decrease in performance, out of memory errors
**Solutions**:
1. Check for buffer accumulation in perception nodes
2. Verify proper cleanup of ROS message queues
3. Monitor memory usage with system tools
4. Restart nodes periodically if needed
5. Check for circular message dependencies

## Debugging Strategies

### Log Analysis
1. Check Isaac Sim logs in the Isaac Sim output window
2. Monitor ROS 2 logs: `ros2 launch <package> <launch_file> --console-screen`
3. Use `rqt_console` for centralized logging
4. Check system logs for driver or hardware issues

### Topic Monitoring
1. Use `ros2 topic list` to verify all expected topics exist
2. Use `ros2 topic echo` to inspect message content
3. Use `rqt_plot` to visualize numerical data
4. Use `rqt_image_view` to view camera feeds

### TF Debugging
1. Use `tf2_echo` to verify transforms
2. Use `view_frames` to visualize TF tree
3. Check transform timing and availability
4. Verify frame naming conventions

### Simulation State
1. Use Isaac Sim debugging tools
2. Check physics properties and collision meshes
3. Verify initial conditions and robot state
4. Test with simplified scenarios first

## Common Error Messages

### Isaac Sim Specific
- `"Failed to initialize renderer"`: Check GPU compatibility and drivers
- `"Plugin failed to load"`: Check plugin paths and dependencies
- `"Physics solver instability"`: Adjust physics parameters

### ROS 2 Specific
- `"Could not find a connection"`: Check topic names and types
- `"Transform timeout"`: Check TF publishing and timing
- `"Parameter not declared"`: Verify parameter declarations

### Nav2 Specific
- `"Failed to create plan"`: Check costmap and planner configuration
- `"Controller failed to follow path"`: Adjust controller parameters
- `"Local planner is not sufficiently close to the global plan"`: Check tolerances

## Getting Help

If you encounter issues not covered in this guide:

1. Check the Isaac ROS documentation and release notes
2. Search the NVIDIA Isaac forums and GitHub issues
3. Use the ROS 2 community forums and answers.ros.org
4. Verify your setup against the quickstart guide
5. Test with simpler scenarios to isolate the issue