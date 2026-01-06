# Bipedal Navigation Guide for Humanoid Robots

This guide covers implementing navigation behaviors specifically tailored for bipedal humanoid movement using Nav2.

## Overview

Bipedal navigation requires special handling to maintain balance and stability while moving on two legs. Unlike wheeled robots, humanoid robots must coordinate leg movements and maintain their center of mass within the support polygon defined by their feet.

## Key Concepts in Bipedal Navigation

### Support Polygon
The area defined by the contact points of the feet that must contain the robot's center of mass for stability.

### Zero Moment Point (ZMP)
A point where the net moment of the ground reaction forces is zero, critical for maintaining balance.

### Gait Patterns
The coordinated movement patterns of the legs during locomotion (e.g., walking, running, stepping).

## Bipedal Navigation Parameters

### Robot Dimensions
- `robot_radius`: 0.3m - Defines the humanoid's space requirements
- `foot_size`: 0.15m - Foot dimensions for step planning
- `step_width`: 0.2m - Lateral distance between feet for stability

### Step Constraints
- `step_length`: 0.3m - Maximum forward step distance
- `step_height`: 0.2m - Maximum step-up height
- `max_step_down`: 0.15m - Maximum step-down height

### Balance Parameters
- `com_height`: 0.8m - Center of mass height
- `zmp_margin`: 0.05m - Safety margin for ZMP
- `ankle_roll_gain`: 0.2 - Ankle adjustment for lateral balance

## Implementing Bipedal Navigation

### 1. Configure Controller for Bipedal Stability

The controller configuration ensures stable bipedal movement:

```yaml
# Controller configuration for bipedal stability
FollowPath:
  plugin: "nav2_rotation_shim_controller::RotationShimController"
  rotation_shim:
    plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    desired_linear_vel: 0.2  # Slower for stability
    lookahead_dist: 0.4      # Appropriate for step-by-step movement
    max_angular_accel: 3.2   # Controlled turning for balance
    goal_dist_tol: 0.25      # Appropriate tolerance for bipedal precision
```

### 2. Set Up Costmaps for Bipedal Navigation

Configure costmaps to account for bipedal-specific requirements:

```yaml
# Costmap configuration for bipedal navigation
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.3  # Humanoid-specific radius
      resolution: 0.05   # Fine resolution for precise footstep placement
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        cost_scaling_factor: 3.0
        inflation_radius: 0.55  # Safety margin for bipedal locomotion
```

### 3. Implement Footstep Planning

The footstep planner generates safe and stable footsteps:

```python
# Example footstep planning parameters
gait_parameters = {
    'step_height': 0.05,      # clearance during swing phase
    'step_duration': 0.8,     # seconds per step
    'stance_duration': 0.4,   # time in stance phase
    'swing_duration': 0.4,    # time in swing phase
    'double_support_ratio': 0.2  # ratio of double support phase
}

stability_parameters = {
    'zmp_margin': 0.05,       # ZMP safety margin
    'com_height': 0.8,        # Center of mass height
    'com_tolerance': 0.05,    # COM position tolerance
    'ankle_roll_gain': 0.2,   # Ankle roll for lateral balance
    'hip_sway_gain': 0.1      # Hip sway for balance
}
```

## Navigation Behaviors for Bipedal Robots

### Stable Walking Pattern
- Maintain consistent step length and timing
- Coordinate arm movements for balance
- Adjust step width based on turning requirements

### Turning Behavior
- Use appropriate turning radius based on stability
- Coordinate foot placement for smooth turns
- Adjust step timing during turns

### Obstacle Negotiation
- Plan safe stepping over or around obstacles
- Maintain balance during obstacle negotiation
- Use perception data to identify safe stepping points

## Testing Bipedal Navigation

Verify bipedal navigation is working properly:

```bash
# Launch the humanoid navigation system
ros2 launch nav2 humanoid_navigation.launch.py

# Send a navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}, header: {frame_id: 'map'}}}"

# Monitor footstep planning
ros2 topic echo /footstep_path

# Check navigation progress
ros2 topic echo /humanoid/plan

# Monitor velocity commands
ros2 topic echo /cmd_vel
```

## Integration with Perception

Bipedal navigation integrates with perception data:

- **Obstacle Detection**: Use perception to identify and avoid obstacles
- **Terrain Analysis**: Use segmentation to identify safe walking surfaces
- **Localization**: Use VSLAM for accurate position tracking

## Advanced Bipedal Navigation Features

### Dynamic Balance Adjustment
- Adjust gait parameters based on terrain
- Modify step patterns for stability on uneven surfaces
- Coordinate with perception for dynamic balance

### Multi-step Planning
- Plan multiple footsteps ahead for complex maneuvers
- Consider balance constraints for extended movements
- Optimize footstep sequences for efficiency

## Troubleshooting

Common bipedal navigation issues:

- **Balance Loss**: Check COM height and ZMP parameters
- **Unstable Turning**: Adjust turning radius and step timing
- **Step Collision**: Verify foot dimensions and inflation parameters
- **Oscillation**: Tune controller gains for stable movement

## Performance Optimization

Optimize bipedal navigation performance:

- **Step Timing**: Adjust step duration for smooth motion
- **Path Smoothing**: Balance path quality with computational cost
- **Balance Control**: Optimize balance parameters for stability
- **Perception Integration**: Optimize fusion of perception and navigation

## Next Steps

After implementing bipedal navigation, continue with the [Obstacle Avoidance Guide](./obstacle-avoidance.md) to enhance navigation with dynamic obstacle handling.