# Path Planning Guide for Bipedal Humanoid Navigation

This guide covers implementing path planning algorithms specifically tailored for bipedal humanoid movement using Nav2.

## Overview

Path planning for bipedal humanoid robots requires special considerations due to the unique constraints of two-legged locomotion. Unlike wheeled robots, humanoid robots must maintain balance and follow specific footstep patterns to navigate safely.

## Bipedal Path Planning Constraints

Humanoid robots have specific constraints that affect path planning:

- **Balance Requirements**: Paths must allow for stable bipedal locomotion
- **Step Constraints**: Limited step length, width, and height
- **Turning Radius**: Minimum radius for safe turning
- **Obstacle Clearance**: Ability to step over or around obstacles
- **Support Polygon**: Maintaining center of mass within support area

## Nav2 Configuration for Bipedal Navigation

The Nav2 configuration has been customized for humanoid navigation:

### Planner Server Configuration

The planner server is configured with bipedal-specific parameters:

```yaml
GridBased:
  plugin: "nav2_navfn_planner::NavfnPlanner"
  tolerance: 0.5
  use_astar: false
  allow_unknown: true
  # Bipedal-specific parameters
  humanoid_costmap:
    plugin: "nav2_navfn_planner::HumanoidCostmap"
    # Parameters to account for bipedal locomotion constraints
    step_height: 0.2
    foot_size: 0.15
    max_step_width: 0.3
    min_turn_radius: 0.4
    max_step_length: 0.6
    max_step_height: 0.2
    max_step_down: 0.15
```

### Controller Server Configuration

The controller server is configured for stable bipedal movement:

```yaml
FollowPath:
  plugin: "nav2_rotation_shim_controller::RotationShimController"
  rotation_shim:
    plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    desired_linear_vel: 0.2  # Slower for stability
    lookahead_dist: 0.4
    max_angular_accel: 3.2
    goal_dist_tol: 0.25
```

## Footstep Planning Integration

The path planning system integrates with the footstep planner:

- **Path Smoothing**: Adapt paths for safe footstep placement
- **Step Generation**: Convert smooth paths to discrete footsteps
- **Balance Checking**: Verify each step maintains stability
- **Gait Pattern**: Follow coordinated leg movement patterns

## Implementing Bipedal Path Planning

### 1. Configure Costmaps

Configure costmaps to account for humanoid dimensions and constraints:

```yaml
# Costmap configuration for bipedal navigation
robot_radius: 0.3  # Humanoid-specific radius
inflation_radius: 0.55  # Safety margin for bipedal locomotion
foot_size: 0.15  # Size of humanoid foot
```

### 2. Set Planning Parameters

Configure planning parameters for humanoid constraints:

- `step_length`: Maximum distance between consecutive steps
- `step_width`: Lateral distance between feet
- `step_height`: Maximum height for step-up capability
- `min_turn_radius`: Minimum turning radius for stability

### 3. Launch Humanoid Navigation

To launch the humanoid navigation system:

```bash
# Launch Nav2 with humanoid-specific configuration
ros2 launch nav2 humanoid_navigation.launch.py
```

## Testing Path Planning

Verify path planning is working with bipedal constraints:

```bash
# Send a navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}, header: {frame_id: 'map'}}}"

# Monitor the planned path
ros2 topic echo /plan

# Check footstep planning
ros2 topic echo /footstep_path
```

## Advanced Path Planning Features

### Semantic Path Planning

Use semantic information from perception for intelligent path planning:

- Prefer paths through traversable terrain
- Avoid areas marked as unstable by perception
- Consider semantic classes for path optimization

### Dynamic Replanning

Handle dynamic obstacles with bipedal constraints:

- Detect moving obstacles using perception
- Reproject path considering bipedal constraints
- Maintain stability during replanning

## Troubleshooting

Common path planning issues:

- **Unstable paths**: Check step constraints and balance parameters
- **Infeasible plans**: Verify costmap inflation and robot dimensions
- **Oscillation**: Adjust controller parameters for stable movement
- **Performance issues**: Optimize planning frequency and parameters

## Integration with Perception

Path planning integrates with perception data:

- **Obstacle Avoidance**: Use perception data for dynamic obstacle avoidance
- **Terrain Analysis**: Use segmentation data for traversability
- **Localization**: Use VSLAM data for accurate path following

## Next Steps

After implementing path planning, continue with the [Bipedal Navigation Guide](./bipedal-navigation.md) to implement bipedal-specific navigation behaviors.