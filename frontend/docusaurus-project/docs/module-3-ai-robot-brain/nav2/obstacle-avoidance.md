# Obstacle Avoidance Guide for Bipedal Humanoid Navigation

This guide covers implementing obstacle avoidance specifically tailored for bipedal humanoid movement using Nav2 and Isaac ROS perception integration.

## Overview

Obstacle avoidance for bipedal humanoid robots requires special considerations due to balance constraints and the discrete nature of footstep-based locomotion. Unlike wheeled robots, humanoid robots must plan safe and stable footsteps around obstacles while maintaining balance.

## Types of Obstacles for Humanoid Navigation

### Ground Obstacles
- Small objects that can be stepped over
- Holes or depressions that need to be stepped around
- Uneven terrain that affects foot placement

### Standing Obstacles
- Furniture and structures at human height
- Other robots or humans in the environment
- Low-hanging objects that require path adjustments

### Dynamic Obstacles
- Moving humans or robots
- Objects that change position over time
- Predictable and unpredictable moving obstacles

## Perception-Driven Obstacle Avoidance

### Sensor Integration
The system integrates multiple sensors for comprehensive obstacle detection:

- **Stereo Cameras**: Provide depth information for 3D obstacle mapping
- **LiDAR**: Accurate distance measurements for reliable obstacle detection
- **IMU**: Balance information to adjust navigation based on stability
- **Depth Sensors**: Precise measurements for safe footstep placement

### Semantic Obstacle Classification
Using Isaac ROS perception, obstacles are classified for appropriate avoidance behavior:

- **Traversable**: Small obstacles that can be stepped over
- **Avoidable**: Obstacles requiring path adjustments
- **Dangerous**: Obstacles requiring significant detours
- **Dynamic**: Moving obstacles requiring prediction

## Configuring Obstacle Avoidance for Bipedal Navigation

### Costmap Configuration

Configure costmaps with bipedal-specific parameters:

```yaml
# Local costmap configuration for bipedal obstacle avoidance
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05  # Fine resolution for precise footstep planning
      robot_radius: 0.3  # Humanoid-specific radius
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55  # Safety margin for bipedal locomotion
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0  # Humanoid height consideration
        mark_threshold: 0
        observation_sources: scan perception_obstacles
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
        perception_obstacles:
          topic: /perception/obstacles
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
```

### Dynamic Obstacle Parameters

Configure parameters for handling dynamic obstacles:

```yaml
# Dynamic obstacle handling parameters
controller_server:
  ros__parameters:
    FollowPath:
      rotation_shim:
        plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        desired_linear_vel: 0.2  # Slower for dynamic obstacle response
        max_allowed_time_to_collision_up_to_carrot: 1.0  # Time to collision threshold
```

## Implementing Bipedal Obstacle Avoidance

### 1. Perception Integration

Integrate perception data into the navigation system:

```python
# Example perception integration in nav2_interface_node
def process_perception_fusion(self):
    # Process obstacle detection from perception data
    if self.perception_data is not None:
        obstacles = self.extract_obstacles_from_perception()
        if obstacles:
            # Update navigation costmap with detected obstacles
            self.update_navigation_costmap(obstacles)

def extract_obstacles_from_perception(self):
    obstacles = []
    # Extract obstacles from detections with confidence filtering
    for detection in self.detections:
        if detection.confidence > self.perception_fusion_module['confidence_threshold']:
            obstacle = {
                'position': self.convert_detection_to_world_frame(detection),
                'confidence': detection.confidence,
                'class_name': detection.class_name,
                'bbox': {
                    'x_min': detection.x_min,
                    'y_min': detection.y_min,
                    'x_max': detection.x_max,
                    'y_max': detection.y_max
                }
            }
            obstacles.append(obstacle)
    return obstacles
```

### 2. Footstep-Aware Obstacle Avoidance

Plan obstacle avoidance considering footstep constraints:

```python
# Example footstep-aware obstacle avoidance
def check_footstep_stability(self, foot_pose: Pose, support_foot_pose: Pose) -> bool:
    # Calculate distance between feet
    dx = foot_pose.position.x - support_foot_pose.position.x
    dy = foot_pose.position.y - support_foot_pose.position.y
    distance = math.sqrt(dx*dx + dy*dy)

    # Check if distance is within acceptable range
    min_distance = self.min_step_width
    max_distance = self.step_length * 1.5  # Allow some flexibility

    return min_distance <= distance <= max_distance
```

### 3. Dynamic Replanning

Implement dynamic replanning for moving obstacles:

```yaml
# Behavior tree configuration for dynamic replanning
bt_navigator:
  ros__parameters:
    behavior_tree_xml_filename: "navigate_w_replanning_and_recovery.xml"
    # Parameters for dynamic obstacle response
    goal_check_rate: 5.0
    velocity_check_rate: 10.0
    velocity_threshold: 0.01
```

## Testing Obstacle Avoidance

Verify obstacle avoidance is working properly:

```bash
# Launch navigation with perception integration
ros2 launch nav2 humanoid_navigation.launch.py

# Launch perception pipeline
ros2 launch isaac_ros_examples perception_pipeline.launch.py

# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}, header: {frame_id: 'map'}}}"

# Monitor obstacle detection
ros2 topic echo /perception/obstacles

# Check local costmap with obstacle information
ros2 topic echo /local_costmap/costmap

# Monitor navigation plan adjustments
ros2 topic echo /humanoid/local_plan
```

## Advanced Obstacle Avoidance Features

### Predictive Avoidance
- Predict trajectories of moving obstacles
- Plan paths that account for future obstacle positions
- Adjust navigation speed based on obstacle predictability

### Multi-Level Avoidance
- Ground-level obstacle avoidance for stepping
- Body-level obstacle avoidance for navigation
- Ceiling-level consideration for overhead obstacles

### Social Navigation
- Respect personal space of humans
- Yield to pedestrians appropriately
- Navigate through crowds safely

## Integration with Isaac ROS

### Perception Pipeline Integration
- Use Isaac ROS perception for accurate obstacle detection
- Integrate VSLAM for dynamic obstacle tracking
- Use semantic segmentation for obstacle classification

### Real-time Adaptation
- Adjust navigation parameters based on perception confidence
- Modify obstacle avoidance behavior based on environmental context
- Coordinate with perception for robust obstacle handling

## Troubleshooting

Common obstacle avoidance issues:

- **Conservative Avoidance**: Check costmap inflation parameters
- **Collision with Obstacles**: Verify sensor calibration and fusion
- **Oscillation Near Obstacles**: Adjust controller and costmap parameters
- **Slow Response to Dynamic Obstacles**: Tune prediction and replanning parameters

## Performance Optimization

Optimize obstacle avoidance performance:

- **Sensor Fusion**: Optimize combination of multiple sensor inputs
- **Prediction Accuracy**: Improve trajectory prediction for dynamic obstacles
- **Replanning Efficiency**: Balance replanning frequency with computational cost
- **Path Smoothing**: Maintain obstacle avoidance while smoothing paths

## Safety Considerations

Ensure safe obstacle avoidance:

- **Conservative Parameters**: Use safety margins in planning
- **Emergency Stops**: Implement emergency stopping for unexpected obstacles
- **Validation**: Verify obstacle avoidance paths are safe before execution
- **Recovery Behaviors**: Implement recovery when obstacle avoidance fails

## Next Steps

After implementing obstacle avoidance, continue with the [Nav2 Exercises](./exercises.md) to practice and validate your bipedal navigation and obstacle avoidance skills.