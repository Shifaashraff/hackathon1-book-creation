# VSLAM Setup Guide for Isaac ROS

This guide covers setting up Visual SLAM (VSLAM) systems using Isaac ROS for humanoid robot perception and navigation.

## Overview

Visual SLAM (Simultaneous Localization and Mapping) enables robots to create maps of their environment while simultaneously determining their position within that map. Isaac ROS provides optimized VSLAM algorithms that leverage NVIDIA hardware acceleration.

## Prerequisites

- Completed Isaac Sim setup with humanoid robot
- Isaac ROS workspace configured
- Stereo camera sensors configured on the robot
- CUDA-compatible GPU installed

## Isaac ROS VSLAM Components

Isaac ROS VSLAM consists of several key components:

- **Stereo Image Processing**: Rectification and preprocessing of stereo camera images
- **Feature Detection**: Extraction of visual features from images
- **Feature Matching**: Matching features between frames and stereo pairs
- **Pose Estimation**: Estimating camera pose from matched features
- **Map Building**: Creating and maintaining 3D maps of the environment
- **Loop Closure**: Detecting and correcting for loop closures

## Setting Up Stereo Cameras

For VSLAM to function properly, your humanoid robot needs stereo cameras configured:

```xml
<!-- Stereo camera configuration in URDF -->
<link name="stereo_camera_base_link">
  <visual>
    <geometry>
      <box size="0.06 0.1 0.05"/>
    </geometry>
  </visual>
</link>

<joint name="stereo_camera_base_joint" type="fixed">
  <parent link="head"/>
  <child link="stereo_camera_base_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>

<!-- Left camera of stereo pair -->
<link name="left_camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.02"/>
    </geometry>
  </visual>
</link>

<joint name="left_camera_joint" type="fixed">
  <parent link="stereo_camera_base_link"/>
  <child link="left_camera_link"/>
  <origin xyz="0 0.05 0" rpy="0 0 0"/>
</joint>

<!-- Right camera of stereo pair -->
<link name="right_camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.02"/>
    </geometry>
  </visual>
</link>

<joint name="right_camera_joint" type="fixed">
  <parent link="stereo_camera_base_link"/>
  <child link="right_camera_link"/>
  <origin xyz="0 -0.05 0" rpy="0 0 0"/>
</joint>
```

## Isaac ROS VSLAM Pipeline

The Isaac ROS VSLAM pipeline includes:

1. **Stereo Rectification**: Correcting camera distortions
2. **Feature Extraction**: Using GPU-accelerated feature detectors
3. **Stereo Matching**: Computing disparity maps
4. **Visual Odometry**: Estimating motion between frames
5. **Bundle Adjustment**: Optimizing camera poses and map points
6. **Loop Closure Detection**: Identifying previously visited locations

## Launching VSLAM

To launch the VSLAM pipeline:

```bash
# Launch Isaac ROS VSLAM nodes
ros2 launch isaac_ros_visual_slam visual_slam.launch.py
```

## VSLAM Configuration Parameters

Key parameters for VSLAM configuration:

- `enable_rectification`: Enable stereo image rectification
- `enable_debug_mode`: Enable debug output for visualization
- `rectified_width`: Width of rectified images
- `rectified_height`: Height of rectified images
- `max_num_points`: Maximum number of points in the map
- `max_num_kf`: Maximum number of keyframes to store

## Testing VSLAM

Verify VSLAM is working by monitoring these topics:

```bash
# Check VSLAM pose estimates
ros2 topic echo /visual_slam/poses

# Check feature tracking
ros2 topic echo /visual_slam/tracked_features

# Check map points
ros2 topic echo /visual_slam/map_points
```

## Troubleshooting

Common VSLAM issues and solutions:

- **Low feature count**: Ensure adequate lighting and texture in the environment
- **Drift**: Check camera calibration and verify stereo baseline
- **Poor tracking**: Verify image quality and camera synchronization
- **Performance issues**: Adjust parameters for your hardware capabilities

## Next Steps

After setting up VSLAM, continue with the [Perception Pipelines Guide](./perception-pipelines.md) to implement object detection and segmentation.