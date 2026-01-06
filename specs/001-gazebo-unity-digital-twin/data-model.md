# Data Model: Digital Twin Curriculum - Gazebo & Unity Integration

**Feature**: Digital Twin Curriculum - Gazebo & Unity Integration
**Date**: 2026-01-02
**Status**: Design

## Entity: Robot Model

**Description**: Digital representation of a humanoid robot including geometry, joints, and physical properties
**Fields**:
- `id`: Unique identifier for the robot model
- `name`: Human-readable name of the robot
- `urdf_path`: Path to the URDF file defining the robot
- `sdf_path`: Path to the SDF file for Gazebo simulation
- `mesh_paths`: Array of paths to 3D mesh files
- `joint_definitions`: Array of joint configurations (type, limits, dynamics)
- `link_definitions`: Array of link configurations (inertial, visual, collision)
- `physical_properties`: Mass, center of mass, moments of inertia
- `scale_factor`: Scale to apply when importing to Unity

**Relationships**:
- Contains multiple `Joint` entities
- Contains multiple `Link` entities
- Associated with multiple `Sensor` entities

**Validation rules**:
- All paths must exist and be accessible
- Joint limits must be within physically realistic ranges
- Mass must be positive
- URDF/SDF files must be valid XML/SDF

## Entity: Sensor Data

**Description**: Information collected from simulated sensors (LiDAR scans, camera images, IMU readings) published via ROS 2
**Fields**:
- `id`: Unique identifier for the sensor data instance
- `sensor_type`: Type of sensor (LiDAR, Camera, IMU)
- `topic_name`: ROS 2 topic to which data is published
- `data_format`: Format of the sensor data (e.g., sensor_msgs/LaserScan)
- `frame_id`: Coordinate frame of the sensor
- `timestamp`: Time when data was captured
- `data_payload`: The actual sensor data (varies by sensor type)
- `robot_id`: Reference to the robot containing this sensor

**Relationships**:
- Belongs to one `Robot Model` entity
- Associated with one `Simulation Environment` entity

**Validation rules**:
- Topic names must follow ROS 2 conventions
- Data format must match sensor type
- Timestamp must be current or recent
- Frame ID must correspond to a valid TF frame

## Entity: Simulation Environment

**Description**: Virtual world space containing robots, objects, and physics properties
**Fields**:
- `id`: Unique identifier for the environment
- `name`: Human-readable name of the environment
- `world_file`: Path to the Gazebo world file (.sdf)
- `physics_properties`: Gravity, damping, and other physics parameters
- `objects`: Array of static and dynamic objects in the environment
- `lighting_config`: Lighting settings for the environment
- `robot_positions`: Initial positions of robots in the environment

**Relationships**:
- Contains multiple `Robot Model` entities
- Contains multiple `Sensor Data` entities
- Associated with one `Visualization Scene` entity

**Validation rules**:
- World file must be valid SDF
- Physics parameters must be within realistic ranges
- Object positions must not result in initial collisions

## Entity: Visualization Scene

**Description**: Unity representation of the simulation environment for visual rendering
**Fields**:
- `id`: Unique identifier for the visualization scene
- `scene_name`: Name of the Unity scene file
- `robot_models`: Array of robot model references for Unity import
- `environment_assets`: Array of asset paths for environment objects
- `camera_config`: Position and settings for visualization cameras
- `lighting_config`: Lighting settings for Unity rendering
- `synchronization_settings`: Settings for real-time sync with Gazebo

**Relationships**:
- Associated with one `Simulation Environment` entity
- Contains multiple `Robot Model` entities (for visualization)

**Validation rules**:
- Scene file must be a valid Unity scene
- Asset paths must be accessible to Unity
- Synchronization settings must be valid ROS 2 parameters

## Entity: Curriculum Chapter

**Description**: A section of the curriculum covering a specific aspect of digital twin simulation
**Fields**:
- `id`: Unique identifier for the chapter
- `title`: Title of the chapter
- `topic`: Main topic (Physics, Sensors, Rendering)
- `content`: Markdown content for the chapter
- `objectives`: Learning objectives for the chapter
- `prerequisites`: Knowledge or setup required before starting
- `exercises`: List of hands-on exercises
- `assessments`: Evaluation criteria and methods

**Relationships**:
- Contains multiple `Exercise` entities
- Associated with multiple `Robot Model` entities (for examples)

**Validation rules**:
- Content must be in valid Markdown format
- Objectives must be measurable
- Prerequisites must be clearly defined
- Exercises must have clear instructions and expected outcomes

## Entity: Exercise

**Description**: A hands-on activity within a curriculum chapter
**Fields**:
- `id`: Unique identifier for the exercise
- `chapter_id`: Reference to the parent chapter
- `title`: Title of the exercise
- `instructions`: Step-by-step instructions for the exercise
- `expected_outcome`: Description of what students should achieve
- `resources`: List of required resources or files
- `duration`: Estimated time to complete the exercise
- `difficulty`: Level of difficulty (beginner, intermediate, advanced)

**Relationships**:
- Belongs to one `Curriculum Chapter` entity
- Associated with one or more `Robot Model` entities
- Associated with one or more `Simulation Environment` entities

**Validation rules**:
- Instructions must be clear and unambiguous
- Expected outcomes must be verifiable
- Duration must be realistic
- Difficulty must match chapter level

## State Transitions

### Robot Model State Transitions
- `created` → `validated`: When model files are verified
- `validated` → `loaded`: When model is loaded into simulation
- `loaded` → `simulated`: When physics simulation begins
- `simulated` → `visualized`: When Unity visualization starts

### Curriculum Chapter State Transitions
- `planned` → `drafted`: When initial content is written
- `drafted` → `reviewed`: When content is reviewed for accuracy
- `reviewed` → `published`: When content is ready for students
- `published` → `assessed`: When student outcomes are evaluated