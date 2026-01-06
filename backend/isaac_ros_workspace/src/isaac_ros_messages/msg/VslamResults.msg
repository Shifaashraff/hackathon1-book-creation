# Isaac ROS Visual SLAM Results message
# Contains VSLAM results including pose and map information

std_msgs/Header header

# Current pose estimate
geometry_msgs/PoseWithCovarianceStamped pose_estimate

# Odometry data
nav_msgs/Odometry odometry

# Map information
nav_msgs/OccupancyGrid local_map
nav_msgs/OccupancyGrid global_map

# Keyframe information
geometry_msgs/PoseArray keyframes
sensor_msgs/Image[] keyframe_images

# Feature information
sensor_msgs/PointCloud2 features
int32[] feature_ids

# Tracking status
float32 tracking_confidence
string tracking_status
bool is_localized
bool is_mapping

# Performance metrics
float32 processing_time_ms
int32 feature_count
int32 keyframe_count

# Loop closure information
bool has_loop_closure
geometry_msgs/Pose loop_closure_pose
float32 loop_closure_confidence