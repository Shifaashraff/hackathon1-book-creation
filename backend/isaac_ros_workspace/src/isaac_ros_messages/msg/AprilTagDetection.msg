# Isaac ROS AprilTag Detection message
# Contains AprilTag detection results for VSLAM

std_msgs/Header header

# Tag ID
int32 id

# Tag size
float32 size  # Size of the tag in meters

# Pose information
geometry_msgs/PoseWithCovarianceStamped pose
geometry_msgs/TransformStamped transform

# Corner points in image coordinates
geometry_msgs/Point[] corner_points_image
geometry_msgs/Point[] corner_points_world

# Detection confidence
float32 confidence

# Homography matrix
float64[9] homography_matrix

# Tag family
string tag_family

# Bounding box in image
float32 x_min
float32 y_min
float32 x_max
float32 y_max

# Processing time
float32 processing_time_ms