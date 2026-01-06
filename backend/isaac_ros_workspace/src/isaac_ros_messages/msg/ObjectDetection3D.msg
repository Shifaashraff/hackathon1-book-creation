# Isaac ROS 3D Object Detection message
# Contains 3D object detection results

std_msgs/Header header

# Object ID
int32 id

# Class information
string class_name
int32 class_id

# 3D bounding box
geometry_msgs/Point center
geometry_msgs/Vector3 size
geometry_msgs/Quaternion orientation

# 2D bounding box projection (for reference)
float32 x_min
float32 y_min
float32 x_max
float32 y_max

# Pose information
geometry_msgs/Pose pose
geometry_msgs/PoseWithCovariance pose_with_covariance

# Confidence score
float32 confidence

# Object attributes
float32 velocity_x
float32 velocity_y
float32 velocity_z
float32 acceleration_x
float32 acceleration_y
float32 acceleration_z
string[] attributes

# Tracking information
int32 tracking_id
string tracking_status
float32 tracking_confidence