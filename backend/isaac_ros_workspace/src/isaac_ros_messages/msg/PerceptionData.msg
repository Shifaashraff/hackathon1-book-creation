# Isaac ROS Perception Data message
# Contains perception data from various sensors

std_msgs/Header header

# RGB image data
sensor_msgs/Image rgb_image

# Depth image data
sensor_msgs/Image depth_image

# Camera info
sensor_msgs/CameraInfo camera_info

# Object detections (2D bounding boxes)
sensor_msgs/PointCloud2 point_cloud

# Semantic segmentation
sensor_msgs/Image segmentation_image

# Object detection results
sensor_msgs/PointCloud2 detections_3d
geometry_msgs/PoseArray object_poses

# Timestamps
builtin_interfaces/Time capture_time
builtin_interfaces/Time processing_time

# Confidence scores
float32[] detection_confidences

# Status flags
bool has_rgb
bool has_depth
bool has_segmentation
bool has_detections
bool is_valid