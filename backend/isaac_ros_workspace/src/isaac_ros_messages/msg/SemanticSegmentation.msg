# Isaac ROS Semantic Segmentation message
# Contains semantic segmentation results

std_msgs/Header header

# Segmentation image
sensor_msgs/Image segmentation_image

# Color map for visualization
sensor_msgs/Image color_map

# Class information
int32[] class_ids
string[] class_names
int32 num_classes

# Pixel counts for each class
int32[] class_pixel_counts

# Confidence map
sensor_msgs/Image confidence_map

# Instance segmentation (optional)
sensor_msgs/Image instance_image
bool has_instances

# Processing information
float32 processing_time_ms
string algorithm_name
string model_name