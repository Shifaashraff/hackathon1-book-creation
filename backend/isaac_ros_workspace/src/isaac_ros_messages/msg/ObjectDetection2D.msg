# Isaac ROS 2D Object Detection message
# Contains 2D object detection results

std_msgs/Header header

# Object ID
int32 id

# Class information
string class_name
int32 class_id

# Bounding box (normalized coordinates)
float32 x_min  # Normalized x coordinate of top-left corner
float32 y_min  # Normalized y coordinate of top-left corner
float32 x_max  # Normalized x coordinate of bottom-right corner
float32 y_max  # Normalized y coordinate of bottom-right corner

# Bounding box (pixel coordinates)
int32 pixel_x_min  # Pixel x coordinate of top-left corner
int32 pixel_y_min  # Pixel y coordinate of top-left corner
int32 pixel_x_max  # Pixel x coordinate of bottom-right corner
int32 pixel_y_max  # Pixel y coordinate of bottom-right corner

# Confidence score
float32 confidence

# Object attributes
float32[] attributes  # Additional attributes like pose, size, etc.
string[] tags         # Additional tags for the detection

# Mask (for instance segmentation)
sensor_msgs/Image mask
bool has_mask