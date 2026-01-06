cmake_minimum_required(VERSION 3.8)
project(isaac_ros_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Define message files
set(msg_files
  "msg/PerceptionData.msg"
  "msg/VslamResults.msg"
  "msg/ObjectDetection2D.msg"
  "msg/ObjectDetection3D.msg"
  "msg/SemanticSegmentation.msg"
  "msg/AprilTagDetection.msg"
)

# Define service files
set(srv_files
  "srv/GetPerceptionData.srv"
  "srv/StartPerception.srv"
  "srv/StopPerception.srv"
)

# Define action files
set(action_files
  "action/Perception.action"
  "action/Vslam.action"
)

# Generate ROS messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs builtin_interfaces
  ADD_LINTER_TESTS
)

ament_package()