cmake_minimum_required(VERSION 3.8)
project(vla_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define message, service and action files
set(msg_files
  "msg/VoiceCommand.msg"
  "msg/CognitivePlan.msg"
  "msg/ActionStep.msg"
  "msg/TaskExecutionState.msg"
)

set(srv_files
  "srv/ValidateCapability.srv"
)

set(action_files
  "action/ExecuteTask.action"
  "action/PlanActions.action"
  "action/ExecuteActionStep.action"
)

# Generate ROS IDL interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs action_msgs sensor_msgs
  ADD_LINTERFACES
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and cpplint is enabled
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()