cmake_minimum_required(VERSION 3.16)
project(emd_msgs)

# This package defines ROS interfaces.  When the necessary ROS tooling is not
# available (such as in the execution environment for these exercises) we skip
# building the interfaces so that the rest of the workspace can still be
# configured.
find_package(ament_cmake QUIET)
if(NOT ament_cmake_FOUND)
  message(WARNING "ament_cmake not found. Skipping build of emd_msgs")
  return()
endif()

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraspMethod.msg"
  "msg/GraspTarget.msg"
  "msg/GraspTask.msg"
  "msg/Option.msg"
  "msg/OptionArray.msg"
  "srv/GraspRequest.srv"
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  shape_msgs
  visualization_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
