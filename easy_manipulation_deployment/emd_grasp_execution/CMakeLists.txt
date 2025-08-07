cmake_minimum_required(VERSION 3.5)
project(emd_grasp_execution)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(emd_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
)

add_library(grasp_execution_interface
  SHARED
  src/action/default_planning_action.cpp
  src/grasp_execution.cpp
  src/core/scheduler.cpp
  src/context.cpp
)

ament_target_dependencies(grasp_execution_interface
  emd_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  trajectory_msgs
  pluginlib
)
target_link_libraries(grasp_execution_interface
  yaml-cpp
)
target_include_directories(grasp_execution_interface
  SYSTEM PRIVATE ${YAML_CPP_INCLUDE_DIRS}
)
target_compile_features(grasp_execution_interface
  PUBLIC cxx_std_17
)

add_library(moveit_cpp_grasp_execution_interface
  SHARED
  src/moveit2/moveit_cpp_if.cpp
)

target_link_libraries(moveit_cpp_grasp_execution_interface
  grasp_execution_interface
)

ament_target_dependencies(moveit_cpp_grasp_execution_interface
  moveit_ros_planning_interface
  trajectory_msgs
  tf2_eigen
)
target_compile_features(moveit_cpp_grasp_execution_interface
  PUBLIC cxx_std_17
)

# Default Executor, Gripper plugin
add_library(grasp_execution_default_plugins
  SHARED
  src/moveit2/executor/default_executor.cpp
  src/gripper/dummy_gripper_driver.cpp
)
ament_target_dependencies(grasp_execution_default_plugins
  moveit_ros_planning_interface
  rclcpp
)
target_compile_features(grasp_execution_default_plugins
  PUBLIC cxx_std_17
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  grasp_execution_interface
  moveit_cpp_grasp_execution_interface
  grasp_execution_default_plugins
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_export_dependencies(
  moveit_ros_planning_interface
  emd_msgs
  tf2
  tf2_ros
  tf2_eigen
  trajectory_msgs
  yaml-cpp
  tinyxml2_vendor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

# TODO(Briancbn): convert to tests instead

ament_export_include_directories(
  include
)
ament_export_libraries(
  grasp_execution_interface
  moveit_cpp_grasp_execution_interface
  grasp_execution_default_plugins
)

pluginlib_export_plugin_description_file(emd_grasp_execution
  grasp_execution_default_plugins.xml)

ament_package()
