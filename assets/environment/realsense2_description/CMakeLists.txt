cmake_minimum_required(VERSION 3.5)
project(realsense2_description)

# This package only installs description resources.  Attempt to find
# ament_cmake so that it behaves as a normal ROS package when the dependency is
# available, but fall back to a plain CMake package when it is not.
find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
  ament_package()
else()
  message(WARNING "ament_cmake not found. Installing realsense2_description as a
  generic CMake package")
endif()

# Install resources regardless of build system.
install(DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME})
