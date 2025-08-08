cmake_minimum_required(VERSION 3.5)
project(ur5_airpick4_test)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(ur_description REQUIRED)
find_package(ur5_moveit_config REQUIRED)
find_package(onrobot_airpick4_description REQUIRED)
find_package(onrobot_airpick4_moveit_config REQUIRED)
find_package(realsense2_description REQUIRED)
find_package(table_description REQUIRED)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
ament_export_dependencies(
  ur_description
  ur5_moveit_config
  onrobot_airpick4_description
  onrobot_airpick4_moveit_config
  realsense2_description
  table_description)
ament_package()
