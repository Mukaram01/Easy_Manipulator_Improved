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
find_package(onrobot_airpick4_description QUIET)
find_package(onrobot_airpick4_moveit_config QUIET)
find_package(realsense2_description REQUIRED)
find_package(workbench_description REQUIRED)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
set(export_dependencies
  ur_description
  ur5_moveit_config
  realsense2_description
  workbench_description)

if(onrobot_airpick4_description_FOUND)
  list(APPEND export_dependencies onrobot_airpick4_description)
else()
  message(WARNING "onrobot_airpick4_description not found; skipping export dependency.")
endif()

if(onrobot_airpick4_moveit_config_FOUND)
  list(APPEND export_dependencies onrobot_airpick4_moveit_config)
else()
  message(WARNING "onrobot_airpick4_moveit_config not found; skipping export dependency.")
endif()

ament_export_dependencies(${export_dependencies})
ament_package()
