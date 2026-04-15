cmake_minimum_required(VERSION 3.16)
project(run_waypoint_execution)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(emd_grasp_execution REQUIRED)
find_package(emd_waypoint_execution REQUIRED)
find_package(emd_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(target_validation src/target_validation.cpp)
ament_target_dependencies(target_validation
  emd_grasp_execution
  emd_msgs
  geometry_msgs
  rclcpp
)
target_include_directories(target_validation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(demo_node src/demo_node.cpp)
target_link_libraries(demo_node target_validation)
target_include_directories(demo_node PRIVATE include)

ament_target_dependencies(demo_node
  emd_grasp_execution
  emd_waypoint_execution
  emd_msgs
  rclcpp
)

add_executable(fake_grasp_pose_publisher src/fake_grasp_pose_publisher.cpp)
ament_target_dependencies(fake_grasp_pose_publisher
  emd_grasp_execution
  emd_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  target_validation
  demo_node
  fake_grasp_pose_publisher
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_target_validation test/test_target_validation.cpp)
  if(TARGET test_target_validation)
    target_link_libraries(test_target_validation target_validation)
    target_include_directories(test_target_validation PRIVATE include)
    ament_target_dependencies(test_target_validation
      emd_grasp_execution
      emd_msgs
      geometry_msgs
      rclcpp
    )
  endif()

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
