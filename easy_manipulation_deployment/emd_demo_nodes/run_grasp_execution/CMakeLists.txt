cmake_minimum_required(VERSION 3.16)
project(run_grasp_execution)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(emd_grasp_execution REQUIRED)
find_package(emd_waypoint_execution REQUIRED)
find_package(emd_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(demo_node src/demo_node.cpp)
target_include_directories(demo_node PUBLIC include)

ament_target_dependencies(demo_node
  emd_grasp_execution
  emd_waypoint_execution
  emd_msgs
  rclcpp
  rclcpp_lifecycle
  ament_index_cpp
)

add_executable(fake_grasp_pose_publisher src/fake_grasp_pose_publisher.cpp)
target_include_directories(fake_grasp_pose_publisher PUBLIC include)
ament_target_dependencies(fake_grasp_pose_publisher
  emd_grasp_execution
  emd_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  demo_node
  fake_grasp_pose_publisher
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py" PATTERN "*.launch.xml"
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_grasp_execution_launch.test.py TIMEOUT 120)
  ament_add_gtest(test_grasp_precheck_collision_filter
    test/test_grasp_precheck_collision_filter.cpp)
  target_include_directories(test_grasp_precheck_collision_filter PUBLIC include)
  ament_add_gtest(test_grasp_candidate_utils
    test/test_grasp_candidate_utils.cpp)
  target_include_directories(test_grasp_candidate_utils PUBLIC include)
  ament_target_dependencies(test_grasp_candidate_utils
    tf2_geometry_msgs
  )
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
