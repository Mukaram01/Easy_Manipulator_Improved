set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DTEST_DIRECTORY=\"${TEST_DIR}\")

ament_add_gtest(test_waypoint_parser
  test_waypoint_parser.cpp
  ../src/waypoint_parser.cpp)

ament_target_dependencies(test_waypoint_parser
  rclcpp
  geometry_msgs
  tf2_geometry_msgs)

target_link_libraries(test_waypoint_parser
  jsoncpp_lib)

target_compile_definitions(test_waypoint_parser PRIVATE EMD_WAYPOINT_PARSER_DISABLE_EXECUTION)
