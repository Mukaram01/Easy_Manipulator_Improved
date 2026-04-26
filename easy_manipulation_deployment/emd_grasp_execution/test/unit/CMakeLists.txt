if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Scheduler Unit Test
ament_add_gtest(test_scheduler
  test_scheduler.cpp
)
target_link_libraries(test_scheduler
  grasp_execution_interface
)
target_compile_features(test_scheduler
  PRIVATE cxx_std_17
)

# Context Unit Test
ament_add_gtest(test_context
  test_context.cpp
)
target_link_libraries(test_context
  grasp_execution_interface
)
target_compile_features(test_context
  PRIVATE cxx_std_17
)

# Cartesian planning options unit tests
ament_add_gtest(test_cartesian_planning_options
  test_cartesian_planning_options.cpp
)
target_link_libraries(test_cartesian_planning_options
  moveit_cpp_grasp_execution_interface
)
target_compile_features(test_cartesian_planning_options
  PRIVATE cxx_std_17
)

# MoveIt object pose utility unit tests
ament_add_gtest(test_moveit_object_pose_utils
  test_moveit_object_pose_utils.cpp
)
target_link_libraries(test_moveit_object_pose_utils
  moveit_cpp_grasp_execution_interface
)
target_compile_features(test_moveit_object_pose_utils
  PRIVATE cxx_std_17
)

ament_add_gtest(test_pose_transform_utils
  test_pose_transform_utils.cpp
)
target_link_libraries(test_pose_transform_utils
  grasp_execution_interface
)
target_compile_features(test_pose_transform_utils
  PRIVATE cxx_std_17
)

ament_add_gtest(test_release_collision_ids
  test_release_collision_ids.cpp
)
target_link_libraries(test_release_collision_ids
  moveit_cpp_grasp_execution_interface
)
target_compile_features(test_release_collision_ids
  PRIVATE cxx_std_17
)
