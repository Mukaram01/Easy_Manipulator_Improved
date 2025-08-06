# Scheduler Unit Test
find_package(yaml-cpp REQUIRED)
ament_add_gtest(test_scheduler
  test_scheduler.cpp
)
target_link_libraries(test_scheduler
  grasp_execution_interface
  yaml-cpp::yaml-cpp
)
target_include_directories(test_scheduler
  PUBLIC ${YAML_CPP_INCLUDE_DIRS}
)
target_compile_features(test_scheduler
  PUBLIC cxx_std_17
)
