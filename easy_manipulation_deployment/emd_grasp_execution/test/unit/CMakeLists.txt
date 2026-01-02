if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Scheduler Unit Test
find_package(yaml-cpp REQUIRED)
ament_add_gtest(test_scheduler
  test_scheduler.cpp
)
target_link_libraries(test_scheduler
  grasp_execution_interface
  yaml-cpp
)
target_include_directories(test_scheduler
  SYSTEM PRIVATE ${YAML_CPP_INCLUDE_DIRS}
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
  yaml-cpp
)
target_include_directories(test_context
  SYSTEM PRIVATE ${YAML_CPP_INCLUDE_DIRS}
)
target_compile_features(test_context
  PRIVATE cxx_std_17
)
