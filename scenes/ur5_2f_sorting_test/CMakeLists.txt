cmake_minimum_required(VERSION 3.5)
project(ur5_2f_sorting_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  environment.yaml
  scene_manifest.yaml
  sorting_manifest.yaml
  README.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_sorting_plan.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME generate_sorting_plan
)

install(PROGRAMS
  scripts/validate_scene_layout.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME validate_scene_layout
)

if(BUILD_TESTING)
  add_test(
    NAME ur5_2f_sorting_test_sorting_manifest_validation
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sorting_manifest.py
  )

  add_test(
    NAME ur5_2f_sorting_test_generate_sorting_plan_output
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_generate_sorting_plan.py
  )

  add_test(
    NAME ur5_2f_sorting_test_validate_scene_layout
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_validate_scene_layout.py
  )
endif()

ament_package()
