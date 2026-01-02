cmake_minimum_required(VERSION 3.5)
project(workbench_description)

find_package(ament_cmake QUIET)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

if(ament_cmake_FOUND AND BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

if(ament_cmake_FOUND)
  ament_package()
endif()
