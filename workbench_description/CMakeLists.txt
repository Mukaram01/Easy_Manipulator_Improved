cmake_minimum_required(VERSION 3.5)
project(workbench_description)

find_package(ament_cmake QUIET)

if(NOT ament_cmake_FOUND)
  message(WARNING "ament_cmake not found. Installing workbench_description as a
  plain CMake package")
endif()

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

if(ament_cmake_FOUND)
  ament_package()
endif()

