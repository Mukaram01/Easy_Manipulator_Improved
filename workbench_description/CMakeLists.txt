cmake_minimum_required(VERSION 3.5)
project(workbench_description)

find_package(ament_cmake QUIET)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})

if(ament_cmake_FOUND)
  ament_package()
endif()

