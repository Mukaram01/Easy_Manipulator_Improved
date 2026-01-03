cmake_minimum_required(VERSION 3.10.2)
project(robotiq_85_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake QUIET)
if(NOT ament_cmake_FOUND)
  message(WARNING "ament_cmake not found. Installing robotiq_85_description as a plain CMake package")
endif()

install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

if(ament_cmake_FOUND)
  ament_package()
endif()

