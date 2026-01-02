cmake_minimum_required(VERSION 3.10.2)
project(panda_description)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY meshes urdf DESTINATION share/${PROJECT_NAME})
