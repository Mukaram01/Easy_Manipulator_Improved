cmake_minimum_required(VERSION 3.10.2)
project(onrobot_airpick4_moveit_config)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(ament_cmake REQUIRED)


install(DIRECTORY config DESTINATION "share/${PROJECT_NAME}")
install(DIRECTORY launch DESTINATION "share/${PROJECT_NAME}")
ament_package()

