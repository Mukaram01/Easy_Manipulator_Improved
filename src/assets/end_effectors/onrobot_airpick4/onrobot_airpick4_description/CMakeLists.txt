cmake_minimum_required(VERSION 3.10.2)
project(onrobot_airpick4_description)

find_package(ament_cmake QUIET)

if(NOT ament_cmake_FOUND)
  message(WARNING "ament_cmake not found. Building onrobot_airpick4_description
  without ROS support")
endif()

install(DIRECTORY meshes DESTINATION "share/${PROJECT_NAME}")
install(DIRECTORY urdf DESTINATION "share/${PROJECT_NAME}")
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(ament_cmake_FOUND)
  ament_package()
endif()

