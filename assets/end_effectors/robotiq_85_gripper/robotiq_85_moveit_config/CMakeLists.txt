cmake_minimum_required(VERSION 3.5)
project(robotiq_85_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
