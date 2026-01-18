cmake_minimum_required(VERSION 3.5)
project(onrobot_airpick4_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
