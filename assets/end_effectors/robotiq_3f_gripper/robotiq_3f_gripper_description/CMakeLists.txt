cmake_minimum_required(VERSION 3.5)
project(robotiq_3f_gripper_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
