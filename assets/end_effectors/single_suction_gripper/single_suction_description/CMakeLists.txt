cmake_minimum_required(VERSION 3.5)
project(single_suction_description)

find_package(ament_cmake REQUIRED)

# This is the critical part that copies the files to the install folder
install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
