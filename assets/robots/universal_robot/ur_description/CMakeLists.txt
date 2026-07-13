cmake_minimum_required(VERSION 3.5)
project(ur_description)

find_package(ament_cmake REQUIRED)

set(UR_DESCRIPTION_RESOURCE_DIRS
  meshes
  urdf
  config
  launch
  rviz
  calibration
)

foreach(resource_dir IN LISTS UR_DESCRIPTION_RESOURCE_DIRS)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${resource_dir}")
    install(
      DIRECTORY "${resource_dir}"
      DESTINATION "share/${PROJECT_NAME}"
    )
  endif()
endforeach()

install(FILES package.xml DESTINATION "share/${PROJECT_NAME}")

ament_package()
