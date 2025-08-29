cmake_minimum_required(VERSION 3.10.2)
project(ur_description)

# These description packages do not require any compiled code but some of the
# upstream sources expect ament_cmake to be available.  When building in a
# minimal environment (like the execution sandbox for these exercises) the
# ament packages may be missing which would normally cause configuration to
# fail.  Allow the build to continue without ament_cmake so the meshes and URDF
# files can still be installed using plain CMake.
find_package(ament_cmake QUIET)

if(NOT ament_cmake_FOUND)
  message(WARNING "ament_cmake not found. Building ur_description without ROS
  support")
endif()

# Install the description resources regardless of whether ament is present.
install(DIRECTORY meshes DESTINATION "share/${PROJECT_NAME}")
install(DIRECTORY urdf DESTINATION "share/${PROJECT_NAME}")

if(ament_cmake_FOUND)
  ament_package()
endif()
