cmake_minimum_required(VERSION 3.16)
project(emd_dynamic_safety)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)

find_package(joint_trajectory_controller REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)

find_package(angles REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(moveit_msgs REQUIRED)
find_package(moveit_core QUIET)

# Optional backends
find_package(tesseract_environment QUIET)
find_package(tesseract_task_composer QUIET)

# Collect optional backend libraries to link into the core library
set(PLANNING_FRAMEWORK_LIBS "")

# Track whether we actually built the optional backends
set(EMD_DYNAMIC_SAFETY_HAS_MOVEIT_BACKEND OFF)
set(EMD_DYNAMIC_SAFETY_HAS_TESSERACT_BACKEND OFF)

# ------------------------------------------------------------------------------
# Include dirs
# ------------------------------------------------------------------------------
include_directories(
  PUBLIC
  include
)

# ------------------------------------------------------------------------------
# MoveIt backend (optional)
# ------------------------------------------------------------------------------
if(moveit_core_FOUND)
  message(STATUS "MoveIt Found!!")

  add_library(${PROJECT_NAME}_moveit SHARED
    src/moveit/collision_checker_moveit.cpp
    src/moveit/replanner_moveit.cpp
  )

  target_include_directories(${PROJECT_NAME}_moveit PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_NAME}_moveit
    rclcpp
    moveit_core
    pluginlib
  )

  target_compile_definitions(${PROJECT_NAME}_moveit PUBLIC EMD_DYNAMIC_SAFETY_MOVEIT)

  ament_export_libraries(${PROJECT_NAME}_moveit)
  list(APPEND PLANNING_FRAMEWORK_LIBS ${PROJECT_NAME}_moveit)

  install(TARGETS ${PROJECT_NAME}_moveit
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
  )

  set(EMD_DYNAMIC_SAFETY_HAS_MOVEIT_BACKEND ON)
else()
  message(WARNING "MoveIt Not Found!!")
endif()

# ------------------------------------------------------------------------------
# Tesseract backend (optional)
# Build ONLY if:
#  - tesseract_environment + tesseract_task_composer are found
#  - AND the backend source files exist in this repository
# ------------------------------------------------------------------------------
set(EMD_TESSERACT_SRC
  src/tesseract/collision_checker_tesseract.cpp
  src/tesseract/replanner_tesseract.cpp
)

set(EMD_TESSERACT_SRC_MISSING OFF)
foreach(_f IN LISTS EMD_TESSERACT_SRC)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_f}")
    message(WARNING "Missing Tesseract backend source: ${_f}")
    set(EMD_TESSERACT_SRC_MISSING ON)
  endif()
endforeach()

if(tesseract_environment_FOUND AND tesseract_task_composer_FOUND AND NOT EMD_TESSERACT_SRC_MISSING
    AND TARGET tesseract::tesseract_environment_core)
  message(STATUS "Tesseract Found!! (and backend sources present)")

  if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
  endif()

  find_package(tesseract_rosutils REQUIRED)
  find_package(tesseract_msgs REQUIRED)
  find_package(tesseract_motion_planners REQUIRED)

  add_library(${PROJECT_NAME}_tesseract SHARED
    ${EMD_TESSERACT_SRC}
  )

  target_include_directories(${PROJECT_NAME}_tesseract PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  set(EMD_DYNAMIC_SAFETY_TESSERACT_LIBS "")

  foreach(_tgt IN ITEMS
    tesseract::tesseract_environment_core
    tesseract::tesseract_environment_kdl
    tesseract::tesseract_environment_ofkt
    tesseract::tesseract_collision_core
    tesseract::tesseract_collision_bullet
    tesseract::tesseract_collision_fcl
    tesseract::tesseract_kinematics_core
    tesseract::tesseract_kinematics_opw
    tesseract::tesseract_kinematics_kdl
    tesseract::tesseract_motion_planners_core
    tesseract::tesseract_motion_planners_ompl
    tesseract::tesseract_motion_planners_trajopt
    tesseract::tesseract_motion_planners_trajopt_ifopt
    tesseract::tesseract_time_parameterization
    tesseract::tesseract_rosutils
  )
    if(TARGET ${_tgt})
      list(APPEND EMD_DYNAMIC_SAFETY_TESSERACT_LIBS ${_tgt})
    else()
      message(WARNING "Skipping ${_tgt} (target not found).")
    endif()
  endforeach()

  target_link_libraries(${PROJECT_NAME}_tesseract
    ${EMD_DYNAMIC_SAFETY_TESSERACT_LIBS}
  )

  ament_target_dependencies(${PROJECT_NAME}_tesseract
    rclcpp
    tesseract_environment
    tesseract_task_composer
    tesseract_rosutils
    tesseract_msgs
    tesseract_motion_planners
  )

  target_compile_definitions(${PROJECT_NAME}_tesseract PUBLIC EMD_DYNAMIC_SAFETY_TESSERACT)

  ament_export_libraries(${PROJECT_NAME}_tesseract)
  list(APPEND PLANNING_FRAMEWORK_LIBS ${PROJECT_NAME}_tesseract)

  install(TARGETS ${PROJECT_NAME}_tesseract
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
  )

  set(EMD_DYNAMIC_SAFETY_HAS_TESSERACT_BACKEND ON)
else()
  message(WARNING "Tesseract backend skipped (deps missing or backend sources missing).")
endif()

# ------------------------------------------------------------------------------
# Core library
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  # src/next_point_publisher.cpp
  src/replanner.cpp
  src/safety_zone.cpp
  src/collision_checker.cpp
  src/interpolate.cpp
  src/visualizer.cpp
  src/dynamic_safety.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${PLANNING_FRAMEWORK_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  realtime_tools
  pluginlib
  std_msgs
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  visualization_msgs
  moveit_msgs
)

# Feature flags on core
if(EMD_DYNAMIC_SAFETY_HAS_MOVEIT_BACKEND)
  target_compile_definitions(${PROJECT_NAME} PUBLIC EMD_DYNAMIC_SAFETY_MOVEIT)
endif()

if(EMD_DYNAMIC_SAFETY_HAS_TESSERACT_BACKEND)
  target_compile_definitions(${PROJECT_NAME} PUBLIC EMD_DYNAMIC_SAFETY_TESSERACT)
endif()

# ------------------------------------------------------------------------------
# Controller plugin
# ------------------------------------------------------------------------------
add_library(${PROJECT_NAME}_controller SHARED
  src/dynamic_safety_trajectory_controller.cpp
)

target_link_libraries(${PROJECT_NAME}_controller
  ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_controller
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  joint_trajectory_controller
  controller_interface
  hardware_interface
  realtime_tools
  pluginlib
  angles
  std_msgs
  sensor_msgs
  trajectory_msgs
)

get_target_property(target_options ${PROJECT_NAME}_controller COMPILE_OPTIONS)
if(NOT target_options STREQUAL "target_options-NOTFOUND")
  list(REMOVE_ITEM target_options "-Wpedantic")
  list(REMOVE_ITEM target_options "-Wconversion")
  list(REMOVE_ITEM target_options "-Wsign-conversion")
  set_property(TARGET ${PROJECT_NAME}_controller PROPERTY COMPILE_OPTIONS ${target_options})
endif()

target_compile_definitions(${PROJECT_NAME}_controller PRIVATE "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# ------------------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

# ------------------------------------------------------------------------------
# Exports
# ------------------------------------------------------------------------------
ament_export_include_directories(include)

ament_export_libraries(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_controller
)

set(EMD_DYNAMIC_SAFETY_EXPORT_DEPENDENCIES
  angles
  controller_interface
  geometry_msgs
  hardware_interface
  lifecycle_msgs
  moveit_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
  trajectory_msgs
  visualization_msgs
)

if(EMD_DYNAMIC_SAFETY_HAS_TESSERACT_BACKEND)
  list(APPEND EMD_DYNAMIC_SAFETY_EXPORT_DEPENDENCIES
    tesseract_environment
    tesseract_motion_planners
    tesseract_msgs
    tesseract_task_composer
    tesseract_rosutils
  )
endif()

ament_export_dependencies(${EMD_DYNAMIC_SAFETY_EXPORT_DEPENDENCIES})

ament_package()

