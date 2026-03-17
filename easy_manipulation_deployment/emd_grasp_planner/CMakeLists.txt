cmake_minimum_required(VERSION 3.5)
project(emd_grasp_planner)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization filters sample_consensus segmentation features)
find_package(OpenMP QUIET)

find_package(fcl QUIET)
if(fcl_FOUND)
  set(FCL_LIBRARIES fcl)
  set(using_new_fcl true)
  message(STATUS "Using FCL version: ${FCL_VERSION}")
else()
  include(FindPkgConfig)
  pkg_check_modules(FCL REQUIRED fcl)
  message(STATUS "Using FCL version: ${FCL_VERSION}")
endif()
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(octomap REQUIRED)

set(EMD_OCTOMAP_LINK_LIBS "")
set(_emd_octomap_targets_valid TRUE)
foreach(_emd_octomap_target IN ITEMS octomap octomath)
  if(TARGET ${_emd_octomap_target})
    get_target_property(_emd_octomap_target_location ${_emd_octomap_target} IMPORTED_LOCATION)
    if(NOT _emd_octomap_target_location)
      get_target_property(_emd_octomap_target_location ${_emd_octomap_target} IMPORTED_LOCATION_RELEASE)
    endif()
    if(NOT _emd_octomap_target_location)
      get_target_property(_emd_octomap_target_location ${_emd_octomap_target} IMPORTED_LOCATION_RELWITHDEBINFO)
    endif()
    if(NOT _emd_octomap_target_location)
      get_target_property(_emd_octomap_target_location ${_emd_octomap_target} IMPORTED_LOCATION_MINSIZEREL)
    endif()
    if(NOT _emd_octomap_target_location)
      get_target_property(_emd_octomap_target_location ${_emd_octomap_target} IMPORTED_LOCATION_DEBUG)
    endif()

    if(_emd_octomap_target_location)
      list(APPEND EMD_OCTOMAP_LINK_LIBS ${_emd_octomap_target})
    else()
      set(_emd_octomap_targets_valid FALSE)
    endif()
  else()
    set(_emd_octomap_targets_valid FALSE)
  endif()
endforeach()

if(NOT _emd_octomap_targets_valid)
  set(EMD_OCTOMAP_LINK_LIBS "")
  unset(_emd_octomap_library_octomap CACHE)
  unset(_emd_octomap_library_octomath CACHE)
  find_library(_emd_octomap_library_octomap NAMES octomap)
  find_library(_emd_octomap_library_octomath NAMES octomath)

  if(_emd_octomap_library_octomap)
    list(APPEND EMD_OCTOMAP_LINK_LIBS ${_emd_octomap_library_octomap})
  endif()
  if(_emd_octomap_library_octomath)
    list(APPEND EMD_OCTOMAP_LINK_LIBS ${_emd_octomap_library_octomath})
  endif()
endif()

if(NOT _emd_octomap_targets_valid AND NOT (_emd_octomap_library_octomap AND _emd_octomap_library_octomath))
  message(FATAL_ERROR "Unable to resolve OctoMap link libraries: neither valid imported targets (octomap/octomath with locations) nor concrete library files were found.")
endif()

find_package(sensor_msgs REQUIRED)
find_package(emd_msgs REQUIRED)
find_package(epd_msgs QUIET)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

include_directories(
  PUBLIC
  include
  msg
)

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

add_library(grasp_planning_interface
  SHARED
  src/grasp_scene.cpp
  src/grasp_object.cpp
  src/epd_detection_adapter.cpp
  src/end_effectors/finger_gripper.cpp
  src/end_effectors/suction_gripper.cpp
  src/common/pcl_functions.cpp
  src/common/pcl_visualizer.cpp
  src/common/fcl_functions.cpp
  src/common/math_functions.cpp
)

target_include_directories(grasp_planning_interface SYSTEM PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)


# Keep strict warnings for project-owned targets and scope suppressions to
# specific third-party imported targets that are known to emit benign warnings.
function(emd_suppress_external_target_warnings external_target)
  if(NOT TARGET ${external_target})
    return()
  endif()

  # Treat the target as external so consumer targets receive its headers as
  # SYSTEM includes when possible. This keeps strict warnings enabled for
  # project code while reducing noise from vendored third-party headers.
  get_target_property(_emd_external_includes ${external_target} INTERFACE_INCLUDE_DIRECTORIES)
  if(_emd_external_includes)
    set_property(TARGET ${external_target} APPEND PROPERTY
      INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_emd_external_includes}")
  endif()

  get_target_property(_emd_external_is_imported ${external_target} IMPORTED)
  if(_emd_external_is_imported)
    set_property(TARGET ${external_target} PROPERTY SYSTEM TRUE)
  endif()

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set_property(TARGET ${external_target} APPEND PROPERTY INTERFACE_COMPILE_OPTIONS
      -Wno-unused-parameter
      -Wno-deprecated-declarations)
  endif()
endfunction()

emd_suppress_external_target_warnings(octomap)
emd_suppress_external_target_warnings(octomath)
emd_suppress_external_target_warnings(qpoases)

if(${FCL_VERSION} VERSION_GREATER_EQUAL 0.6.0)
  target_compile_definitions(grasp_planning_interface
    PUBLIC
    FCL_VERSION_0_6_OR_HIGHER=1
  )
else()
  target_compile_definitions(grasp_planning_interface
    PUBLIC
    FCL_VERSION_0_6_OR_HIGHER=0
  )
endif()

if(epd_msgs_FOUND)
  ament_target_dependencies(grasp_planning_interface
        tf2
        rclcpp
        emd_msgs
        epd_msgs
        vision_msgs
        sensor_msgs
        geometry_msgs
        shape_msgs
        tf2_ros
        tf2_geometry_msgs
        message_filters
        cv_bridge
        visualization_msgs)
  target_compile_definitions(grasp_planning_interface
    PUBLIC
    EPD_ENABLED=1
  )
else()
  ament_target_dependencies(grasp_planning_interface
        tf2
        rclcpp
        emd_msgs
        vision_msgs
        sensor_msgs
        geometry_msgs
        shape_msgs
        tf2_ros
        tf2_geometry_msgs
        message_filters
        cv_bridge
        visualization_msgs)
  target_compile_definitions(grasp_planning_interface
    PUBLIC
    EPD_ENABLED=0
  )
endif()

target_link_libraries(grasp_planning_interface
  ${PCL_LIBRARIES}
  ${FCL_LIBRARIES}
  ${EMD_OCTOMAP_LINK_LIBS}
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(grasp_planning_interface OpenMP::OpenMP_CXX)
  ament_export_dependencies(OpenMP)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

# install(
#   TARGETS
#   grasp_planning_interface
#   RUNTIME DESTINATION bin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib)

# ament_export_include_directories(
#   include
# )

# ament_export_libraries(
#   grasp_planning_interface
# )

# add_subdirectory(
#   example
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(grasptest test/init_tests.cpp)
  ament_target_dependencies(grasptest ament_index_cpp)

  if(${FCL_VERSION} VERSION_GREATER_EQUAL 0.6.0)
  target_compile_definitions(grasptest
    PUBLIC
    FCL_VERSION_0_6_OR_HIGHER=1
  )
  else()
    target_compile_definitions(grasptest
      PUBLIC
      FCL_VERSION_0_6_OR_HIGHER=0
    )
  endif()

  target_link_libraries(grasptest
  grasp_planning_interface
  ${PCL_LIBRARIES}
  ${EMD_OCTOMAP_LINK_LIBS}
  ${FCL_LIBRARIES}
  ${GTEST_LIBRARIES}
  ${Boost_LIBRARIES}
  ccd)

  ament_export_dependencies(
    rosidl_default_runtime)

endif()


install(
  TARGETS grasp_planning_interface
  EXPORT export_grasp_planning_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)

ament_export_include_directories(
  include
)

ament_export_libraries(grasp_planning_interface)
ament_export_targets(export_grasp_planning_interface HAS_LIBRARY_TARGET)

if(epd_msgs_FOUND)
  ament_export_dependencies(
    emd_msgs
    epd_msgs
    vision_msgs
    shape_msgs
    visualization_msgs
    cv_bridge
    message_filters
    tf2_ros
    tf2_geometry_msgs
    PCL
    octomap
  )
else()
  ament_export_dependencies(
    emd_msgs
    vision_msgs
    shape_msgs
    visualization_msgs
    cv_bridge
    message_filters
    tf2_ros
    tf2_geometry_msgs
    PCL
    octomap
  )
endif()

ament_package()
