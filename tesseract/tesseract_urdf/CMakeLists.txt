cmake_minimum_required(VERSION 3.15.0)

include("${CMAKE_CURRENT_LIST_DIR}/../tesseract_common/cmake/tesseract_cmake_boilerplate.cmake")
tesseract_initialize_boilerplate_support()
include("${CMAKE_CURRENT_LIST_DIR}/../tesseract_common/cmake/tesseract_macros.cmake")
if(NOT COMMAND extract_package_metadata)
  message(FATAL_ERROR "Failed to load tesseract_cmake_boilerplate.cmake; extract_package_metadata is unavailable")
endif()

# Extract package name and version
extract_package_metadata(pkg PACKAGE_XML "${CMAKE_CURRENT_LIST_DIR}/package.xml")
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(Boost REQUIRED)
set(_tesseract_urdf_have_boost_graph FALSE)
if(TARGET Boost::graph)
  set(_tesseract_urdf_have_boost_graph TRUE)
else()
  find_package(Boost QUIET COMPONENTS graph)
  if(TARGET Boost::graph)
    set(_tesseract_urdf_have_boost_graph TRUE)
  endif()
endif()

if(NOT _tesseract_urdf_have_boost_graph)
  message(STATUS "Boost::graph component not found; providing a header-only interface target")
  if(NOT TARGET Boost::graph)
    add_library(Boost::graph INTERFACE IMPORTED)
  endif()
  if(TARGET Boost::headers)
    target_link_libraries(Boost::graph INTERFACE Boost::headers)
  elseif(TARGET Boost::boost)
    target_link_libraries(Boost::graph INTERFACE Boost::boost)
  elseif(Boost_INCLUDE_DIRS)
    target_include_directories(Boost::graph INTERFACE ${Boost_INCLUDE_DIRS})
  endif()
endif()
find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(PCL QUIET COMPONENTS io)
set(TESSERACT_URDF_HAS_PCL ${PCL_FOUND})
if(NOT TESSERACT_URDF_HAS_PCL)
  message(STATUS "PCL not found; disabling point cloud parsing support in tesseract_urdf")
endif()
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(tesseract_collision REQUIRED COMPONENTS bullet)
find_package(PkgConfig REQUIRED)
pkg_check_modules(tinyxml2 REQUIRED tinyxml2)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                  ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                    ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/test/*
    /*/install/*
    /*/devel/*
    /*/gtest/*
    /*/bullet/LinearMath/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

set(
  _tesseract_urdf_sources
  src/box.cpp
  src/calibration.cpp
  src/capsule.cpp
  src/collision.cpp
  src/cone.cpp
  src/cylinder.cpp
  src/dynamics.cpp
  src/geometry.cpp
  src/inertial.cpp
  src/joint.cpp
  src/limits.cpp
  src/link.cpp
  src/material.cpp
  src/mesh.cpp
  src/mimic.cpp
  src/octomap.cpp
  src/octree.cpp
  src/origin.cpp
  src/safety_controller.cpp
  src/sdf_mesh.cpp
  src/sphere.cpp
  src/urdf_parser.cpp
  src/visual.cpp
  src/utils.cpp)

if(TESSERACT_URDF_HAS_PCL)
  list(APPEND _tesseract_urdf_sources src/point_cloud.cpp)
endif()

add_library(${PROJECT_NAME} ${_tesseract_urdf_sources})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Boost::boost
         Boost::graph
         Eigen3::Eigen
         ${tinyxml2_LIBRARIES}
         tesseract::tesseract_common
         tesseract::tesseract_geometry
         tesseract::tesseract_scene_graph
         tesseract::tesseract_collision_bullet
         console_bridge::console_bridge)
target_link_libraries(${PROJECT_NAME} PRIVATE Boost::boost console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})

# PCL does not support c++11 on Xenial so cannot include point cloud parsing from urdf Boost version number is in XYYYZZ
# format such that: (BOOST_VERSION % 100) is the sub-minor version ((BOOST_VERSION / 100) % 1000) is the minor version
# (BOOST_VERSION_MACRO / 100000) is the major version.
if(TESSERACT_URDF_HAS_PCL)
  if(Boost_VERSION_MACRO)
    set(TESSERACT_Boost_VERSION_MACRO ${Boost_VERSION_MACRO})
  else()
    set(TESSERACT_Boost_VERSION_MACRO ${Boost_VERSION})
  endif()
  if(TESSERACT_Boost_VERSION_MACRO VERSION_GREATER "106000")
    target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_IO_LIBRARIES} ${PCL_COMMON_LIBRARIES})
    target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
    target_compile_definitions(${PROJECT_NAME} PUBLIC TESSERACT_PARSE_POINT_CLOUDS="ON")
    foreach(DEF ${PCL_DEFINITIONS})
      string(STRIP ${DEF} DEF)
      if(NOT "${DEF}" STREQUAL "")
        string(SUBSTRING "${DEF}" 0 2 DEF_PREFIX)
        if("${DEF_PREFIX}" STREQUAL "-m")
          string(REPLACE " " ";" DEF ${DEF})
          foreach(OPTION_DEF ${DEF})
            target_compile_options(${PROJECT_NAME} PUBLIC ${OPTION_DEF})
          endforeach()
        else()
          target_compile_definitions(${PROJECT_NAME} PUBLIC ${DEF})
        endif()
      endif()
    endforeach()
  else()
    message(WARNING "Boost version is ${TESSERACT_Boost_VERSION_MACRO}; skipping point cloud parsing support")
  endif()
endif()

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>"
                                                  ${tinyxml2_INCLUDE_DIRS})

target_clang_tidy(${PROJECT_NAME} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

set(_tesseract_urdf_dependencies
    Boost
    Eigen3
    console_bridge
    PkgConfig
    tinyxml2
    "tesseract_collision COMPONENTS bullet"
    tesseract_geometry
    tesseract_common
    tesseract_scene_graph)
if(TESSERACT_URDF_HAS_PCL)
  list(APPEND _tesseract_urdf_dependencies "PCL COMPONENTS common io")
endif()

configure_package(
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}
  DEPENDENCIES ${_tesseract_urdf_dependencies}
  CFG_EXTRAS cmake/tesseract_urdf-extras.cmake)

# Mark cpp header files for installation
install(FILES include/${PROJECT_NAME}/urdf_parser.h DESTINATION include/${PROJECT_NAME})
install(FILES include/${PROJECT_NAME}/utils.h DESTINATION include/${PROJECT_NAME})

# ADD Examples
if(TESSERACT_ENABLE_EXAMPLES OR TESSERACT_URDF_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

if(TESSERACT_ENABLE_TESTING OR TESSERACT_URDF_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  cpack(
    VERSION ${pkg_extracted_version}
    MAINTAINER_NAME ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "libpcl-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-bullet"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
    WINDOWS_DEPENDS
      "console-bridge"
      "Eigen3"
      "pcl"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-bullet"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
