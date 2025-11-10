cmake_minimum_required(VERSION 3.15.0)

include("${CMAKE_CURRENT_LIST_DIR}/../tesseract_common/cmake/tesseract_cmake_boilerplate.cmake")
tesseract_initialize_boilerplate_support()
if(NOT COMMAND extract_package_metadata)
  message(FATAL_ERROR "Failed to load tesseract_cmake_boilerplate.cmake; extract_package_metadata is unavailable")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/../tesseract_common/cmake/tesseract_macros.cmake")

# Extract package name and version
extract_package_metadata(pkg PACKAGE_XML "${CMAKE_CURRENT_LIST_DIR}/package.xml")
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# All of these are used by all components so to avoid having them in each they are include here
find_package(Eigen3 REQUIRED)
find_package(
  Boost REQUIRED
  COMPONENTS filesystem system program_options serialization stacktrace_backtrace stacktrace_noop)
find_package(octomap REQUIRED)
find_package(console_bridge REQUIRED)
find_package(assimp REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(yaml-cpp REQUIRED)

set(_tesseract_common_have_exports FALSE)
foreach(_var IN ITEMS LIBRARIES INCLUDE_DIRS COMPILE_DEFINITIONS COMPILE_OPTIONS)
  if(DEFINED tesseract_common_${_var} AND tesseract_common_${_var})
    set(_tesseract_common_have_exports TRUE)
  endif()
endforeach()
if(NOT TARGET tesseract::tesseract_common)
  if(TARGET tesseract_common::tesseract_common)
    add_library(tesseract::tesseract_common ALIAS tesseract_common::tesseract_common)
  elseif(TARGET tesseract_common)
    add_library(tesseract::tesseract_common ALIAS tesseract_common)
  elseif(_tesseract_common_have_exports)
    add_library(tesseract::tesseract_common INTERFACE IMPORTED GLOBAL)
    if(DEFINED tesseract_common_LIBRARIES AND tesseract_common_LIBRARIES)
      set_property(TARGET tesseract::tesseract_common PROPERTY INTERFACE_LINK_LIBRARIES
                   ${tesseract_common_LIBRARIES})
    endif()
    if(DEFINED tesseract_common_INCLUDE_DIRS AND tesseract_common_INCLUDE_DIRS)
      set_property(TARGET tesseract::tesseract_common PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                   ${tesseract_common_INCLUDE_DIRS})
    endif()
    if(DEFINED tesseract_common_COMPILE_DEFINITIONS AND tesseract_common_COMPILE_DEFINITIONS)
      set_property(TARGET tesseract::tesseract_common PROPERTY INTERFACE_COMPILE_DEFINITIONS
                   ${tesseract_common_COMPILE_DEFINITIONS})
    endif()
    if(DEFINED tesseract_common_COMPILE_OPTIONS AND tesseract_common_COMPILE_OPTIONS)
      set_property(TARGET tesseract::tesseract_common PROPERTY INTERFACE_COMPILE_OPTIONS
                   ${tesseract_common_COMPILE_OPTIONS})
    endif()
  endif()
endif()

if(NOT TARGET tesseract_common::tesseract_common)
  if(TARGET tesseract::tesseract_common)
    add_library(tesseract_common::tesseract_common INTERFACE IMPORTED GLOBAL)
    set_target_properties(tesseract_common::tesseract_common PROPERTIES
                          INTERFACE_LINK_LIBRARIES tesseract::tesseract_common)
  elseif(TARGET tesseract_common)
    add_library(tesseract_common::tesseract_common ALIAS tesseract_common)
  elseif(_tesseract_common_have_exports)
    add_library(tesseract_common::tesseract_common INTERFACE IMPORTED GLOBAL)
    if(DEFINED tesseract_common_LIBRARIES AND tesseract_common_LIBRARIES)
      set_property(TARGET tesseract_common::tesseract_common PROPERTY INTERFACE_LINK_LIBRARIES
                   ${tesseract_common_LIBRARIES})
    endif()
    if(DEFINED tesseract_common_INCLUDE_DIRS AND tesseract_common_INCLUDE_DIRS)
      set_property(TARGET tesseract_common::tesseract_common PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                   ${tesseract_common_INCLUDE_DIRS})
    endif()
    if(DEFINED tesseract_common_COMPILE_DEFINITIONS AND tesseract_common_COMPILE_DEFINITIONS)
      set_property(TARGET tesseract_common::tesseract_common PROPERTY INTERFACE_COMPILE_DEFINITIONS
                   ${tesseract_common_COMPILE_DEFINITIONS})
    endif()
    if(DEFINED tesseract_common_COMPILE_OPTIONS AND tesseract_common_COMPILE_OPTIONS)
      set_property(TARGET tesseract_common::tesseract_common PROPERTY INTERFACE_COMPILE_OPTIONS
                   ${tesseract_common_COMPILE_OPTIONS})
    endif()
  endif()
endif()

set(_tesseract_geometry_have_exports FALSE)
foreach(_var IN ITEMS LIBRARIES INCLUDE_DIRS COMPILE_DEFINITIONS COMPILE_OPTIONS)
  if(DEFINED tesseract_geometry_${_var} AND tesseract_geometry_${_var})
    set(_tesseract_geometry_have_exports TRUE)
  endif()
endforeach()
if(NOT TARGET tesseract::tesseract_geometry)
  if(TARGET tesseract_geometry::tesseract_geometry)
    add_library(tesseract::tesseract_geometry ALIAS tesseract_geometry::tesseract_geometry)
  elseif(TARGET tesseract_geometry)
    add_library(tesseract::tesseract_geometry ALIAS tesseract_geometry)
  elseif(_tesseract_geometry_have_exports)
    add_library(tesseract::tesseract_geometry INTERFACE IMPORTED GLOBAL)
    if(DEFINED tesseract_geometry_LIBRARIES AND tesseract_geometry_LIBRARIES)
      set_property(TARGET tesseract::tesseract_geometry PROPERTY INTERFACE_LINK_LIBRARIES
                   ${tesseract_geometry_LIBRARIES})
    endif()
    if(DEFINED tesseract_geometry_INCLUDE_DIRS AND tesseract_geometry_INCLUDE_DIRS)
      set_property(TARGET tesseract::tesseract_geometry PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                   ${tesseract_geometry_INCLUDE_DIRS})
    endif()
    if(DEFINED tesseract_geometry_COMPILE_DEFINITIONS AND tesseract_geometry_COMPILE_DEFINITIONS)
      set_property(TARGET tesseract::tesseract_geometry PROPERTY INTERFACE_COMPILE_DEFINITIONS
                   ${tesseract_geometry_COMPILE_DEFINITIONS})
    endif()
    if(DEFINED tesseract_geometry_COMPILE_OPTIONS AND tesseract_geometry_COMPILE_OPTIONS)
      set_property(TARGET tesseract::tesseract_geometry PROPERTY INTERFACE_COMPILE_OPTIONS
                   ${tesseract_geometry_COMPILE_OPTIONS})
    endif()
  endif()
endif()

if(NOT TARGET tesseract_geometry::tesseract_geometry)
  if(TARGET tesseract::tesseract_geometry)
    add_library(tesseract_geometry::tesseract_geometry INTERFACE IMPORTED GLOBAL)
    set_target_properties(tesseract_geometry::tesseract_geometry PROPERTIES
                          INTERFACE_LINK_LIBRARIES tesseract::tesseract_geometry)
  elseif(TARGET tesseract_geometry)
    add_library(tesseract_geometry::tesseract_geometry ALIAS tesseract_geometry)
  elseif(_tesseract_geometry_have_exports)
    add_library(tesseract_geometry::tesseract_geometry INTERFACE IMPORTED GLOBAL)
    if(DEFINED tesseract_geometry_LIBRARIES AND tesseract_geometry_LIBRARIES)
      set_property(TARGET tesseract_geometry::tesseract_geometry PROPERTY INTERFACE_LINK_LIBRARIES
                   ${tesseract_geometry_LIBRARIES})
    endif()
    if(DEFINED tesseract_geometry_INCLUDE_DIRS AND tesseract_geometry_INCLUDE_DIRS)
      set_property(TARGET tesseract_geometry::tesseract_geometry PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                   ${tesseract_geometry_INCLUDE_DIRS})
    endif()
    if(DEFINED tesseract_geometry_COMPILE_DEFINITIONS AND tesseract_geometry_COMPILE_DEFINITIONS)
      set_property(TARGET tesseract_geometry::tesseract_geometry PROPERTY INTERFACE_COMPILE_DEFINITIONS
                   ${tesseract_geometry_COMPILE_DEFINITIONS})
    endif()
    if(DEFINED tesseract_geometry_COMPILE_OPTIONS AND tesseract_geometry_COMPILE_OPTIONS)
      set_property(TARGET tesseract_geometry::tesseract_geometry PROPERTY INTERFACE_COMPILE_OPTIONS
                   ${tesseract_geometry_COMPILE_OPTIONS})
    endif()
  endif()
endif()

unset(_tesseract_common_have_exports)
unset(_tesseract_geometry_have_exports)

set(TESSERACT_COMMON_TARGET tesseract::tesseract_common)
if(NOT TARGET ${TESSERACT_COMMON_TARGET} AND TARGET tesseract_common::tesseract_common)
  set(TESSERACT_COMMON_TARGET tesseract_common::tesseract_common)
elseif(NOT TARGET ${TESSERACT_COMMON_TARGET} AND TARGET tesseract_common)
  set(TESSERACT_COMMON_TARGET tesseract_common)
endif()

set(TESSERACT_GEOM_TARGET tesseract::tesseract_geometry)
if(NOT TARGET ${TESSERACT_GEOM_TARGET} AND TARGET tesseract_geometry::tesseract_geometry)
  set(TESSERACT_GEOM_TARGET tesseract_geometry::tesseract_geometry)
elseif(NOT TARGET ${TESSERACT_GEOM_TARGET} AND TARGET tesseract_geometry)
  set(TESSERACT_GEOM_TARGET tesseract_geometry)
endif()

if(NOT TARGET ${TESSERACT_COMMON_TARGET})
  message(FATAL_ERROR
          "tesseract_collision: Failed to locate a tesseract_common target (expected either "
          "tesseract::tesseract_common, tesseract_common::tesseract_common, or tesseract_common)")
else()
  if("${TESSERACT_COMMON_TARGET}" STREQUAL "tesseract::tesseract_common")
    set(_tesseract_common_namespace "tesseract::")
  elseif("${TESSERACT_COMMON_TARGET}" STREQUAL "tesseract_common::tesseract_common")
    set(_tesseract_common_namespace "tesseract_common::")
  else()
    set(_tesseract_common_namespace "${TESSERACT_COMMON_TARGET}")
  endif()
  message(
    STATUS
      "tesseract_collision: using ${TESSERACT_COMMON_TARGET} for tesseract_common exports (namespace ${_tesseract_common_namespace})")
endif()

if(NOT TARGET ${TESSERACT_GEOM_TARGET})
  message(FATAL_ERROR
          "tesseract_collision: Failed to locate a tesseract_geometry target (expected either "
          "tesseract::tesseract_geometry, tesseract_geometry::tesseract_geometry, or tesseract_geometry)")
else()
  if("${TESSERACT_GEOM_TARGET}" STREQUAL "tesseract::tesseract_geometry")
    set(_tesseract_geometry_namespace "tesseract::")
  elseif("${TESSERACT_GEOM_TARGET}" STREQUAL "tesseract_geometry::tesseract_geometry")
    set(_tesseract_geometry_namespace "tesseract_geometry::")
  else()
    set(_tesseract_geometry_namespace "${TESSERACT_GEOM_TARGET}")
  endif()
  message(
    STATUS
      "tesseract_collision: using ${TESSERACT_GEOM_TARGET} for tesseract_geometry exports (namespace ${_tesseract_geometry_namespace})")
endif()

# These targets are necessary for 16.04 builds. Remove when Kinetic support is dropped
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

if(NOT TARGET octomap)
  add_library(octomap INTERFACE IMPORTED)
  set_target_properties(octomap PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${OCTOMAP_INCLUDE_DIRS}")
  set_target_properties(octomap PROPERTIES INTERFACE_LINK_LIBRARIES "${OCTOMAP_LIBRARIES}")
endif()
if(NOT TARGET octomath)
  add_library(octomath INTERFACE IMPORTED)
  set_target_properties(octomath PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${OCTOMAP_INCLUDE_DIRS}")
  set_target_properties(octomath PROPERTIES INTERFACE_LINK_LIBRARIES "${OCTOMAP_LIBRARIES}")
endif()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/test/*
    ${CMAKE_CURRENT_LIST_DIR}/include/tesseract_collision/test_suite/*
    ${CMAKE_CURRENT_LIST_DIR}/include/tesseract_collision/vhacd/*
    /*/install/*
    /*/devel/*
    /*/gtest/*
    /*/bullet/LinearMath/*
    /*/bullet/BulletCollision/*
    /*/include/ccd/*
    /*/include/fcl/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Define compile-time default variables
if(MSVC)
  set(TESSERACT_CONTACT_MANAGERS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/bin)
else()
  set(TESSERACT_CONTACT_MANAGERS_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/lib)
endif()

# Variable for kinematics plugins
set(CONTACT_MANAGERS_PLUGINS "")

# Core
set(SUPPORTED_COMPONENTS core)
add_subdirectory(core)

# Bullet (currently required for creation of convex hulls)
list(APPEND SUPPORTED_COMPONENTS bullet)
add_subdirectory(bullet)

# Test Suite
option(TESSERACT_BUILD_TEST_SUITE "Build Test Suite components" ON)
if(TESSERACT_BUILD_TEST_SUITE)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test_suite/CMakeLists.txt")
    message("Building Test Suite components")
    list(APPEND SUPPORTED_COMPONENTS test_suite)
    add_subdirectory(test_suite)
  else()
    message(WARNING "Test Suite components requested but test_suite directory is missing; skipping build")
  endif()
endif()

# FCL
option(TESSERACT_BUILD_FCL "Build FCL components" ON)
if(TESSERACT_BUILD_FCL)
  message("Building FCL components")
  list(APPEND SUPPORTED_COMPONENTS fcl)
  add_subdirectory(fcl)
endif()

# VHACD
option(TESSERACT_BUILD_VHACD "Build VHACD components" ON)
if(TESSERACT_BUILD_VHACD)
  message("Building VHACD components")
  list(APPEND SUPPORTED_COMPONENTS vhacd)
  add_subdirectory(vhacd)
endif()

# Add compiler definition to core so it can find all plugins produced
string(
  REPLACE ";"
          ":"
          CONTACT_MANAGERS_PLUGINS_STRING
          "${CONTACT_MANAGERS_PLUGINS}")
target_compile_definitions(${PROJECT_NAME}_core
                           PRIVATE TESSERACT_CONTACT_MANAGERS_PLUGINS="${CONTACT_MANAGERS_PLUGINS_STRING}")

# Testing
if((TESSERACT_ENABLE_TESTING OR TESSERACT_COLLISION_ENABLE_TESTING) AND TESSERACT_BUILD_FCL)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Benchmarks
if((TESSERACT_ENABLE_BENCHMARKING OR TESSERACT_COLLISION_ENABLE_BENCHMARKING) AND TESSERACT_BUILD_FCL
   AND TESSERACT_BUILD_TEST_SUITE)
  add_subdirectory(test/benchmarks)
endif()

# Examples
if((TESSERACT_ENABLE_EXAMPLES OR TESSERACT_COLLISION_ENABLE_EXAMPLES) AND TESSERACT_BUILD_FCL)
  message("Building examples components")
  list(APPEND SUPPORTED_COMPONENTS examples)
  add_subdirectory(examples)
endif()

configure_package(COMPONENT core SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS})

if(TESSERACT_PACKAGE)
  cpack_component_package(
    VERSION ${pkg_extracted_version}
    MAINTAINER_NAME ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    COMPONENT_DEPENDS ${SUPPORTED_COMPONENTS})

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
