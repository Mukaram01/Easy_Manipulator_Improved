cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# All of these are used by all components so to avoid having them in each they are include her
find_package(console_bridge REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(rclcpp QUIET)
find_package(OpenMP REQUIRED)
find_package(cereal QUIET)
if(NOT cereal_FOUND)
  find_package(cereal_vendor QUIET)
  if(cereal_vendor_FOUND)
    find_package(cereal REQUIRED)
  endif()
endif()

option(TESSERACT_DISABLE_CEREAL_FETCH "Disable fetching cereal with FetchContent fallback" ON)
add_compile_definitions(TESSERACT_DISABLE_CEREAL_FETCH=$<BOOL:${TESSERACT_DISABLE_CEREAL_FETCH}>)

if(NOT cereal_FOUND)
  set(_cereal_hint "Install the libcereal-dev package or the ROS cereal_vendor package and retry. FetchContent support was removed to avoid recursive downloads during configure.")
  if(TESSERACT_DISABLE_CEREAL_FETCH)
    message(FATAL_ERROR "cereal not found and TESSERACT_DISABLE_CEREAL_FETCH is ON. ${_cereal_hint}")
  else()
    message(FATAL_ERROR "cereal not found. ${_cereal_hint}")
  endif()
endif()

if(cereal_FOUND)
  # Rewrite cereal include directories to guard against vendor packages exporting /include
  get_target_property(_cereal_include_dirs cereal::cereal INTERFACE_INCLUDE_DIRECTORIES)
  set(_cereal_valid_include_dirs "")
  foreach(_cereal_include_dir ${_cereal_include_dirs})
    string(GENEX_STRIP "${_cereal_include_dir}" _cereal_include_dir_stripped)
    if(EXISTS "${_cereal_include_dir_stripped}")
      list(APPEND _cereal_valid_include_dirs "${_cereal_include_dir_stripped}")
    endif()
  endforeach()

  set(_cereal_pruned_include_dirs "${_cereal_include_dirs}")
  foreach(_cereal_valid_include_dir ${_cereal_valid_include_dirs})
    list(REMOVE_ITEM _cereal_pruned_include_dirs "${_cereal_valid_include_dir}")
  endforeach()

  if(_cereal_pruned_include_dirs)
    message(STATUS "Pruning cereal include directories that do not exist: ${_cereal_pruned_include_dirs}")
  endif()

  if(NOT _cereal_valid_include_dirs)
    find_path(_cereal_header_path
              NAMES cereal/cereal.hpp
              PATHS ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES}
              NO_CACHE)
    if(_cereal_header_path)
      list(APPEND _cereal_valid_include_dirs "${_cereal_header_path}")
      message(STATUS "cereal headers found in implicit include path: ${_cereal_header_path}")
    endif()
  endif()

  if(NOT _cereal_valid_include_dirs)
    set(_cereal_hint "Install a proper cereal package (e.g., libcereal-dev or the ROS cereal_vendor) or toggle TESSERACT_DISABLE_CEREAL_FETCH accordingly.")
    message(FATAL_ERROR "cereal::cereal target exports no valid include directories or discoverable headers (found: ${_cereal_include_dirs}). ${_cereal_hint}")
  endif()

  if(_cereal_valid_include_dirs)
    set_target_properties(cereal::cereal PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_cereal_valid_include_dirs}")
  endif()
endif()

include(${tesseract_common_DIR}/tesseract_macros.cmake)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/core/test/*
    ${CMAKE_CURRENT_LIST_DIR}/descartes/test/*
    ${CMAKE_CURRENT_LIST_DIR}/ompl/test/*
    ${CMAKE_CURRENT_LIST_DIR}/simple/test/*
    ${CMAKE_CURRENT_LIST_DIR}/trajopt/test/*
    ${CMAKE_CURRENT_LIST_DIR}/trajopt_ifopt/test/*
    /*/install/*
    /*/devel/*
    /*/gtest/*
    /*/include/descartes_light/*
    /*/include/opw_kinematics/*
    /*/include/trajopt/*
    /*/include/trajopt_sco/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Enable Testing
if(TESSERACT_ENABLE_TESTING)
  find_gtest()
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
endif()

# Core
set(SUPPORTED_COMPONENTS core)
add_subdirectory(core)

# Simple
list(APPEND SUPPORTED_COMPONENTS simple)
add_subdirectory(simple)

# OMPL
option(TESSERACT_BUILD_OMPL "Build the OMPL planner" ON)
if(TESSERACT_BUILD_OMPL)
  message("Building OMPL Planner")
  list(APPEND SUPPORTED_COMPONENTS ompl)
  add_subdirectory(ompl)
endif()

# Descartes
option(TESSERACT_BUILD_DESCARTES "Build the Descartes planner" ON)
if(TESSERACT_BUILD_DESCARTES)
  message("Building Descartes Planner")
  list(APPEND SUPPORTED_COMPONENTS descartes)
  add_subdirectory(descartes)
endif()

# Trajopt
set(TESSERACT_TRAJOPT_AVAILABLE ON)
if(NOT rclcpp_FOUND OR NOT OpenMP_FOUND)
  set(TESSERACT_TRAJOPT_AVAILABLE OFF)
endif()
set(TESSERACT_BUILD_TRAJOPT_DESCRIPTION "Build the Trajopt planner")
option(TESSERACT_BUILD_TRAJOPT "${TESSERACT_BUILD_TRAJOPT_DESCRIPTION}" ${TESSERACT_TRAJOPT_AVAILABLE})
if(TESSERACT_BUILD_TRAJOPT)
  if(NOT rclcpp_FOUND)
    message(FATAL_ERROR "Trajopt planner requires rclcpp, but it was not found. Please install rclcpp or disable TESSERACT_BUILD_TRAJOPT.")
  endif()
  if(NOT OpenMP_FOUND)
    message(FATAL_ERROR "Trajopt planner requires OpenMP, but it was not found. Please install OpenMP or disable TESSERACT_BUILD_TRAJOPT.")
  endif()

  message("Building Trajopt Planner")
  list(APPEND SUPPORTED_COMPONENTS trajopt)
  add_subdirectory(trajopt)
endif()

# Trajopt IFOPT
set(TESSERACT_TRAJOPT_IFOPT_AVAILABLE OFF)
find_package(trajopt_ifopt QUIET)
find_package(trajopt_sqp QUIET)
if(trajopt_ifopt_FOUND AND trajopt_sqp_FOUND)
  set(TESSERACT_TRAJOPT_IFOPT_AVAILABLE ON)
endif()

set(TESSERACT_BUILD_TRAJOPT_IFOPT_DESCRIPTION "Build the Trajopt IFOPT planner")
option(TESSERACT_BUILD_TRAJOPT_IFOPT "${TESSERACT_BUILD_TRAJOPT_IFOPT_DESCRIPTION}" ${TESSERACT_TRAJOPT_IFOPT_AVAILABLE})

if(TESSERACT_BUILD_TRAJOPT_IFOPT AND NOT TESSERACT_TRAJOPT_IFOPT_AVAILABLE)
  message(WARNING "trajopt_ifopt and/or trajopt_sqp not available. Disabling Trajopt IFOPT planner.")
  set(TESSERACT_BUILD_TRAJOPT_IFOPT OFF CACHE BOOL "${TESSERACT_BUILD_TRAJOPT_IFOPT_DESCRIPTION}" FORCE)
endif()

if(TESSERACT_BUILD_TRAJOPT_IFOPT)
  message("Building Trajopt IFOPT Planner")
  list(APPEND SUPPORTED_COMPONENTS trajopt_ifopt)
  add_subdirectory(trajopt_ifopt)
endif()

# Examples
if(TESSERACT_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

# Package configuration
set(_${PROJECT_NAME}_component_paths "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-component-paths.cmake")
file(WRITE ${_${PROJECT_NAME}_component_paths} "# Component package hints for ${PROJECT_NAME}\n")
foreach(_component IN LISTS SUPPORTED_COMPONENTS)
  file(APPEND
       ${_${PROJECT_NAME}_component_paths}
       "set(${PROJECT_NAME}_${_component}_DIR \"${CMAKE_CURRENT_LIST_DIR}\")\n")
endforeach()

configure_package(
  COMPONENT core
  SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS}
  CFG_EXTRAS ${_${PROJECT_NAME}_component_paths})

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
