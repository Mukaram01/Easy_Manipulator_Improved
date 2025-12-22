cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(console_bridge REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(boost_plugin_loader REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cereal QUIET)
find_package(tinyxml2 CONFIG QUIET)
if(NOT tinyxml2_FOUND)
  message(FATAL_ERROR "Failed to find the tinyxml2 CONFIG package. Please install libtinyxml2-dev.")
endif()

if(TARGET TinyXML2::TinyXML2 AND NOT TARGET tinyxml2::tinyxml2)
  add_library(tinyxml2::tinyxml2 ALIAS TinyXML2::TinyXML2)
endif()

if(NOT TARGET tinyxml2::tinyxml2 AND NOT TARGET TinyXML2::TinyXML2)
  message(FATAL_ERROR "tinyxml2 target not available. Please install libtinyxml2-dev.")
endif()

find_package(Boost REQUIRED COMPONENTS stacktrace_backtrace)
set(BOOST_STACKTRACE_TARGET "")
foreach(_stacktrace_component IN ITEMS stacktrace_backtrace stacktrace_addr2line stacktrace_basic stacktrace_noop)
  if(TARGET "Boost::${_stacktrace_component}")
    set(BOOST_STACKTRACE_TARGET "Boost::${_stacktrace_component}")
    break()
  endif()
endforeach()

if(NOT BOOST_STACKTRACE_TARGET)
  message(
    FATAL_ERROR
      "Failed to find a Boost stacktrace target (checked stacktrace_backtrace, stacktrace_addr2line, stacktrace_basic, stacktrace_noop). "
      "Please ensure Boost stacktrace components are installed.")
elseif(NOT TARGET Boost::stacktrace_backtrace)
  add_library(Boost::stacktrace_backtrace ALIAS ${BOOST_STACKTRACE_TARGET})
endif()

find_path(CEREAL_INCLUDE_DIR NAMES cereal/cereal.hpp)
if(NOT CEREAL_INCLUDE_DIR)
  message(FATAL_ERROR "Failed to find cereal headers. Please install libcereal-dev.")
endif()

if(TARGET cereal::cereal)
  get_target_property(_cereal_interface_dirs cereal::cereal INTERFACE_INCLUDE_DIRECTORIES)
  set(_cereal_reset_interface FALSE)
  if(NOT _cereal_interface_dirs)
    set(_cereal_reset_interface TRUE)
  else()
    foreach(_cereal_dir IN LISTS _cereal_interface_dirs)
      if(NOT EXISTS "${_cereal_dir}")
        set(_cereal_reset_interface TRUE)
        break()
      endif()
    endforeach()
  endif()

  if(_cereal_reset_interface)
    set_property(TARGET cereal::cereal PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${CEREAL_INCLUDE_DIR}")
  endif()
else()
  add_library(tesseract_task_composer_cereal INTERFACE)
  target_include_directories(tesseract_task_composer_cereal INTERFACE "${CEREAL_INCLUDE_DIR}")
  add_library(cereal::cereal ALIAS tesseract_task_composer_cereal)
endif()

# Import shared Tesseract CMake utilities (clang-tidy args, compiler options, C++ standard, etc.)
include(${tesseract_common_DIR}/tesseract_macros.cmake)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/test/*
    ${CMAKE_CURRENT_LIST_DIR}/core/include/tesseract_task_composer/core/test_suite/*
    /*/install/*
    /*/devel/*
    /*/gtest/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Define compile-time default variables
if(MSVC)
  set(TESSERACT_TASK_COMPOSER_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/bin)
else()
  set(TESSERACT_TASK_COMPOSER_PLUGIN_PATH ${CMAKE_INSTALL_PREFIX}/lib)
endif()

# Variable for task composer plugins
set(TASK_COMPOSER_PLUGINS "")

# Core
set(SUPPORTED_COMPONENTS core)
add_subdirectory(core)

# Planning Tasks
option(TESSERACT_BUILD_TASK_COMPOSER_PLANNING "Build task composer planning components" ON)
if(TESSERACT_BUILD_TASK_COMPOSER_PLANNING)
  message("Build task composer planning components")
  list(APPEND SUPPORTED_COMPONENTS planning)
  add_subdirectory(planning)
endif()

# Taskflow Executor
option(TESSERACT_BUILD_TASK_COMPOSER_TASKFLOW "Build task composer taskflow components" ON)
if(TESSERACT_BUILD_TASK_COMPOSER_TASKFLOW)
  message("Build task composer taskflow components")
  list(APPEND SUPPORTED_COMPONENTS taskflow)
  add_subdirectory(taskflow)
endif()

# Add compiler definition to core so it can find all plugins produced
string(
  REPLACE ";"
          ":"
          TASK_COMPOSER_PLUGINS_STRING
          "${TASK_COMPOSER_PLUGINS}")
target_compile_definitions(${PROJECT_NAME} PRIVATE TESSERACT_TASK_COMPOSER_PLUGINS="${TASK_COMPOSER_PLUGINS_STRING}")

# Examples
if(TESSERACT_ENABLE_EXAMPLES)
  message("Build task composer examples components")
  list(APPEND SUPPORTED_COMPONENTS examples)
  add_subdirectory(examples)
endif()

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config COMPONENT core)

# Testing
if(TESSERACT_ENABLE_TESTING)
  find_gtest()
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Package configuration
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
