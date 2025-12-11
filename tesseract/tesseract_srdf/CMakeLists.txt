cmake_minimum_required(VERSION 3.15.0)

include("${CMAKE_CURRENT_LIST_DIR}/../tesseract_common/cmake/tesseract_cmake_boilerplate.cmake")
tesseract_initialize_boilerplate_support()
if(NOT COMMAND extract_package_metadata)
  message(FATAL_ERROR "Failed to load tesseract_cmake_boilerplate.cmake; extract_package_metadata is unavailable")
endif()

# Extract package name and version
extract_package_metadata(pkg PACKAGE_XML "${CMAKE_CURRENT_LIST_DIR}/package.xml")
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/"
                             "${CMAKE_CURRENT_LIST_DIR}/../tesseract_common/cmake/")

include("${CMAKE_CURRENT_LIST_DIR}/../tesseract_common/cmake/tesseract_macros.cmake")

set(_tesseract_srdf_boost_components serialization stacktrace_noop)
if(NOT APPLE)
  list(APPEND _tesseract_srdf_boost_components stacktrace_backtrace stacktrace_basic)
endif()
find_package(Boost REQUIRED COMPONENTS ${_tesseract_srdf_boost_components})
find_package(TinyXML2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_scene_graph REQUIRED)
find_package(yaml-cpp REQUIRED)

tesseract_resolve_tinyxml2(TESSERACT_TINYXML2_TARGET TESSERACT_TINYXML2_LINK_LIBRARIES)
if(NOT TESSERACT_TINYXML2_TARGET AND NOT TESSERACT_TINYXML2_LINK_LIBRARIES)
  message(FATAL_ERROR "Failed to locate a TinyXML2 library for tesseract_srdf")
endif()

if(TARGET Boost::stacktrace_backtrace)
  set(TESSERACT_SRDF_BACKTRACE_COMPONENT stacktrace_backtrace)
  set(TESSERACT_SRDF_BACKTRACE_LIB Boost::stacktrace_backtrace)
elseif(TARGET Boost::stacktrace_basic)
  set(TESSERACT_SRDF_BACKTRACE_COMPONENT stacktrace_basic)
  set(TESSERACT_SRDF_BACKTRACE_LIB Boost::stacktrace_basic)
else()
  set(TESSERACT_SRDF_BACKTRACE_COMPONENT stacktrace_noop)
  set(TESSERACT_SRDF_BACKTRACE_LIB Boost::stacktrace_noop)
endif()

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

add_library(
  ${PROJECT_NAME}
  src/collision_margins.cpp
  src/configs.cpp
  src/disabled_collisions.cpp
  src/group_states.cpp
  src/group_tool_center_points.cpp
  src/groups.cpp
  src/kinematics_information.cpp
  src/srdf_model.cpp
  src/utils.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
         tesseract::tesseract_common
         tesseract::tesseract_scene_graph
         yaml-cpp
         Boost::serialization
         ${TESSERACT_TINYXML2_TARGET}
         ${TESSERACT_TINYXML2_LINK_LIBRARIES})
target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE Boost::boost
          ${TESSERACT_SRDF_BACKTRACE_LIB}
          console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

configure_package(
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}
  DEPENDENCIES
    TinyXML2
    Boost COMPONENTS serialization ${TESSERACT_SRDF_BACKTRACE_COMPONENT}
    Eigen3
    console_bridge
    tesseract_common
    tesseract_scene_graph
    yaml-cpp
  CFG_EXTRAS cmake/tesseract_srdf-extras.cmake)

# Mark cpp header files for installation
install(
  FILES include/${PROJECT_NAME}/srdf_model.h
        include/${PROJECT_NAME}/kinematics_information.h
        include/${PROJECT_NAME}/utils.h
        include/${PROJECT_NAME}/fwd.h
  DESTINATION include/${PROJECT_NAME})

# ADD Examples
if(TESSERACT_ENABLE_EXAMPLES OR TESSERACT_SRDF_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

if(TESSERACT_ENABLE_TESTING OR TESSERACT_SRDF_ENABLE_TESTING)
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
      "libyaml-cpp-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
    WINDOWS_DEPENDS
      "console-bridge"
      "Eigen3"
      "yaml-cpp"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
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
