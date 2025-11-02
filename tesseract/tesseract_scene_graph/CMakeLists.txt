cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(Boost REQUIRED COMPONENTS graph)
find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)

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

set(_tesseract_geometry_candidate_targets
    tesseract::tesseract_geometry
    tesseract_geometry::tesseract_geometry
    tesseract_geometry)
foreach(_tesseract_geometry_target ${_tesseract_geometry_candidate_targets})
  if(TARGET ${_tesseract_geometry_target})
    set(TESSERACT_GEOMETRY_TARGET ${_tesseract_geometry_target})
    break()
  endif()
endforeach()

if(NOT TESSERACT_GEOMETRY_TARGET)
  set(_tesseract_geometry_imported_target tesseract::tesseract_geometry)
  if(TARGET ${_tesseract_geometry_imported_target})
    set(TESSERACT_GEOMETRY_TARGET ${_tesseract_geometry_imported_target})
  else()
    add_library(${_tesseract_geometry_imported_target} INTERFACE IMPORTED)
    if(tesseract_geometry_INCLUDE_DIRS)
      set_target_properties(
        ${_tesseract_geometry_imported_target}
        PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${tesseract_geometry_INCLUDE_DIRS}")
    endif()
    if(tesseract_geometry_LIBRARIES)
      set_target_properties(
        ${_tesseract_geometry_imported_target}
        PROPERTIES INTERFACE_LINK_LIBRARIES "${tesseract_geometry_LIBRARIES}")
    endif()
    if(tesseract_geometry_DEFINITIONS)
      set_target_properties(
        ${_tesseract_geometry_imported_target}
        PROPERTIES INTERFACE_COMPILE_DEFINITIONS "${tesseract_geometry_DEFINITIONS}")
    endif()
    if(tesseract_geometry_COMPILE_OPTIONS)
      set_target_properties(
        ${_tesseract_geometry_imported_target}
        PROPERTIES INTERFACE_COMPILE_OPTIONS "${tesseract_geometry_COMPILE_OPTIONS}")
    endif()
    set(TESSERACT_GEOMETRY_TARGET ${_tesseract_geometry_imported_target})
  endif()
endif()

if(NOT TESSERACT_GEOMETRY_TARGET)
  message(
    FATAL_ERROR
      "Unable to locate a tesseract_geometry target. Expected one of: tesseract::tesseract_geometry, tesseract_geometry::tesseract_geometry, tesseract_geometry.")
endif()

set(_tesseract_common_candidate_targets
    tesseract::tesseract_common
    tesseract_common::tesseract_common
    tesseract_common)
foreach(_tesseract_common_target ${_tesseract_common_candidate_targets})
  if(TARGET ${_tesseract_common_target})
    set(TESSERACT_COMMON_TARGET ${_tesseract_common_target})
    break()
  endif()
endforeach()

if(NOT TESSERACT_COMMON_TARGET)
  set(_tesseract_common_imported_target tesseract::tesseract_common)
  if(TARGET ${_tesseract_common_imported_target})
    set(TESSERACT_COMMON_TARGET ${_tesseract_common_imported_target})
  else()
    add_library(${_tesseract_common_imported_target} INTERFACE IMPORTED)
    if(tesseract_common_INCLUDE_DIRS)
      set_target_properties(
        ${_tesseract_common_imported_target}
        PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${tesseract_common_INCLUDE_DIRS}")
    endif()
    if(tesseract_common_LIBRARIES)
      set_target_properties(
        ${_tesseract_common_imported_target}
        PROPERTIES INTERFACE_LINK_LIBRARIES "${tesseract_common_LIBRARIES}")
    endif()
    if(tesseract_common_DEFINITIONS)
      set_target_properties(
        ${_tesseract_common_imported_target}
        PROPERTIES INTERFACE_COMPILE_DEFINITIONS "${tesseract_common_DEFINITIONS}")
    endif()
    if(tesseract_common_COMPILE_OPTIONS)
      set_target_properties(
        ${_tesseract_common_imported_target}
        PROPERTIES INTERFACE_COMPILE_OPTIONS "${tesseract_common_COMPILE_OPTIONS}")
    endif()
    set(TESSERACT_COMMON_TARGET ${_tesseract_common_imported_target})
  endif()
endif()

if(NOT TESSERACT_COMMON_TARGET)
  message(
    FATAL_ERROR
      "Unable to locate a tesseract_common target. Expected one of: tesseract::tesseract_common, tesseract_common::tesseract_common, tesseract_common.")
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
    /*/gtest/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

add_library(
  ${PROJECT_NAME}
  src/graph.cpp
  src/joint.cpp
  src/kdl_parser.cpp
  src/link.cpp
  src/scene_state.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Boost::boost
         Boost::graph
         Eigen3::Eigen
         ${TESSERACT_GEOMETRY_TARGET}
         ${TESSERACT_COMMON_TARGET}
         orocos-kdl
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(
  ${PROJECT_NAME}
  PUBLIC ${TESSERACT_COMPILE_DEFINITIONS}
         Boost_USE_STATIC_LIBS=OFF
         Boost_USE_MULTITHREADED=ON
         Boost_USE_STATIC_RUNTIME=OFF)
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
    "Boost COMPONENTS graph"
    Eigen3
    console_bridge
    orocos_kdl
    tesseract_geometry
    tesseract_common
  CFG_EXTRAS cmake/tesseract_scene_graph-extras.cmake)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

# ADD Examples
if(TESSERACT_ENABLE_EXAMPLES OR TESSERACT_SCENE_GRAPH_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

if(TESSERACT_ENABLE_TESTING OR TESSERACT_SCENE_GRAPH_ENABLE_TESTING)
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
      "libboost-all-dev"
      "liborocos-kdl-dev"
      "libeigen3-dev"
      "libconsole-bridge-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry"
    WINDOWS_DEPENDS
      "boost-all"
      "orocos-kdl"
      "Eigen3"
      "console-bridge"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
