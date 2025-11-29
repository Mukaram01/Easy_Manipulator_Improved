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

include(${CMAKE_CURRENT_LIST_DIR}/../tesseract_common/cmake/tesseract_macros.cmake)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../tesseract_common/cmake")

set(_tesseract_scene_graph_boost_components serialization system filesystem stacktrace_backtrace stacktrace_noop)
find_package(Boost REQUIRED COMPONENTS ${_tesseract_scene_graph_boost_components})

# Boost 1.83 packages on Debian/Ubuntu split each component into its own
# optional package.  The "graph" component is not always installed in ROS-free
# environments, even though the headers ship with the base Boost development
# package.  The upstream Tesseract sources rely only on the header-only pieces
# of Boost.Graph, so fall back to a lightweight imported interface when the
# compiled component is unavailable.  This avoids hard build failures while
# still allowing downstream users to provide the full Boost::graph target when
# desired.
set(_tesseract_scene_graph_have_boost_graph FALSE)
if(TARGET Boost::graph)
  set(_tesseract_scene_graph_have_boost_graph TRUE)
else()
  find_package(Boost QUIET COMPONENTS graph)
  if(TARGET Boost::graph)
    set(_tesseract_scene_graph_have_boost_graph TRUE)
  endif()
endif()

if(NOT _tesseract_scene_graph_have_boost_graph)
  message(
    STATUS
      "Boost::graph component not found; providing a header-only interface target")
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
find_package(orocos_kdl QUIET)
set(TESSERACT_SCENE_GRAPH_HAS_KDL ${orocos_kdl_FOUND})
if(NOT TESSERACT_SCENE_GRAPH_HAS_KDL)
  message(STATUS "orocos_kdl not found; disabling KDL parser helpers")
endif()
find_package(assimp REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(tinyxml2 REQUIRED tinyxml2)

set(_tesseract_geometry_candidate_targets
    tesseract_geometry::tesseract_geometry
    tesseract::tesseract_geometry
    tesseract_geometry)
foreach(_tesseract_geometry_target ${_tesseract_geometry_candidate_targets})
  if(TARGET ${_tesseract_geometry_target})
    set(TESSERACT_GEOMETRY_TARGET ${_tesseract_geometry_target})
    break()
  endif()
endforeach()

if(NOT TESSERACT_GEOMETRY_TARGET)
  set(_tesseract_geometry_imported_target tesseract_geometry::tesseract_geometry)
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
      "Unable to locate a tesseract_geometry target. Expected one of: tesseract_geometry::tesseract_geometry, tesseract::tesseract_geometry, tesseract_geometry.")
endif()

set(_tesseract_common_candidate_targets
    tesseract_common::tesseract_common
    tesseract::tesseract_common
    tesseract_common)
foreach(_tesseract_common_target ${_tesseract_common_candidate_targets})
  if(TARGET ${_tesseract_common_target})
    set(TESSERACT_COMMON_TARGET ${_tesseract_common_target})
    break()
  endif()
endforeach()

if(NOT TESSERACT_COMMON_TARGET)
  set(_tesseract_common_imported_target tesseract_common::tesseract_common)
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
      "Unable to locate a tesseract_common target. Expected one of: tesseract_common::tesseract_common, tesseract::tesseract_common, tesseract_common.")
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
    /*/gtest/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

set(
  _tesseract_scene_graph_sources
  src/graph.cpp
  src/joint.cpp
  src/link.cpp
  src/scene_state.cpp)

if(TESSERACT_SCENE_GRAPH_HAS_KDL)
  list(APPEND _tesseract_scene_graph_sources src/kdl_parser.cpp)
endif()

add_library(${PROJECT_NAME} ${_tesseract_scene_graph_sources})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Boost::boost
         Boost::graph
         Eigen3::Eigen
         ${tinyxml2_LIBRARIES}
         ${TESSERACT_GEOMETRY_TARGET}
         ${TESSERACT_COMMON_TARGET}
         console_bridge::console_bridge)
if(TESSERACT_SCENE_GRAPH_HAS_KDL)
  target_link_libraries(${PROJECT_NAME} PUBLIC orocos-kdl)
endif()
target_compile_options(${PROJECT_NAME} PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(
  ${PROJECT_NAME}
  PUBLIC ${TESSERACT_COMPILE_DEFINITIONS}
         Boost_USE_STATIC_LIBS=OFF
         Boost_USE_MULTITHREADED=ON
         Boost_USE_STATIC_RUNTIME=OFF)
if(TESSERACT_SCENE_GRAPH_HAS_KDL)
  target_compile_definitions(${PROJECT_NAME} PUBLIC TESSERACT_SCENE_GRAPH_HAS_KDL=1)
else()
  target_compile_definitions(${PROJECT_NAME} PUBLIC TESSERACT_SCENE_GRAPH_HAS_KDL=0)
endif()
target_clang_tidy(${PROJECT_NAME} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>"
                                                  ${tinyxml2_INCLUDE_DIRS})

set(_tesseract_scene_graph_dependencies
    Boost
    Eigen3
    PkgConfig
    tinyxml2
    console_bridge
    assimp
    tesseract_geometry
    tesseract_common)
if(TESSERACT_SCENE_GRAPH_HAS_KDL)
  list(APPEND _tesseract_scene_graph_dependencies orocos_kdl)
endif()

configure_package(
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}
  DEPENDENCIES ${_tesseract_scene_graph_dependencies}
  CFG_EXTRAS cmake/tesseract_scene_graph-extras.cmake)

# Colcon does not automatically extend the CMAKE_PREFIX_PATH for plain CMake
# packages.  Install the standard environment hook so downstream packages such
# as tesseract_urdf can locate the exported package configuration files without
# additional manual configuration.
set(_hook_directory share/${PROJECT_NAME}/hook)
configure_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/cmake_prefix_path.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_cmake_prefix_path.sh
  @ONLY)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_cmake_prefix_path.sh
  DESTINATION ${_hook_directory}
  RENAME cmake_prefix_path.sh)

configure_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/cmake_prefix_path.dsv.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_cmake_prefix_path.dsv
  @ONLY)
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_cmake_prefix_path.dsv
  DESTINATION ${_hook_directory}
  RENAME cmake_prefix_path.dsv)

# Mark cpp header files for installation
set(_tesseract_scene_graph_install_patterns PATTERN "*.h" PATTERN ".svn" EXCLUDE)
if(NOT TESSERACT_SCENE_GRAPH_HAS_KDL)
  list(APPEND _tesseract_scene_graph_install_patterns PATTERN "kdl_parser.h" EXCLUDE)
endif()

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  ${_tesseract_scene_graph_install_patterns})

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
  set(_tesseract_scene_graph_linux_deps
      "libboost-all-dev"
      "libeigen3-dev"
      "libconsole-bridge-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry")
  set(_tesseract_scene_graph_windows_deps
      "boost-all"
      "Eigen3"
      "console-bridge"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-geometry")
  if(TESSERACT_SCENE_GRAPH_HAS_KDL)
    list(APPEND _tesseract_scene_graph_linux_deps "liborocos-kdl-dev")
    list(APPEND _tesseract_scene_graph_windows_deps "orocos-kdl")
  endif()

  cpack(
    VERSION ${pkg_extracted_version}
    MAINTAINER_NAME ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    LINUX_DEPENDS ${_tesseract_scene_graph_linux_deps}
    WINDOWS_DEPENDS ${_tesseract_scene_graph_windows_deps})

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
