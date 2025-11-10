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

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

include("${CMAKE_CURRENT_LIST_DIR}/../tesseract_common/cmake/tesseract_macros.cmake")

find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(octomap REQUIRED)
find_package(assimp REQUIRED)
if(NOT APPLE)
  find_package(
    Boost REQUIRED
    COMPONENTS system
               filesystem
               serialization
               stacktrace_noop
    OPTIONAL_COMPONENTS stacktrace_backtrace stacktrace_basic)
else()
  find_package(
    Boost REQUIRED
    COMPONENTS system
               filesystem
               serialization
               stacktrace_noop)
endif()
tesseract_resolve_tinyxml2(TESSERACT_TINYXML2_TARGET TESSERACT_TINYXML2_LINK_LIBRARIES)
if(NOT TESSERACT_TINYXML2_TARGET AND NOT TESSERACT_TINYXML2_LINK_LIBRARIES)
  message(FATAL_ERROR "Failed to locate a TinyXML2 library for tesseract_geometry")
endif()

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES
                        INTERFACE_INCLUDE_DIRECTORIES ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES
                        ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES
                          INTERFACE_INCLUDE_DIRECTORIES ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

# These targets are necessary for 16.04 builds. Remove when Kinetic support is dropped
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

find_file(ASSIMP_PBR_MATERIAL_FOUND assimp/pbrmaterial.h HINTS ${assimp_INCLUDE_DIRS})

if(ASSIMP_PBR_MATERIAL_FOUND)
  set(TESSERACT_ASSIMP_USE_PBRMATERIAL TESSERACT_ASSIMP_USE_PBRMATERIAL=1)
else()
  set(TESSERACT_ASSIMP_USE_PBRMATERIAL)
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

find_package(tesseract_common REQUIRED)

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
      set_target_properties(${_tesseract_common_imported_target} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                            "${tesseract_common_INCLUDE_DIRS}")
    endif()
    if(tesseract_common_LIBRARIES)
      set_target_properties(${_tesseract_common_imported_target} PROPERTIES INTERFACE_LINK_LIBRARIES
                            "${tesseract_common_LIBRARIES}")
    endif()
    if(tesseract_common_DEFINITIONS)
      set_target_properties(${_tesseract_common_imported_target} PROPERTIES INTERFACE_COMPILE_DEFINITIONS
                            "${tesseract_common_DEFINITIONS}")
    endif()
    if(tesseract_common_COMPILE_OPTIONS)
      set_target_properties(${_tesseract_common_imported_target} PROPERTIES INTERFACE_COMPILE_OPTIONS
                            "${tesseract_common_COMPILE_OPTIONS}")
    endif()
    set(TESSERACT_COMMON_TARGET ${_tesseract_common_imported_target})
  endif()
endif()

if(NOT TESSERACT_COMMON_TARGET)
  message(FATAL_ERROR
          "Unable to locate a tesseract_common target. Expected one of: tesseract_common::tesseract_common, tesseract::tesseract_common, tesseract_common.")
endif()

add_library(
  ${PROJECT_NAME} SHARED
  src/conversions.cpp
  src/geometry.cpp
  src/utils.cpp
  src/geometries/box.cpp
  src/geometries/capsule.cpp
  src/geometries/compound_mesh.cpp
  src/geometries/cone.cpp
  src/geometries/convex_mesh.cpp
  src/geometries/cylinder.cpp
  src/geometries/mesh_material.cpp
  src/geometries/mesh.cpp
  src/geometries/octree.cpp
  src/geometries/plane.cpp
  src/geometries/polygon_mesh.cpp
  src/geometries/sdf_mesh.cpp
  src/geometries/sphere.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    ${TESSERACT_COMMON_TARGET}
    console_bridge::console_bridge
    octomap
    octomath
    assimp::assimp
    Boost::serialization
    ${TESSERACT_TINYXML2_TARGET}
    ${TESSERACT_TINYXML2_LINK_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_DEFINITIONS} ${TESSERACT_ASSIMP_USE_PBRMATERIAL})
target_clang_tidy(${PROJECT_NAME} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(${PROJECT_NAME} PRIVATE ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}
                           PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                  "$<INSTALL_INTERFACE:include>")

configure_package(NAMESPACE tesseract TARGETS ${PROJECT_NAME})

if(NOT TARGET tesseract::tesseract_common AND TARGET tesseract_common::tesseract_common)
  add_library(tesseract::tesseract_common INTERFACE IMPORTED)
  set_target_properties(tesseract::tesseract_common PROPERTIES INTERFACE_LINK_LIBRARIES
                        tesseract_common::tesseract_common)
endif()

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

install(FILES "${CMAKE_CURRENT_LIST_DIR}/cmake/Findassimp.cmake" DESTINATION lib/cmake/${PROJECT_NAME})

# ADD Examples
if(TESSERACT_ENABLE_EXAMPLES OR TESSERACT_GEOMETRY_ENABLE_EXAMPLES)
  add_subdirectory(examples)
endif()

if(TESSERACT_ENABLE_TESTING OR TESSERACT_GEOMETRY_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  cpack(VERSION ${pkg_extracted_version}
        MAINTAINER_NAME ${pkg_extracted_maintainer_name}
        MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
        DESCRIPTION ${pkg_extracted_description}
        LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
        README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
        LINUX_DEPENDS "libassimp-dev"
                      "libconsole-bridge-dev"
                      "liboctomap-dev (>= 1.9.3) | ${TESSERACT_PACKAGE_PREFIX}octomap"
                      "libeigen3-dev"
                      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
        WINDOWS_DEPENDS "Assimp"
                        "console_bridge"
                        "octomap | ${TESSERACT_PACKAGE_PREFIX}octomap"
                        "Eigen3"
                        "${TESSERACT_PACKAGE_PREFIX}tesseract-common")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
                                UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
                                DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
                                DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
                                DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(tesseract_common Eigen3 console_bridge octomap assimp tinyxml2_vendor TinyXML2)
ament_package()
