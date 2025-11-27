cmake_minimum_required(VERSION 3.10.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

include(cmake/trajopt_macros.cmake)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(Eigen3 REQUIRED)
# Prefer module-mode Boost discovery but respect custom Boost_DIR/BOOST_ROOT
# settings when provided so users with non-system Boost installations can
# satisfy the dependency without patching the build. If Boost is missing, fail
# with a clear message that references the helper script.
find_package(Boost REQUIRED COMPONENTS system filesystem serialization program_options)
include_directories(${Boost_INCLUDE_DIRS})
if(NOT Boost_FOUND)
  message(FATAL_ERROR "Boost (system/filesystem/program_options/serialization) not found. Install libboost-dev "
                      "libboost-graph-dev libboost-system-dev libboost-filesystem-dev libboost-program-options-dev libboost-serialization-dev "
                      "(or run scripts/install_system_deps.sh) and "
                      "then delete any cached build/trajopt_common install/trajopt_common log/trajopt_common directories before"
                      " reconfiguring. If you have a custom Boost installation, set BOOST_ROOT or Boost_DIR accordingly before "
                      "re-running CMake.")
endif()
find_path(BOOST_GRAPH_INCLUDE_DIR
          NAMES boost/graph/adjacency_list.hpp
          PATHS ${Boost_INCLUDE_DIRS}
          NO_DEFAULT_PATH)
if(NOT BOOST_GRAPH_INCLUDE_DIR)
  # Fall back to a broader search so that typical system include locations are
  # still considered when Boost did not report its include dirs explicitly.
  find_path(BOOST_GRAPH_INCLUDE_DIR boost/graph/adjacency_list.hpp)
endif()

if(NOT BOOST_GRAPH_INCLUDE_DIR)
  message(FATAL_ERROR "Boost graph headers not found. Install libboost-dev libboost-graph-dev libboost-system-dev "
                      "libboost-filesystem-dev libboost-program-options-dev libboost-serialization-dev (or run scripts/install_system_deps.sh) and then delete any cached build/"
                      "trajopt_common install/trajopt_common log/trajopt_common directories before reconfiguring."
                      " If you have a custom Boost installation, set BOOST_ROOT or Boost_DIR accordingly before re-running CMake.")
endif()

find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_collision COMPONENTS core REQUIRED)
find_package(tesseract_kinematics COMPONENTS core REQUIRED)
find_package(PkgConfig QUIET)
find_package(tinyxml2 CONFIG QUIET)

set(TINYXML2_TARGET tinyxml2::tinyxml2)
set(TINYXML2_FOUND FALSE)

if(TARGET ${TINYXML2_TARGET})
  set(TINYXML2_FOUND TRUE)
elseif(PkgConfig_FOUND)
  pkg_check_modules(TINYXML2 QUIET tinyxml2)

  if(TINYXML2_FOUND)
    add_library(${TINYXML2_TARGET} INTERFACE IMPORTED)
    set_target_properties(
      ${TINYXML2_TARGET}
      PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${TINYXML2_INCLUDE_DIRS}"
                 INTERFACE_LINK_LIBRARIES "${TINYXML2_LINK_LIBRARIES}")
  endif()
endif()

if(NOT TINYXML2_FOUND)
  find_library(TINYXML2_LIBRARY NAMES tinyxml2)
  find_path(TINYXML2_INCLUDE_DIR NAMES tinyxml2.h)

  if(TINYXML2_LIBRARY AND TINYXML2_INCLUDE_DIR)
    add_library(${TINYXML2_TARGET} UNKNOWN IMPORTED)
    set_target_properties(
      ${TINYXML2_TARGET}
      PROPERTIES IMPORTED_LOCATION "${TINYXML2_LIBRARY}"
                 INTERFACE_INCLUDE_DIRECTORIES "${TINYXML2_INCLUDE_DIR}")
    set(TINYXML2_FOUND TRUE)
  endif()
endif()

if(NOT TINYXML2_FOUND)
  message(FATAL_ERROR "tinyxml2 not found. Install it via your package manager, ensure pkg-config can locate it, or "
                      "provide tinyxml2 headers and libraries to CMake via TINYXML2_LIBRARY/TINYXML2_INCLUDE_DIR.\n"
                      "If you have a custom installation, set PKG_CONFIG_PATH or CMAKE_PREFIX_PATH accordingly before re-running "
                      "CMake.")
endif()
find_package(tesseract_state_solver COMPONENTS kdl REQUIRED)

# Load variable for clang tidy args, compiler options and cxx version
trajopt_variables()

set(UTILS_SOURCE_FILES
    src/stl_to_string.cpp
    src/clock.cpp
    src/collision_types.cpp
    src/collision_utils.cpp
    src/logging.cpp
    src/utils.cpp)

add_library(${PROJECT_NAME} ${UTILS_SOURCE_FILES})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC Eigen3::Eigen
         Boost::serialization
         ${Boost_LIBRARIES}
         ${TINYXML2_TARGET}
         tesseract::tesseract_common
         tesseract::tesseract_collision_core
         tesseract::tesseract_kinematics_core
         tesseract::tesseract_state_solver_kdl)
target_compile_options(${PROJECT_NAME} PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TRAJOPT_COMPILE_DEFINITIONS}
                                                  TRAJOPT_DATA_DIR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/data")
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TRAJOPT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME} ENABLE ${TRAJOPT_ENABLE_CLANG_TIDY})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")

configure_package(NAMESPACE trajopt TARGETS ${PROJECT_NAME})

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

if(TRAJOPT_ENABLE_TESTING)
  enable_testing()
  add_custom_target(run_tests)
  add_subdirectory(test)
endif()

install(FILES "${CMAKE_CURRENT_LIST_DIR}/cmake/trajopt_macros.cmake" DESTINATION lib/cmake/${PROJECT_NAME})

foreach(dir data)
  install(DIRECTORY ${dir}/ DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

if(TRAJOPT_PACKAGE)
  cpack(
    VERSION ${pkg_extracted_version} MAINTAINER <https://github.com/tesseract-robotics/trajopt>
    VENDOR "ROS-Industrial"
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ""
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    PACKAGE_PREFIX ${TRAJOPT_PACKAGE_PREFIX}
    COMPONENT ${PROJECT_NAME})
endif()
