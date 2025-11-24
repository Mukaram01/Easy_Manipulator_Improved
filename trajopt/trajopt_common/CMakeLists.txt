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
# Fail fast with a helpful message if the common graph header is missing before
# invoking CMake's Boost discovery logic. This provides clearer instructions
# than the default "Could NOT find Boost (missing: Boost_INCLUDE_DIR graph)"
# error, and it points users at the helper script that installs the necessary
# development packages.
find_path(BOOST_GRAPH_INCLUDE_DIR boost/graph/adjacency_list.hpp)
if(NOT BOOST_GRAPH_INCLUDE_DIR)
  message(FATAL_ERROR "Boost graph headers not found. Install libboost-dev libboost-graph-dev libboost-program-options-dev "
                      "libboost-serialization-dev (or run scripts/install_system_deps.sh) and then delete any cached build/"
                      "trajopt_common install/trajopt_common log/trajopt_common directories before reconfiguring.")
endif()

# Force use of CMake's built-in Boost finder to avoid picking up any
# BoostConfig.cmake files from overlay workspaces that may not provide the
# required components (e.g., graph). Fall back to a clear, actionable error
# that points users at the helper script when Boost headers are missing,
# matching the common "Could NOT find Boost (missing: Boost_INCLUDE_DIR
# graph)" failure seen on minimal environments.
if(DEFINED Boost_DIR)
  message(WARNING "Ignoring Boost_DIR='${Boost_DIR}' to ensure system Boost packages with graph/program_options/serialization are used")
  unset(Boost_DIR CACHE)
endif()
# BOOST_ROOT can also point CMake at a non-system Boost installation (for
# example, in a container overlay) that lacks the graph/program_options/
# serialization components required by trajopt_common. Clear it so the
# system headers installed by scripts/install_system_deps.sh are always
# preferred.
if(DEFINED ENV{BOOST_ROOT})
  message(WARNING "Ignoring BOOST_ROOT='${BOOST_ROOT}' to ensure system Boost packages with graph/program_options/serialization are used")
  unset(ENV{BOOST_ROOT})
endif()
# Force module-mode discovery to dodge any stray BoostConfig.cmake files that may
# be present in overlay workspaces and lack the required components.
set(Boost_NO_BOOST_CMAKE ON CACHE BOOL "Prefer FindBoost.cmake over BoostConfig.cmake" FORCE)
find_package(Boost MODULE QUIET COMPONENTS graph program_options serialization)
if(NOT Boost_FOUND)
  message(FATAL_ERROR "Boost graph/program_options/serialization headers not found. "
                      "Install libboost-dev libboost-graph-dev libboost-program-options-dev libboost-serialization-dev "
                      "or run scripts/install_system_deps.sh, then remove any cached build/trajopt_common install/trajopt_common "
                      "log/trajopt_common directories before reconfiguring.")
endif()
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_collision COMPONENTS core REQUIRED)
find_package(tesseract_kinematics COMPONENTS core REQUIRED)
find_package(tinyxml2 CONFIG REQUIRED)
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
         Boost::graph
         Boost::program_options
         Boost::serialization
         tinyxml2::tinyxml2
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
