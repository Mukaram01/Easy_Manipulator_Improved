cmake_minimum_required(VERSION 3.10.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

option(HAVE_BPMPD "Build the bundled BPMPD caller and enable BPMPD support" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(GUROBI QUIET)
find_package(osqp QUIET)
include(CheckIncludeFileCXX)
check_include_file_cxx(osqp/osqp_api_types.h OSQP_HAVE_V1_HEADERS)
find_package(Eigen3 REQUIRED)
find_package(trajopt_common REQUIRED)

list(
  APPEND
    TRAJOPT_MACRO_SEARCH_PATHS
    "${trajopt_common_DIR}"
    "${trajopt_common_DIR}/cmake"
    "${trajopt_common_DIR}/../cmake"
    "${trajopt_common_DIR}/../lib/cmake/trajopt_common"
    "${CMAKE_CURRENT_SOURCE_DIR}/../trajopt_common/cmake")

find_file(
  TRAJOPT_MACROS_FILE
  trajopt_macros.cmake
  PATHS ${TRAJOPT_MACRO_SEARCH_PATHS}
  PATH_SUFFIXES "" cmake
  NO_DEFAULT_PATH)
if(NOT TRAJOPT_MACROS_FILE)
  find_file(TRAJOPT_MACROS_FILE trajopt_macros.cmake)
endif()
if(NOT TRAJOPT_MACROS_FILE)
  message(FATAL_ERROR
          "Failed to locate trajopt_macros.cmake; ensure trajopt_common is properly built and installed or available in the workspace.")
endif()
include(${TRAJOPT_MACROS_FILE})
if(NOT TARGET JsonCpp::JsonCpp)
  find_package(jsoncpp REQUIRED)
elseif(NOT TARGET jsoncpp_lib)
  add_library(jsoncpp_lib ALIAS JsonCpp::JsonCpp)
endif()
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem serialization program_options)
find_package(OpenMP REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tesseract_common REQUIRED)
include(${tesseract_common_DIR}/tesseract_macros.cmake)
tesseract_resolve_tinyxml2(TRAJOPT_TINYXML2_TARGET TRAJOPT_TINYXML2_LINK_LIBRARIES)
if(NOT TRAJOPT_TINYXML2_TARGET AND NOT TRAJOPT_TINYXML2_LINK_LIBRARIES)
  message(FATAL_ERROR "Failed to locate a TinyXML2 library to satisfy trajopt_sco")
endif()
find_package(tesseract_collision COMPONENTS core REQUIRED)
find_package(tesseract_kinematics COMPONENTS core REQUIRED)
find_package(tesseract_state_solver COMPONENTS kdl REQUIRED)

# qpOASES
option(TRAJOPT_BUILD_qpOASES "Build qpOASES components" ON)
if(TRAJOPT_BUILD_qpOASES)
  find_package(qpOASES QUIET)
endif()

# Load variable for clang tidy args, compiler options and cxx version
trajopt_variables()

set(SCO_SOURCE_FILES
    src/solver_interface.cpp
    src/solver_utils.cpp
    src/modeling.cpp
    src/expr_ops.cpp
    src/optimizers.cpp
    src/modeling_utils.cpp
    src/num_diff.cpp)

# BPMPD requires proprietary libraries which are not bundled with this
# repository. Disable by default unless explicitly enabled.
if(NOT DEFINED HAVE_BPMPD)
  if(NOT APPLE AND NOT WIN32)
    set(HAVE_BPMPD TRUE)
  else()
    set(HAVE_BPMPD FALSE)
  endif()
endif()
message(STATUS "trajopt_sco: HAVE_BPMPD = ${HAVE_BPMPD}")

if(HAVE_BPMPD)
  add_executable(bpmpd_caller src/bpmpd_caller.cpp)

  if(CMAKE_SIZEOF_VOID_P EQUAL 8) # 64 bits
    set(BPMPD_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/3rdpartylib/bpmpd_linux64.a")
  else()
    set(BPMPD_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/3rdpartylib/bpmpd_linux32.a")
  endif()

  target_link_libraries(bpmpd_caller PRIVATE ${BPMPD_LIBRARY})
  target_compile_definitions(bpmpd_caller PUBLIC BPMPD_WORKING_DIR="${CMAKE_CURRENT_BINARY_DIR}")
  target_cxx_version(bpmpd_caller PUBLIC VERSION ${TRAJOPT_CXX_VERSION})
  target_include_directories(bpmpd_caller PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:include>")
  target_include_directories(bpmpd_caller SYSTEM
                             PUBLIC $<TARGET_PROPERTY:trajopt::trajopt_common,INTERFACE_INCLUDE_DIRECTORIES>)

  list(APPEND SCO_SOURCE_FILES src/bpmpd_interface.cpp)

  install(
    TARGETS bpmpd_caller
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
  install(FILES src/bpmpd.par DESTINATION bin)
endif()

if(GUROBI_FOUND)
  list(APPEND SCO_SOURCE_FILES src/gurobi_interface.cpp)
endif(GUROBI_FOUND)

if(osqp_FOUND)
  list(APPEND SCO_SOURCE_FILES src/osqp_interface.cpp)
endif()

if(qpOASES_FOUND AND TRAJOPT_BUILD_qpOASES)
  list(APPEND SCO_SOURCE_FILES src/qpoases_interface.cpp)
endif()

add_library(${PROJECT_NAME} ${SCO_SOURCE_FILES})
if(OSQP_HAVE_V1_HEADERS)
  set(TRAJOPT_OSQP_V1 1)
else()
  set(TRAJOPT_OSQP_V1 0)
endif()
message(STATUS "trajopt_sco: TRAJOPT_OSQP_V1=${TRAJOPT_OSQP_V1}")
target_compile_definitions(${PROJECT_NAME} PRIVATE TRAJOPT_OSQP_V1=${TRAJOPT_OSQP_V1})
if(GUROBI_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${GUROBI_LIBRARIES})
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${GUROBI_INCLUDE_DIRS})
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_GUROBI=ON)
endif()
if(HAVE_BPMPD)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${BPMPD_LIBRARY})
  target_compile_definitions(${PROJECT_NAME} PRIVATE
    HAVE_BPMPD=ON
    BPMPD_CALLER="${CMAKE_INSTALL_PREFIX}/bin/bpmpd_caller")
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_BPMPD=OFF)
endif()
if(osqp_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE osqp::osqp)
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_OSQP=ON)
endif()
if(qpOASES_FOUND AND TRAJOPT_BUILD_qpOASES)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${qpOASES_LIBRARIES})
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${qpOASES_INCLUDE_DIRS})
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_QPOASES=ON)
endif()

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC trajopt::trajopt_common
         Boost::boost
         Eigen3::Eigen
         rclcpp::rclcpp
         ${CMAKE_DL_LIBS}
         ${TRAJOPT_TINYXML2_TARGET}
         ${TRAJOPT_TINYXML2_LINK_LIBRARIES}
         jsoncpp_lib
         OpenMP::OpenMP_CXX)
target_compile_options(${PROJECT_NAME} PRIVATE ${TRAJOPT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME} PUBLIC ${TRAJOPT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${TRAJOPT_COMPILE_DEFINITIONS})
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

# install(FILES cmake/FindGUROBI.cmake cmake/FindqpOASES.cmake DESTINATION lib/cmake/${PROJECT_NAME})

if(TRAJOPT_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TRAJOPT_ENABLE_RUN_TESTING})

  # if (HAVE_BPMPD) # BPMPD Expects the caller to be in the install location so for # testing it must manually be copied
  # so tests will run. add_custom_target(test_depend ALL COMMAND ${CMAKE_COMMAND} -E copy_if_different
  # ${CMAKE_CURRENT_SOURCE_DIR}/src/bpmpd.par ${CMAKE_INSTALL_PREFIX}/bin/bpmpd.par COMMAND ${CMAKE_COMMAND} -E
  # copy_if_different ${CMAKE_CURRENT_BINARY_DIR}/bpmpd_caller ${CMAKE_INSTALL_PREFIX}/bin/bpmpd_caller)
  # add_dependencies(test_depend ${PROJECT_NAME})

  # add_dependencies(run_tests test_depend) endif()

  add_subdirectory(test)
endif()

if(TRAJOPT_PACKAGE)
  set(LINUX_DEPENDS
      "libeigen3-dev"
      "libboost-dev"
      "libjsoncpp-dev"
      "${TRAJOPT_PACKAGE_PREFIX}trajopt-common")
  set(WINDOWS_DEPENDS
      "Eigen3"
      "boost"
      "jsoncpp"
      "${TRAJOPT_PACKAGE_PREFIX}trajopt-common")

  if(GUROBI_FOUND)
    list(APPEND LINUX_DEPENDS "${TRAJOPT_PACKAGE_PREFIX}gurobi")
    list(APPEND WINDOWS_DEPENDS "${TRAJOPT_PACKAGE_PREFIX}gurobi")
  endif()

  if(osqp_FOUND)
    list(APPEND LINUX_DEPENDS "${TRAJOPT_PACKAGE_PREFIX}osqp")
    list(APPEND WINDOWS_DEPENDS "${TRAJOPT_PACKAGE_PREFIX}osqp")
  endif()

  if(qpOASES_FOUND AND TRAJOPT_BUILD_qpOASES)
    list(APPEND LINUX_DEPENDS "${TRAJOPT_PACKAGE_PREFIX}qpoases")
    list(APPEND WINDOWS_DEPENDS "${TRAJOPT_PACKAGE_PREFIX}qpoases")
  endif()

  cpack(
    VERSION ${pkg_extracted_version} MAINTAINER <https://github.com/tesseract-robotics/trajopt>
    VENDOR "ROS-Industrial"
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ""
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    PACKAGE_PREFIX ${TRAJOPT_PACKAGE_PREFIX}
    LINUX_DEPENDS ${LINUX_DEPENDS}
    WINDOWS_DEPENDS ${WINDOWS_DEPENDS})
endif()
