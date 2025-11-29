if(NOT TARGET tesseract::tesseract_common)
  if(TARGET tesseract_common::tesseract_common)
    add_library(tesseract::tesseract_common ALIAS tesseract_common::tesseract_common)
  elseif(TARGET tesseract_common)
    add_library(tesseract::tesseract_common ALIAS tesseract_common)
  endif()
endif()
if(NOT TARGET tesseract::tesseract_geometry)
  if(TARGET tesseract_geometry::tesseract_geometry)
    add_library(tesseract::tesseract_geometry ALIAS tesseract_geometry::tesseract_geometry)
  elseif(TARGET tesseract_geometry)
    add_library(tesseract::tesseract_geometry ALIAS tesseract_geometry)
  endif()
endif()

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

# Create interface for core
add_library(
  ${PROJECT_NAME}_core
  src/common.cpp
  src/contact_managers_plugin_factory.cpp
  src/continuous_contact_manager.cpp
  src/discrete_contact_manager.cpp
  src/serialization.cpp
  src/types.cpp
  src/utils.cpp
  src/contact_result_validator.cpp)
target_link_libraries(
  ${PROJECT_NAME}_core
  PUBLIC Eigen3::Eigen
         ${TESSERACT_COMMON_TARGET}
         ${TESSERACT_GEOM_TARGET}
         Boost::boost
         Boost::system
         Boost::filesystem
         Boost::serialization
         ${tinyxml2_LIBRARIES}
         yaml-cpp
         octomap)
target_compile_options(${PROJECT_NAME}_core PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_compile_definitions(${PROJECT_NAME}_core
                           PRIVATE TESSERACT_CONTACT_MANAGERS_PLUGIN_PATH="${TESSERACT_CONTACT_MANAGERS_PLUGIN_PATH}")
target_cxx_version(${PROJECT_NAME}_core PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_core ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  ${PROJECT_NAME}_core
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_core PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>"
                                                       ${tinyxml2_INCLUDE_DIRS})

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  COMPONENT core
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

configure_package(
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_core
  DEPENDENCIES
    Eigen3
    PkgConfig
    tinyxml2
    console_bridge
    tesseract_geometry
    tesseract_common
    yaml-cpp
    octomap
    "Boost COMPONENTS filesystem system program_options serialization"
  CFG_EXTRAS cmake/core-extras.cmake)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT core
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Collision core components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "libyaml-cpp-dev"
      "liboctomap-dev"
      "libboost-filesystem-dev"
      "libboost-system-dev"
      "libboost-program-options-dev"
      "libboost-serialization-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver"
    WINDOWS_DEPENDS
      "console_bridge"
      "Eigen3"
      "yaml-cpp"
      "liboctomap-dev"
      "libboost-filesystem-dev"
      "libboost-system-dev"
      "libboost-program-options-dev"
      "libboost-serialization-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-scene-graph"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-state-solver")
endif()
