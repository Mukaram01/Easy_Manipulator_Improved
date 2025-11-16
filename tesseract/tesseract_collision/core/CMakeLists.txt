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
         ${TESSERACT_TINYXML2_TARGET}
         ${TESSERACT_TINYXML2_LINK_LIBRARIES}
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
                                                       "$<INSTALL_INTERFACE:include>")

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

configure_component(
  COMPONENT core
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_core
  DEPENDENCIES
    Eigen3
    console_bridge
    tesseract_geometry
    tesseract_common
    yaml-cpp
    octomap
    "Boost COMPONENTS filesystem system program_options serialization"
  CFG_EXTRAS cmake/core-extras.cmake)

# Some releases of ros_industrial_cmake_boilerplate provide a configure_component
# implementation that does not emit a ${PROJECT_NAME}_coreConfig.cmake file.
# When that happens downstream packages such as tesseract_urdf fail to locate
# the core collision library.  Detect the missing config at configure time and
# fall back to the local configure_package helper so the exported target is
# always installed.
set(_tesseract_core_config_dir "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_core_cmake")
set(_tesseract_core_config "${_tesseract_core_config_dir}/${PROJECT_NAME}_coreConfig.cmake")
if(NOT EXISTS "${_tesseract_core_config}")
  message(WARNING "configure_component() did not generate ${PROJECT_NAME}_coreConfig.cmake; synthesizing fallback config")

  include(CMakePackageConfigHelpers)
  file(MAKE_DIRECTORY "${_tesseract_core_config_dir}")

  # configure_package() installs the export file as
  # "${PROJECT_NAME}_core-export.cmake" in the same directory as the
  # generated config.  Point the fallback config at that file instead of
  # the legacy core-targets.cmake helper so the include statement matches
  # the actual export produced during the build.
  set(CORE_TARGETS_FILENAME "${PROJECT_NAME}_core-export.cmake")
  set(CORE_FIND_DEPENDENCIES
      "include(CMakeFindDependencyMacro)\n"
      "find_dependency(Eigen3)\n"
      "find_dependency(console_bridge)\n"
      "find_dependency(tesseract_geometry)\n"
      "find_dependency(tesseract_common)\n"
      "find_dependency(yaml-cpp)\n"
      "find_dependency(octomap)\n"
      "find_dependency(Boost COMPONENTS filesystem system program_options serialization)\n")
  string(CONCAT CORE_FIND_DEPENDENCIES ${CORE_FIND_DEPENDENCIES})
  configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/cmake/core-config-fallback.cmake.in
    ${_tesseract_core_config}
    @ONLY)

  set(_tesseract_core_version_file "${_tesseract_core_config_dir}/${PROJECT_NAME}_coreConfigVersion.cmake")
  set(_tesseract_core_version "${pkg_extracted_version}")
  if(NOT _tesseract_core_version)
    set(_tesseract_core_version "${PROJECT_VERSION}")
  endif()
  if(NOT _tesseract_core_version)
    set(_tesseract_core_version "0.0.0")
  endif()
  write_basic_package_version_file(
    "${_tesseract_core_version_file}"
    VERSION "${_tesseract_core_version}"
    COMPATIBILITY SameMajorVersion)

  install(
    FILES
      "${_tesseract_core_config}"
      "${_tesseract_core_version_file}"
    DESTINATION
      lib/cmake/${PROJECT_NAME}_core
    COMPONENT
      core)
endif()

unset(CORE_FIND_DEPENDENCIES)
unset(CORE_TARGETS_FILENAME)
unset(_tesseract_core_config)
unset(_tesseract_core_config_dir)

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
