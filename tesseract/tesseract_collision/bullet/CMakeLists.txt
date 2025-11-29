# Recreate upstream CMake logic for Bullet collision manager and
# provide a fallback definition of the find_bullet() helper when the
# ros_industrial_cmake_boilerplate module is unavailable. The overlay
# is copied into the workspace before colcon builds the upstream
# package, so this file augments the upstream version.

cmake_minimum_required(VERSION 3.10)

include("${CMAKE_CURRENT_LIST_DIR}/../../tesseract_common/cmake/tesseract_cmake_boilerplate.cmake")
tesseract_initialize_boilerplate_support()

find_package(tesseract_common REQUIRED CONFIG)
find_package(tesseract_geometry REQUIRED CONFIG)
include(${CMAKE_CURRENT_LIST_DIR}/../../tesseract_common/cmake/tesseract_macros.cmake)
find_package(PkgConfig REQUIRED)
pkg_check_modules(tinyxml2 REQUIRED tinyxml2)

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

include(${CMAKE_CURRENT_LIST_DIR}/cmake/bullet-find-bullet.cmake)

find_bullet()

# Create target for Bullet implementation
add_library(
  ${PROJECT_NAME}_bullet
  src/bullet_cast_bvh_manager.cpp
  src/bullet_cast_simple_manager.cpp
  src/bullet_collision_shape_cache.cpp
  src/bullet_discrete_bvh_manager.cpp
  src/bullet_discrete_simple_manager.cpp
  src/bullet_utils.cpp
  src/convex_hull_compat.cpp
  src/convex_hull_utils.cpp
  src/tesseract_compound_collision_algorithm.cpp
  src/tesseract_compound_compound_collision_algorithm.cpp
  src/tesseract_collision_configuration.cpp
  src/tesseract_convex_convex_algorithm.cpp
  src/tesseract_gjk_pair_detector.cpp)
target_link_libraries(
  ${PROJECT_NAME}_bullet
  PUBLIC
    ${PROJECT_NAME}_core
    Eigen3::Eigen
    Bullet3::Bullet
    ${TESSERACT_GEOM_TARGET}
    console_bridge::console_bridge
    octomap
    octomath
    ${tinyxml2_LIBRARIES})
target_compile_options(${PROJECT_NAME}_bullet PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_bullet PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_bullet PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_bullet PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_bullet ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(${PROJECT_NAME}_bullet PRIVATE ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_bullet
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../core/include>"
         "$<INSTALL_INTERFACE:include>"
         ${tinyxml2_INCLUDE_DIRS})

# Create target for Bullet implementation
add_library(${PROJECT_NAME}_bullet_factories src/bullet_factories.cpp)
target_link_libraries(${PROJECT_NAME}_bullet_factories PUBLIC ${PROJECT_NAME}_bullet)
target_compile_options(${PROJECT_NAME}_bullet_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_bullet_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_bullet_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_bullet_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_bullet_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(${PROJECT_NAME}_bullet_factories PRIVATE ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_bullet_factories
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../core/include>"
         "$<INSTALL_INTERFACE:include>"
         ${tinyxml2_INCLUDE_DIRS})

# Convex decomposition libraries
if(HACD_LIBRARY)
  add_library(${PROJECT_NAME}_hacd_convex_decomposition src/convex_decomposition_hacd.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_hacd_convex_decomposition
    PUBLIC
      ${PROJECT_NAME}_bullet
      Eigen3::Eigen
      Bullet3::Bullet
      ${TESSERACT_GEOM_TARGET}
      console_bridge::console_bridge
      ${HACD_LIBRARY})
  target_compile_options(${PROJECT_NAME}_hacd_convex_decomposition PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
  target_compile_options(${PROJECT_NAME}_hacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${PROJECT_NAME}_hacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
  target_cxx_version(${PROJECT_NAME}_hacd_convex_decomposition PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_clang_tidy(${PROJECT_NAME}_hacd_convex_decomposition ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_code_coverage(${PROJECT_NAME}_hacd_convex_decomposition PRIVATE ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  target_include_directories(
    ${PROJECT_NAME}_hacd_convex_decomposition
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
           "$<INSTALL_INTERFACE:include>")
  install_targets(TARGETS ${PROJECT_NAME}_hacd_convex_decomposition COMPONENT bullet)
endif()

option(TESSERACT_COLLISION_BUILD_CREATE_CONVEX_HULL "Build the create_convex_hull utility" ON)

if(NOT MSVC AND TESSERACT_COLLISION_BUILD_CREATE_CONVEX_HULL)
  # Create target for creating convex hulls from meshes
  add_executable(create_convex_hull src/create_convex_hull.cpp)
  target_link_libraries(
    create_convex_hull
    PUBLIC
      ${PROJECT_NAME}_core
      ${PROJECT_NAME}_bullet
      Bullet3::Bullet
      Boost::boost
      Boost::program_options
      Eigen3::Eigen
      ${TESSERACT_COMMON_TARGET}
      ${TESSERACT_GEOM_TARGET}
      console_bridge::console_bridge
      octomap
      octomath
      ${tinyxml2_LIBRARIES})
  target_compile_options(create_convex_hull PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(create_convex_hull PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_cxx_version(create_convex_hull PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_clang_tidy(create_convex_hull ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_include_directories(
    create_convex_hull
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../core/include>"
           ${tinyxml2_INCLUDE_DIRS})

  list(APPEND PACKAGE_LIBRARIES create_convex_hull)

  install_targets(TARGETS create_convex_hull COMPONENT bullet)
endif()

# Add factory library so contact_managers_factory can find these factories by defauult
set(CONTACT_MANAGERS_PLUGINS ${CONTACT_MANAGERS_PLUGINS} "${PROJECT_NAME}_bullet_factories" PARENT_SCOPE)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
        COMPONENT bullet
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN "*.inl"
        PATTERN ".svn" EXCLUDE)

# Plugin should not be export only installed
install(TARGETS ${PROJECT_NAME}_bullet_factories
        COMPONENT bullet
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

configure_package(
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_bullet
  DEPENDENCIES
    "tesseract_collision COMPONENTS core"
    Eigen3
    console_bridge
    tesseract_geometry
    tesseract_common
    octomap
  CFG_EXTRAS cmake/bullet-extras.cmake)

install(FILES cmake/bullet-find-bullet.cmake
        DESTINATION lib/cmake/tesseract_collision_bullet)

if(TESSERACT_PACKAGE)
  cpack_component(COMPONENT bullet
                  VERSION ${pkg_extracted_version}
                  DESCRIPTION "Tesseract Collision bullet components"
                  COMPONENT_DEPENDS core
                  LINUX_DEPENDS "libbullet-dev" "libbullet-extras-dev"
                  WINDOWS_DEPENDS "libbullet-dev" "libbullet-extras-dev")
endif()
