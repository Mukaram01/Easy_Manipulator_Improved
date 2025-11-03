find_package(fcl 0.6 REQUIRED)

if(NOT TARGET tesseract::tesseract_common AND TARGET tesseract_common::tesseract_common)
  add_library(tesseract::tesseract_common ALIAS tesseract_common::tesseract_common)
endif()
if(NOT TARGET tesseract::tesseract_geometry AND TARGET tesseract_geometry::tesseract_geometry)
  add_library(tesseract::tesseract_geometry ALIAS tesseract_geometry::tesseract_geometry)
endif()

set(TESSERACT_COMMON_TARGET tesseract::tesseract_common)
if(NOT TARGET ${TESSERACT_COMMON_TARGET} AND TARGET tesseract_common::tesseract_common)
  set(TESSERACT_COMMON_TARGET tesseract_common::tesseract_common)
endif()

set(TESSERACT_GEOM_TARGET tesseract::tesseract_geometry)
if(NOT TARGET ${TESSERACT_GEOM_TARGET} AND TARGET tesseract_geometry::tesseract_geometry)
  set(TESSERACT_GEOM_TARGET tesseract_geometry::tesseract_geometry)
endif()

# Create target for FCL implementation
add_library(
  ${PROJECT_NAME}_fcl
  src/fcl_collision_geometry_cache.cpp
  src/fcl_discrete_managers.cpp
  src/fcl_utils.cpp
  src/fcl_collision_object_wrapper.cpp)
target_link_libraries(
  ${PROJECT_NAME}_fcl
  PUBLIC ${PROJECT_NAME}_core
         Eigen3::Eigen
         ${TESSERACT_GEOM_TARGET}
         fcl
         console_bridge::console_bridge
         octomap
         octomath)
target_compile_options(${PROJECT_NAME}_fcl PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_fcl PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_fcl PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_fcl PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_fcl ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  ${PROJECT_NAME}_fcl
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_fcl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                      "$<INSTALL_INTERFACE:include>")

add_library(${PROJECT_NAME}_fcl_factories src/fcl_factories.cpp)
target_link_libraries(${PROJECT_NAME}_fcl_factories PUBLIC ${PROJECT_NAME}_fcl)
target_compile_options(${PROJECT_NAME}_fcl_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_fcl_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_fcl_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_fcl_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_fcl_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_fcl_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_fcl_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                                "$<INSTALL_INTERFACE:include>")

# Add factory library so contact_managers_factory can find these factories by defauult
set(CONTACT_MANAGERS_PLUGINS ${CONTACT_MANAGERS_PLUGINS} "${PROJECT_NAME}_fcl_factories" PARENT_SCOPE)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  COMPONENT fcl
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

# Plugin should not be export only installed
install(
  TARGETS ${PROJECT_NAME}_fcl_factories
  COMPONENT fcl
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

configure_component(
  COMPONENT fcl
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_fcl
  DEPENDENCIES "tesseract_collision COMPONENTS core" "fcl 0.6")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT fcl
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Collision FCL components"
    COMPONENT_DEPENDS core
    LINUX_DEPENDS "libfcl-dev (>= 0.6.0) | fcl (>= 0.6.0) | ${TESSERACT_PACKAGE_PREFIX}fcl"
    WINDOWS_DEPENDS "libfcl-dev (>= 0.6.0) | fcl (>= 0.6.0) | ${TESSERACT_PACKAGE_PREFIX}fcl")
endif()
