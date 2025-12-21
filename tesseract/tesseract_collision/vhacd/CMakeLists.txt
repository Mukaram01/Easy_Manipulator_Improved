include(${CMAKE_CURRENT_LIST_DIR}/../../tesseract_common/cmake/tesseract_macros.cmake)

# Make trajopt CMake helpers discoverable whether we are building from source or
# against an installed trajopt_common package. The installed package exports the
# macros alongside trajopt_common_DIR; the in-tree path is retained for source builds.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../../trajopt/trajopt_common/cmake")
find_package(trajopt_common REQUIRED)

set(_trajopt_macros_candidates
    "${trajopt_common_DIR}/trajopt_macros.cmake"
    "${CMAKE_CURRENT_LIST_DIR}/../../../trajopt/trajopt_common/cmake/trajopt_macros.cmake")

set(_trajopt_macros_found FALSE)
foreach(_trajopt_macros ${_trajopt_macros_candidates})
  if(EXISTS "${_trajopt_macros}")
    include("${_trajopt_macros}")
    set(_trajopt_macros_found TRUE)
    break()
  endif()
endforeach()

if(NOT _trajopt_macros_found OR NOT COMMAND trajopt_variables)
  message(FATAL_ERROR
          "trajopt_macros.cmake could not be found after locating trajopt_common; "
          "ensure trajopt_common is installed and exports trajopt_macros.cmake")
endif()

trajopt_variables()

tesseract_resolve_tinyxml2(TESSERACT_TINYXML2_TARGET TESSERACT_TINYXML2_LINK_LIBRARIES)
if(NOT TESSERACT_TINYXML2_TARGET AND NOT TESSERACT_TINYXML2_LINK_LIBRARIES)
  message(FATAL_ERROR "Failed to locate a TinyXML2 library for tesseract_collision vhacd components")
endif()

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

# Convex decomposition library
add_library(${PROJECT_NAME}_vhacd_convex_decomposition src/convex_decomposition_vhacd.cpp)
target_link_libraries(
  ${PROJECT_NAME}_vhacd_convex_decomposition
  PUBLIC ${PROJECT_NAME}_core
         ${PROJECT_NAME}_bullet
         Eigen3::Eigen
         ${TESSERACT_GEOM_TARGET}
         ${TESSERACT_TINYXML2_TARGET}
         ${TESSERACT_TINYXML2_LINK_LIBRARIES}
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_vhacd_convex_decomposition PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_vhacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_vhacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_vhacd_convex_decomposition PUBLIC VERSION ${TESSERACT_CXX_VERSION})
# target_clang_tidy(${PROJECT_NAME}_vhacd_convex_decomposition ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})

target_code_coverage(
  ${PROJECT_NAME}_vhacd_convex_decomposition
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_vhacd_convex_decomposition PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../bullet/include>"
                                                   "$<INSTALL_INTERFACE:include>")

if(NOT MSVC)
  find_package(Threads REQUIRED)
  # Create target for creating convex decompositions from meshes
  add_executable(create_convex_decomposition src/create_convex_decomposition.cpp)
  target_link_libraries(
    create_convex_decomposition
    PRIVATE ${PROJECT_NAME}_vhacd_convex_decomposition
            Boost::boost
            Boost::program_options
            Eigen3::Eigen
            ${TESSERACT_COMMON_TARGET}
            ${TESSERACT_GEOM_TARGET}
            ${TESSERACT_TINYXML2_TARGET}
            ${TESSERACT_TINYXML2_LINK_LIBRARIES}
            console_bridge::console_bridge
            Threads::Threads)
  target_compile_options(create_convex_decomposition PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                             ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(create_convex_decomposition PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_cxx_version(create_convex_decomposition PRIVATE VERSION ${TESSERACT_CXX_VERSION})

  list(APPEND PACKAGE_LIBRARIES create_convex_decomposition)

  install_targets(TARGETS create_convex_decomposition COMPONENT vhacd)
endif()

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  COMPONENT vhacd
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

configure_component(
  COMPONENT vhacd
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_vhacd_convex_decomposition
  DEPENDENCIES "tesseract_collision COMPONENTS core")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT vhacd
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Collision vhacd components"
    COMPONENT_DEPENDS core bullet)
endif()
