cmake_minimum_required(VERSION 3.15.0)

include("${CMAKE_CURRENT_LIST_DIR}/../tesseract/tesseract_common/cmake/tesseract_cmake_boilerplate.cmake")
tesseract_initialize_boilerplate_support()
if(NOT COMMAND extract_package_metadata)
  message(FATAL_ERROR "Failed to load tesseract_cmake_boilerplate.cmake; extract_package_metadata is unavailable")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/../tesseract/tesseract_common/cmake/tesseract_macros.cmake")

extract_package_metadata(pkg PACKAGE_XML "${CMAKE_CURRENT_LIST_DIR}/package.xml")
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

# Core collision backends used by the plugin registrations
find_package(tesseract_collision CONFIG REQUIRED)
find_package(tesseract_collision_fcl CONFIG QUIET)

option(TESSERACT_ENABLE_BULLET "Build Bullet collision plugins" ON)
if(TESSERACT_ENABLE_BULLET)
  find_package(tesseract_collision_bullet CONFIG REQUIRED)
endif()

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE tesseract::tesseract_collision)
if(TARGET tesseract::tesseract_collision_fcl)
  target_link_libraries(${PROJECT_NAME} INTERFACE tesseract::tesseract_collision_fcl)
endif()

if(TESSERACT_ENABLE_BULLET AND TARGET tesseract::tesseract_collision_bullet)
  target_link_libraries(${PROJECT_NAME} INTERFACE tesseract::tesseract_collision_bullet)
endif()

configure_package(NAMESPACE tesseract TARGETS ${PROJECT_NAME})
