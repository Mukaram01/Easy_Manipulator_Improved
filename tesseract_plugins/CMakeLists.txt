cmake_minimum_required(VERSION 3.10)
project(tesseract_plugins)

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

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-targets)
