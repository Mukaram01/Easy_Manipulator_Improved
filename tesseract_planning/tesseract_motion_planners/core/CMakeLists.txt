# Create interface for core
include(CMakePackageConfigHelpers)

add_library(${PROJECT_NAME}_core src/planner.cpp src/types.cpp src/utils.cpp)
target_link_libraries(
  ${PROJECT_NAME}_core
  PUBLIC tesseract::tesseract_environment
         tesseract::tesseract_common
         tesseract::tesseract_command_language
         console_bridge::console_bridge
         Eigen3::Eigen)
target_compile_options(${PROJECT_NAME}_core PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_core PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_core ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_core PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_core
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_core PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                       "$<INSTALL_INTERFACE:include>")

# Install and export core targets so dependent packages can locate the component config
install(
  TARGETS ${PROJECT_NAME}_core
  EXPORT ${PROJECT_NAME}_core-targets
  DESTINATION lib
  COMPONENT core)

install(
  EXPORT ${PROJECT_NAME}_core-targets
  FILE tesseract_motion_planners_coreTargets.cmake
  NAMESPACE tesseract::
  DESTINATION lib/cmake/tesseract_motion_planners_core
  COMPONENT core)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/tesseract_motion_planners_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/tesseract_motion_planners_coreConfig.cmake
  INSTALL_DESTINATION lib/cmake/tesseract_motion_planners_core)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/tesseract_motion_planners_coreConfig.cmake
        DESTINATION lib/cmake/tesseract_motion_planners_core COMPONENT core)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include COMPONENT core)

# Configure Components
configure_component(
  COMPONENT core
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_core
  DEPENDENCIES
    tesseract_environment
    tesseract_common
    tesseract_command_language
    console_bridge
    Threads
    Boost)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT core
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract motion planners core components"
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libeigen3-dev"
      "libboost-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-command-language"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
    WINDOWS_DEPENDS
      "console-bridge"
      "Eigen3"
      "boost"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-command-language"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment")
endif()
