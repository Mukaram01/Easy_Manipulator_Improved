add_library(
  ${PROJECT_NAME}_simple
  src/cereal_serialization.cpp
  src/interpolation.cpp
  src/simple_motion_planner.cpp
  src/profile/simple_planner_fixed_size_assign_no_ik_move_profile.cpp
  src/profile/simple_planner_fixed_size_assign_move_profile.cpp
  src/profile/simple_planner_fixed_size_move_profile.cpp
  src/profile/simple_planner_lvs_assign_no_ik_move_profile.cpp
  src/profile/simple_planner_lvs_assign_move_profile.cpp
  src/profile/simple_planner_lvs_no_ik_move_profile.cpp
  src/profile/simple_planner_lvs_move_profile.cpp
  src/profile/simple_planner_profile.cpp)
target_link_libraries(${PROJECT_NAME}_simple PUBLIC ${PROJECT_NAME}_core Boost::boost cereal::cereal)
target_compile_options(${PROJECT_NAME}_simple PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_simple PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_simple PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_simple ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_simple PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_simple
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_simple PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:include>")
add_dependencies(${PROJECT_NAME}_simple ${PROJECT_NAME}_core)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include COMPONENT ${PROJECT_NAME}_simple)

# Testing
if(TESSERACT_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# Generate and install a package configuration alongside the exported
# targets so the simple planner can be discovered from an install space.
include(CMakePackageConfigHelpers)
set(_${PROJECT_NAME}_simple_config_dir ${CMAKE_CURRENT_BINARY_DIR}/cmake)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}_simpleConfig.cmake.in
  ${_${PROJECT_NAME}_simple_config_dir}/${PROJECT_NAME}_simpleConfig.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}_simple)
write_basic_package_version_file(
  ${_${PROJECT_NAME}_simple_config_dir}/${PROJECT_NAME}_simpleConfigVersion.cmake
  VERSION ${pkg_extracted_version}
  COMPATIBILITY SameMajorVersion)

# Configure Components
configure_component(
  COMPONENT ${PROJECT_NAME}_simple
  NAMESPACE tesseract
  EXPORT_NAME tesseract_motion_planners_simpleTargets
  EXPORT_FILE tesseract_motion_planners_simpleTargets.cmake
  TARGETS ${PROJECT_NAME}_simple
  DEPENDENCIES
    tesseract_motion_planners_core
    tesseract_environment
    tesseract_command_language
    cereal)

install(
  FILES
    ${_${PROJECT_NAME}_simple_config_dir}/${PROJECT_NAME}_simpleConfig.cmake
    ${_${PROJECT_NAME}_simple_config_dir}/${PROJECT_NAME}_simpleConfigVersion.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/tesseract_motion_planners_simpleTargets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}_simple
  COMPONENT ${PROJECT_NAME}_simple)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT ${PROJECT_NAME}_simple
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract motion planners simple components"
    COMPONENT_DEPENDS ${PROJECT_NAME}_core)
endif()
