cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

find_package(console_bridge REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_kinematics REQUIRED COMPONENTS core)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(Eigen3 REQUIRED)

include(${tesseract_common_DIR}/tesseract_macros.cmake)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                  ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                    ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

initialize_code_coverage(ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
set(COVERAGE_EXCLUDE
    /usr/*
    /opt/*
    ${CMAKE_CURRENT_LIST_DIR}/test/*
    /*/install/*
    /*/devel/*
    /*/gtest/*)
add_code_coverage_all_targets(EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})

# Enable Testing
if(TESSERACT_ENABLE_TESTING)
  find_gtest()
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
endif()

# Core
set(SUPPORTED_COMPONENTS core)
add_subdirectory(core)

# ISP
option(TESSERACT_BUILD_ISP "Build the Iterative Spline Parameterization (ISP) time parameterization implementation" ON)
if(TESSERACT_BUILD_ISP)
  message("Building ISP")
  list(APPEND SUPPORTED_COMPONENTS isp)
  add_subdirectory(isp)
endif()

# KDL
option(TESSERACT_BUILD_KDL "Build the Constant TCP Speed Parameterization (KDL) time parameterization implementation"
       ON)
if(TESSERACT_BUILD_KDL)
  find_package(tesseract_kinematics_kdl QUIET)
  if(tesseract_kinematics_kdl_FOUND)
    message("Building KDL")
    list(APPEND SUPPORTED_COMPONENTS kdl)
    add_subdirectory(kdl)
  else()
    message(WARNING "tesseract_kinematics_kdl not found, skipping KDL time parameterization")
  endif()
endif()

# TOTG
option(TESSERACT_BUILD_TOTG "Build the Time Optimal Trajectory Generation (TOTG) time parameterization implementation"
       ON)
if(TESSERACT_BUILD_TOTG)
  message("Building TOTG")
  list(APPEND SUPPORTED_COMPONENTS totg)
  add_subdirectory(totg)
endif()

# RUCKIG
option(TESSERACT_BUILD_RUCKIG "Build the Ruckig Trajectory Smoothing time parameterization implementation" ON)
if(TESSERACT_BUILD_RUCKIG)
  message("Building Ruckig")
  list(APPEND SUPPORTED_COMPONENTS ruckig)
  add_subdirectory(ruckig)
endif()

# Package configuration
set(_${PROJECT_NAME}_component_paths "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-component-paths.cmake")
file(WRITE ${_${PROJECT_NAME}_component_paths} "# Component package hints for ${PROJECT_NAME}\n")
foreach(_component IN LISTS SUPPORTED_COMPONENTS)
  file(APPEND
       ${_${PROJECT_NAME}_component_paths}
       "set(${PROJECT_NAME}_${_component}_DIR \"\\${CMAKE_CURRENT_LIST_DIR}/../${PROJECT_NAME}_${_component}\")\n")
endforeach()

configure_package(
  COMPONENT core
  SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS}
  # Avoid forcing a default component dependency when none is requested; the
  # meta-package configuration now treats core as internal unless explicitly
  # asked for, preventing a hard dependency on a non-existent *_core package.
  CFG_EXTRAS ${_${PROJECT_NAME}_component_paths})

install(CODE "
  set(_components ${SUPPORTED_COMPONENTS})
  foreach(_component IN LISTS _components)
    set(_component_prefix \"${PROJECT_NAME}_\${_component}\")
    set(_component_dir \"\${CMAKE_INSTALL_PREFIX}/lib/cmake/\${_component_prefix}\")
    if(NOT IS_DIRECTORY \"\${_component_dir}\")
      continue()
    endif()

    file(GLOB _prefixed_wrapper_files
      \"\${_component_dir}/\${_component_prefix}-config.cmake\"
      \"\${_component_dir}/\${_component_prefix}-targets*.cmake\"
      \"\${_component_dir}/\${_component_prefix}-extras.cmake\")
    foreach(_wrapper_file IN LISTS _prefixed_wrapper_files)
      get_filename_component(_wrapper_name \"\${_wrapper_file}\" NAME)
      string(REPLACE \"\${_component_prefix}\" \"\${_component}\" _compat_name \"\${_wrapper_name}\")
      set(_compat_path \"\${_component_dir}/\${_compat_name}\")
      if(NOT EXISTS \"\${_compat_path}\")
        configure_file(\"\${_wrapper_file}\" \"\${_compat_path}\" COPYONLY)
      endif()
    endforeach()

    set(_version_src \"\${_component_dir}/\${_component_prefix}ConfigVersion.cmake\")
    set(_version_alias \"\${_component_dir}/\${_component_prefix}-config-version.cmake\")
    if(EXISTS \"\${_version_src}\" AND NOT EXISTS \"\${_version_alias}\")
      configure_file(\"\${_version_src}\" \"\${_version_alias}\" COPYONLY)
    endif()
  endforeach()
")

set(_${PROJECT_NAME}_wrapper_dir "${CMAKE_CURRENT_BINARY_DIR}/cmake/config_wrappers")
file(MAKE_DIRECTORY ${_${PROJECT_NAME}_wrapper_dir})
include(CMakePackageConfigHelpers)
foreach(_component IN LISTS SUPPORTED_COMPONENTS)
  set(_component_prefix "${PROJECT_NAME}_${_component}")
  set(_component_install_dir "lib/cmake/${_component_prefix}")
  set(COMPONENT_PREFIX "${_component_prefix}")
  set(COMPONENT_NAME "${_component_prefix}")
  configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/cmake/component-targets-wrapper.cmake.in"
    "${_${PROJECT_NAME}_wrapper_dir}/${_component_prefix}-targets.cmake"
    @ONLY)
  install(FILES "${_${PROJECT_NAME}_wrapper_dir}/${_component_prefix}-targets.cmake"
          DESTINATION "${_component_install_dir}")
  if(_component STREQUAL "core")
    configure_file(
      "${CMAKE_CURRENT_LIST_DIR}/cmake/component-extras-wrapper.cmake.in"
      "${_${PROJECT_NAME}_wrapper_dir}/${_component_prefix}-extras.cmake"
      @ONLY)
    install(FILES "${_${PROJECT_NAME}_wrapper_dir}/${_component_prefix}-extras.cmake"
            DESTINATION "${_component_install_dir}")
    set(_core_config_wrapper "${_${PROJECT_NAME}_wrapper_dir}/${_component_prefix}-config.cmake")
    configure_package_config_file("${CMAKE_CURRENT_LIST_DIR}/cmake/component-config-wrapper.cmake.in"
                                  "${_core_config_wrapper}"
                                  INSTALL_DESTINATION "${_component_install_dir}")
    set(_core_config_version "${_${PROJECT_NAME}_wrapper_dir}/${_component_prefix}-config-version.cmake")
    write_basic_package_version_file(
      "${_core_config_version}"
      VERSION "${PROJECT_VERSION}"
      COMPATIBILITY SameMajorVersion)
    install(FILES "${_core_config_wrapper}" "${_core_config_version}" DESTINATION "${_component_install_dir}")
  endif()
endforeach()

if(TESSERACT_PACKAGE)
  cpack_component_package(
    VERSION ${pkg_extracted_version}
    MAINTAINER_NAME ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    COMPONENT_DEPENDS ${SUPPORTED_COMPONENTS})

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
