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
  set(_pkg \"${PROJECT_NAME}\")
  foreach(_component IN LISTS _components)
    set(_src_dir \"\${CMAKE_INSTALL_PREFIX}/lib/cmake/\${_pkg}\")
    set(_dst_dir \"\${CMAKE_INSTALL_PREFIX}/lib/cmake/\${_pkg}_\${_component}\")
    file(GLOB _config_files
      \"\${_src_dir}/\${_component}-config.cmake\"
      \"\${_src_dir}/\${_component}-targets*.cmake\"
      \"\${_src_dir}/\${_component}-extras.cmake\"
      \"\${_src_dir}/\${_component}Config.cmake\"
      \"\${_src_dir}/\${_component}Targets*.cmake\")
    if(_config_files)
      file(MAKE_DIRECTORY \"\${_dst_dir}\")
      foreach(_src_file IN LISTS _config_files)
        get_filename_component(_fname \"\${_src_file}\" NAME)
        string(REPLACE \"\${_component}\" \"\${_pkg}_\${_component}\" _dst_fname \"\${_fname}\")
        file(COPY \"\${_src_file}\" DESTINATION \"\${_dst_dir}\")
        file(RENAME \"\${_dst_dir}/\${_fname}\" \"\${_dst_dir}/\${_dst_fname}\")
      endforeach()
    endif()
  endforeach()
")

set(_${PROJECT_NAME}_wrapper_dir "${CMAKE_CURRENT_BINARY_DIR}/cmake/config_wrappers")
file(MAKE_DIRECTORY ${_${PROJECT_NAME}_wrapper_dir})
foreach(_component IN LISTS SUPPORTED_COMPONENTS)
  set(_component_prefix "${PROJECT_NAME}_${_component}")
  set(_component_install_dir "lib/cmake/${_component_prefix}")
  set(COMPONENT_PREFIX "${_component_prefix}")
  configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/cmake/component-targets-wrapper.cmake.in"
    "${_${PROJECT_NAME}_wrapper_dir}/${_component}-targets.cmake"
    @ONLY)
  install(FILES "${_${PROJECT_NAME}_wrapper_dir}/${_component}-targets.cmake"
          DESTINATION "${_component_install_dir}")
  if(_component STREQUAL "core")
    configure_file(
      "${CMAKE_CURRENT_LIST_DIR}/cmake/component-extras-wrapper.cmake.in"
      "${_${PROJECT_NAME}_wrapper_dir}/${_component}-extras.cmake"
      @ONLY)
    install(FILES "${_${PROJECT_NAME}_wrapper_dir}/${_component}-extras.cmake"
            DESTINATION "${_component_install_dir}")
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
