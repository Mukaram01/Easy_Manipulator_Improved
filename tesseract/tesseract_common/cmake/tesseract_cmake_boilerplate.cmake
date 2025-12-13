# Minimal shims for ament_cmake and ros_industrial_cmake_boilerplate functionality
# required to build the vendored Tesseract packages in environments where the
# original CMake packages are unavailable (e.g., plain CMake/colcon
# installations without ROS). The upstream projects provide significantly more
# comprehensive helpers; here we replicate only the portions that the local
# packages depend upon.

include_guard(GLOBAL)

# Ensure all build options referenced by the packages exist with sensible
# defaults so that conditional logic works regardless of whether the ROS
# boilerplate is present.
option(TESSERACT_ENABLE_TESTING "Enable building test targets for Tesseract" OFF)
option(TESSERACT_ENABLE_RUN_TESTING "Enable creation of the run_tests convenience target" OFF)
option(TESSERACT_ENABLE_EXAMPLES "Build example targets for Tesseract components" OFF)
option(TESSERACT_ENABLE_BENCHMARKING "Build benchmarking targets for Tesseract components" OFF)
option(TESSERACT_ENABLE_CODE_COVERAGE "Enable code coverage instrumentation" OFF)
option(TESSERACT_ENABLE_CLANG_TIDY "Enable clang-tidy analysis" OFF)
option(TESSERACT_PACKAGE "Generate binary packages with CPack" OFF)
option(TESSERACT_PACKAGE_SOURCE "Generate source packages with CPack" OFF)
option(TESSERACT_COMMON_ENABLE_TESTING "Enable tests specific to tesseract_common" OFF)
option(TESSERACT_COMMON_ENABLE_EXAMPLES "Enable examples specific to tesseract_common" OFF)
option(TESSERACT_GEOMETRY_ENABLE_TESTING "Enable tests specific to tesseract_geometry" OFF)
option(TESSERACT_GEOMETRY_ENABLE_EXAMPLES "Enable examples specific to tesseract_geometry" OFF)
option(TESSERACT_SCENE_GRAPH_ENABLE_TESTING "Enable tests specific to tesseract_scene_graph" OFF)
option(TESSERACT_SCENE_GRAPH_ENABLE_EXAMPLES "Enable examples specific to tesseract_scene_graph" OFF)
option(TESSERACT_COLLISION_ENABLE_TESTING "Enable tests specific to tesseract_collision" OFF)
option(TESSERACT_COLLISION_ENABLE_EXAMPLES "Enable examples specific to tesseract_collision" OFF)
option(TESSERACT_COLLISION_ENABLE_BENCHMARKING "Enable benchmarks specific to tesseract_collision" OFF)

if(NOT DEFINED TESSERACT_PACKAGE_SOURCE_UPLOAD)
  set(TESSERACT_PACKAGE_SOURCE_UPLOAD "" CACHE STRING "Upload target for source packages")
endif()
if(NOT DEFINED TESSERACT_PACKAGE_SOURCE_DPUT_HOST)
  set(TESSERACT_PACKAGE_SOURCE_DPUT_HOST "" CACHE STRING "dput host for source package uploads")
endif()
if(NOT DEFINED TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT)
  set(TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT "" CACHE STRING "Debian revision increment for source packages")
endif()
if(NOT DEFINED TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS)
  set(TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS "" CACHE STRING "Target Debian distributions for source packages")
endif()

# Helper to normalise whitespace extracted from XML content.
function(tesseract_trim_whitespace value out_var)
  set(_tmp "${value}")
  string(REGEX REPLACE "^[\r\n\t ]+" "" _tmp "${_tmp}")
  string(REGEX REPLACE "[\r\n\t ]+$" "" _tmp "${_tmp}")
  string(REGEX REPLACE "[\r\n]+" " " _tmp "${_tmp}")
  string(REGEX REPLACE "[ ]+" " " _tmp "${_tmp}")
  set(${out_var} "${_tmp}" PARENT_SCOPE)
endfunction()

# Provide a thin wrapper around ctest so downstream code can create the
# canonical run_tests target even without the ROS boilerplate helpers.
if(NOT COMMAND add_run_tests_target)
  function(add_run_tests_target)
    cmake_parse_arguments(_ART "" "ENABLE" "" ${ARGN})
    if(NOT _ART_ENABLE)
      return()
    endif()

    if(TARGET run_tests)
      return()
    endif()

    if(CMAKE_CTEST_COMMAND)
      set(_ctest_command "${CMAKE_CTEST_COMMAND}")
    else()
      find_program(_ctest_exe ctest)
      set(_ctest_command "${_ctest_exe}")
    endif()

    if(NOT _ctest_command)
      message(WARNING "Unable to locate ctest executable; run_tests target will not be created")
      return()
    endif()

    add_custom_target(
      run_tests
      COMMAND "${_ctest_command}" --output-on-failure
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
  endfunction()
endif()

# Stubs for the packaging helper macros. They are no-ops in this environment
# because packaging is outside the scope of the build we are interested in.
if(NOT COMMAND cpack_component_package)
  function(cpack_component_package)
    message(STATUS "cpack_component_package() stub invoked; packaging is disabled in this environment")
  endfunction()
endif()

if(NOT COMMAND cpack_debian_source_package)
  function(cpack_debian_source_package)
    message(STATUS "cpack_debian_source_package() stub invoked; source packaging is disabled in this environment")
  endfunction()
endif()

if(NOT COMMAND cpack_component)
  function(cpack_component)
    message(STATUS "cpack_component() stub invoked; packaging is disabled in this environment")
  endfunction()
endif()

function(install_targets)
  cmake_parse_arguments(_IT "" "COMPONENT" "TARGETS" ${ARGN})

  if(NOT _IT_TARGETS)
    message(WARNING "install_targets() called without TARGETS; nothing will be installed")
    return()
  endif()

  set(_install_component)
  if(_IT_COMPONENT)
    set(_install_component COMPONENT ${_IT_COMPONENT})
  endif()

  install(TARGETS ${_IT_TARGETS}
          ${_install_component}
          RUNTIME DESTINATION bin
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib)
endfunction()

if(NOT COMMAND configure_component)
  function(configure_component)
    cmake_parse_arguments(
      _CC
      ""
      "COMPONENT;NAMESPACE"
      "TARGETS;DEPENDENCIES;CFG_EXTRAS"
      ${ARGN})

    if(NOT _CC_TARGETS)
      message(WARNING "configure_component() stub called without TARGETS; nothing to configure")
      return()
    endif()

    if(_CC_NAMESPACE)
      set(_cc_namespace_args NAMESPACE ${_CC_NAMESPACE})
    else()
      set(_cc_namespace_args)
    endif()

    if(_CC_COMPONENT)
      set(_cc_component_args COMPONENT ${_CC_COMPONENT})
    else()
      set(_cc_component_args)
    endif()

    configure_package(
      ${_cc_namespace_args}
      ${_cc_component_args}
      TARGETS ${_CC_TARGETS}
      DEPENDENCIES ${_CC_DEPENDENCIES}
      CFG_EXTRAS ${_CC_CFG_EXTRAS})
  endfunction()
endif()

# ---------------------------------------------------------------------------
# ament_cmake shim
# ---------------------------------------------------------------------------
if(NOT DEFINED TESSERACT_AMENT_SHIM_INITIALISED)
  set(TESSERACT_AMENT_SHIM_INITIALISED TRUE)

  if(NOT DEFINED ament_cmake_FOUND OR NOT ament_cmake_FOUND)
    find_package(ament_cmake QUIET)
  endif()

  if(NOT ament_cmake_FOUND)
    set(ament_cmake_FOUND TRUE CACHE INTERNAL "Stub ament_cmake availability")

    if(NOT COMMAND ament_export_dependencies)
      function(ament_export_dependencies)
        # No-op in the stub implementation.
      endfunction()
    endif()

    if(NOT COMMAND ament_export_include_directories)
      function(ament_export_include_directories)
        # No-op for non-ROS builds.
      endfunction()
    endif()

    if(NOT COMMAND ament_export_interfaces)
      function(ament_export_interfaces)
        # No-op for non-ROS builds.
      endfunction()
    endif()

    if(NOT COMMAND ament_export_libraries)
      function(ament_export_libraries)
        # No-op for non-ROS builds.
      endfunction()
    endif()

    if(NOT COMMAND ament_export_targets)
      function(ament_export_targets)
        # No-op for non-ROS builds.
      endfunction()
    endif()

    if(NOT COMMAND ament_target_dependencies)
      function(ament_target_dependencies)
        # Downstream packages do not require dependency propagation here.
      endfunction()
    endif()

    if(NOT COMMAND ament_package)
      function(ament_package)
        cmake_parse_arguments(_AP "" "CONFIG_EXTRAS" "" ${ARGN})
        set(AMENT_PACKAGE_NAME "${PROJECT_NAME}")
        if(_AP_CONFIG_EXTRAS)
          foreach(_extra IN LISTS _AP_CONFIG_EXTRAS)
            if(NOT IS_ABSOLUTE "${_extra}")
              set(_extra "${CMAKE_CURRENT_LIST_DIR}/${_extra}")
            endif()
            if(NOT EXISTS "${_extra}")
              message(WARNING "ament_package() stub could not locate config extras file ${_extra}")
            endif()
          endforeach()
        endif()

        # Replicate the small subset of ament's packaging hooks that colcon relies
        # upon to propagate CMAKE_PREFIX_PATH/AMENT_PREFIX_PATH.  The real
        # ament_cmake implementation generates a suite of hook templates, but we
        # only need the cmake_prefix_path and ament_prefix_path variants to
        # support dependent packages locating exported *Config.cmake files.
        if(NOT AMENT_PACKAGE_NAME)
          message(FATAL_ERROR "ament_package() requires AMENT_PACKAGE_NAME to be set")
        endif()

        if(NOT CMAKE_INSTALL_PREFIX)
          message(FATAL_ERROR "ament_package() stub requires CMAKE_INSTALL_PREFIX to generate environment hooks")
        endif()

        set(_ament_hook_prefix_posix "${CMAKE_INSTALL_PREFIX}")
        string(REPLACE "\"" "\\\"" _ament_hook_prefix_posix "${_ament_hook_prefix_posix}")
        file(TO_NATIVE_PATH "${CMAKE_INSTALL_PREFIX}" _ament_hook_prefix_native)

        set(_ament_hook_build_dir "${CMAKE_CURRENT_BINARY_DIR}/ament_package_hooks/${AMENT_PACKAGE_NAME}")
        file(MAKE_DIRECTORY "${_ament_hook_build_dir}")
        set(_ament_hook_destination "share/${AMENT_PACKAGE_NAME}/hook")

        set(_ament_cmake_hook_sh "${_ament_hook_build_dir}/cmake_prefix_path.sh")
        set(_ament_cmake_hook_bat "${_ament_hook_build_dir}/cmake_prefix_path.bat")
        set(_ament_hook_sh "${_ament_hook_build_dir}/ament_prefix_path.sh")
        set(_ament_hook_bat "${_ament_hook_build_dir}/ament_prefix_path.bat")

        file(WRITE "${_ament_cmake_hook_sh}"
"# Generated by tesseract_cmake_boilerplate ament_package() shim\n"
"_colcon_prepend_unique_value CMAKE_PREFIX_PATH \"${_ament_hook_prefix_posix}\"\n")

        file(WRITE "${_ament_hook_sh}"
"# Generated by tesseract_cmake_boilerplate ament_package() shim\n"
"_colcon_prepend_unique_value AMENT_PREFIX_PATH \"${_ament_hook_prefix_posix}\"\n")

        file(WRITE "${_ament_cmake_hook_bat}"
"@echo off\r\n"
":: Generated by tesseract_cmake_boilerplate ament_package() shim\r\n"
"call :_colcon_prepend_unique_value CMAKE_PREFIX_PATH \"${_ament_hook_prefix_native}\"\r\n")

        file(WRITE "${_ament_hook_bat}"
"@echo off\r\n"
":: Generated by tesseract_cmake_boilerplate ament_package() shim\r\n"
"call :_colcon_prepend_unique_value AMENT_PREFIX_PATH \"${_ament_hook_prefix_native}\"\r\n")

        install(
          FILES
            "${_ament_cmake_hook_sh}"
            "${_ament_hook_sh}"
            "${_ament_cmake_hook_bat}"
            "${_ament_hook_bat}"
          DESTINATION "${_ament_hook_destination}")
      endfunction()
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# ros_industrial_cmake_boilerplate shim
# ---------------------------------------------------------------------------
if(NOT DEFINED TESSERACT_ROS_IB_SHIM_INITIALISED)
  set(TESSERACT_ROS_IB_SHIM_INITIALISED TRUE)

  if(NOT DEFINED ros_industrial_cmake_boilerplate_FOUND OR NOT ros_industrial_cmake_boilerplate_FOUND)
    find_package(ros_industrial_cmake_boilerplate QUIET)
  endif()

  if(NOT ros_industrial_cmake_boilerplate_FOUND)
    set(ros_industrial_cmake_boilerplate_FOUND TRUE CACHE INTERNAL "Stub ros_industrial_cmake_boilerplate availability")

    if(NOT COMMAND extract_package_metadata)
      set(_TESSERACT_DEFINE_EXTRACT_METADATA_STUB TRUE)
    endif()
  endif()
endif()

# Shared implementation for the extract_package_metadata shim. Defining the
# parsing logic in a dedicated helper makes it easy to provide a fallback even
# when the real ros_industrial_cmake_boilerplate package is partially
# available (e.g., discovered by find_package() but not actually providing the
# helper function).
function(_tesseract_extract_package_metadata_impl prefix)
  cmake_parse_arguments(_EPM "" "PACKAGE_XML" "" ${ARGN})
  if(_EPM_PACKAGE_XML)
    set(_pkg_xml "${_EPM_PACKAGE_XML}")
  else()
    set(_pkg_xml "${CMAKE_CURRENT_LIST_DIR}/package.xml")
  endif()

  if(NOT IS_ABSOLUTE "${_pkg_xml}")
    set(_pkg_xml "${CMAKE_CURRENT_LIST_DIR}/${_pkg_xml}")
  endif()

  if(NOT EXISTS "${_pkg_xml}")
    message(FATAL_ERROR "extract_package_metadata: package.xml not found at ${_pkg_xml}")
  endif()

  file(READ "${_pkg_xml}" _pkg_contents)
  set(_pkg_flat "${_pkg_contents}")
  string(REGEX REPLACE "[\r\n]" " " _pkg_flat "${_pkg_flat}")

  set(_name "")
  if(_pkg_flat MATCHES "<name>([^<]+)</name>")
    set(_name "${CMAKE_MATCH_1}")
    tesseract_trim_whitespace("${_name}" _name)
  endif()

  set(_version "")
  if(_pkg_flat MATCHES "<version>([^<]+)</version>")
    set(_version "${CMAKE_MATCH_1}")
    tesseract_trim_whitespace("${_version}" _version)
  endif()

  set(_description "")
  if(_pkg_flat MATCHES "<description[^>]*>([^<]*)</description>")
    set(_description "${CMAKE_MATCH_1}")
    tesseract_trim_whitespace("${_description}" _description)
  endif()

  set(_maintainer_name "")
  set(_maintainer_email "")
  if(_pkg_flat MATCHES "<maintainer[^>]*email=\"([^\"]*)\"[^>]*>([^<]*)</maintainer>")
    set(_maintainer_email "${CMAKE_MATCH_1}")
    set(_maintainer_name "${CMAKE_MATCH_2}")
    tesseract_trim_whitespace("${_maintainer_email}" _maintainer_email)
    tesseract_trim_whitespace("${_maintainer_name}" _maintainer_name)
  elseif(_pkg_flat MATCHES "<maintainer[^>]*>([^<]*)</maintainer>")
    set(_maintainer_name "${CMAKE_MATCH_1}")
    tesseract_trim_whitespace("${_maintainer_name}" _maintainer_name)
  endif()

  if(NOT _version)
    set(_version "0.0.0")
  endif()

  set(${prefix}_extracted_name "${_name}" PARENT_SCOPE)
  set(${prefix}_extracted_version "${_version}" PARENT_SCOPE)
  set(${prefix}_extracted_description "${_description}" PARENT_SCOPE)
  set(${prefix}_extracted_maintainer_name "${_maintainer_name}" PARENT_SCOPE)
  set(${prefix}_extracted_maintainer_email "${_maintainer_email}" PARENT_SCOPE)
endfunction()

# The legacy build scripts invoke this helper before accessing the metadata.
# The shims are initialised during inclusion, so the function is retained only
# for compatibility with existing CMakeLists files.
function(tesseract_initialize_boilerplate_support)
  # No additional work required; the stubs are initialised eagerly.
endfunction()

  if(NOT COMMAND extract_package_metadata)
    function(extract_package_metadata prefix)
      _tesseract_extract_package_metadata_impl(${prefix} ${ARGN})

      # Propagate parsed metadata to the caller's scope instead of keeping it
      # local to the wrapper, matching the behavior of
      # ros_industrial_cmake_boilerplate's implementation.
      set(${prefix}_extracted_name "${${prefix}_extracted_name}" PARENT_SCOPE)
      set(${prefix}_extracted_version "${${prefix}_extracted_version}" PARENT_SCOPE)
      set(${prefix}_extracted_description "${${prefix}_extracted_description}" PARENT_SCOPE)
      set(${prefix}_extracted_maintainer_name "${${prefix}_extracted_maintainer_name}" PARENT_SCOPE)
      set(${prefix}_extracted_maintainer_email "${${prefix}_extracted_maintainer_email}" PARENT_SCOPE)
    endfunction()
  elseif(DEFINED _TESSERACT_DEFINE_EXTRACT_METADATA_STUB)
    unset(_TESSERACT_DEFINE_EXTRACT_METADATA_STUB)
  endif()
