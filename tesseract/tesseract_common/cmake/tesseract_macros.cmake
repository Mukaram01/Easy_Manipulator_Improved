# Minimal implementations of the Tesseract CMake helper macros.
# The original project provides a richer set of utilities, but for the
# purposes of this workspace we only require a subset to support the build.

include_guard(GLOBAL)

function(tesseract_variables)
  set(default_enable_clang_tidy OFF)
  set(default_clang_tidy_args "")
  set(default_cxx_version 17)
  set(default_compile_options_public "")
  set(default_compile_definitions "")
  set(default_enable_code_coverage OFF)

  set(TESSERACT_ENABLE_CLANG_TIDY ${default_enable_clang_tidy} CACHE BOOL
      "Enable clang-tidy on Tesseract targets")
  set(TESSERACT_CLANG_TIDY_ARGS "${default_clang_tidy_args}" CACHE STRING
      "Additional clang-tidy arguments")
  set(TESSERACT_CXX_VERSION ${default_cxx_version} CACHE STRING
      "C++ standard version used by Tesseract targets")
  set(TESSERACT_COMPILE_OPTIONS_PUBLIC "${default_compile_options_public}" CACHE STRING
      "Public compile options applied to Tesseract targets")
  set(TESSERACT_COMPILE_DEFINITIONS "${default_compile_definitions}" CACHE STRING
      "Public compile definitions applied to Tesseract targets")
  set(TESSERACT_ENABLE_CODE_COVERAGE ${default_enable_code_coverage} CACHE BOOL
      "Enable code coverage for Tesseract targets")
endfunction()

function(target_clang_tidy target)
  if(NOT TESSERACT_ENABLE_CLANG_TIDY)
    return()
  endif()

  find_program(_clang_tidy_exe NAMES clang-tidy)
  if(NOT _clang_tidy_exe)
    message(WARNING "clang-tidy requested but not found in PATH")
    return()
  endif()

  cmake_parse_arguments(_TCT "" "ENABLE" "ARGS" ${ARGN})
  if(DEFINED _TCT_ENABLE AND NOT _TCT_ENABLE)
    return()
  endif()

  set(_clang_tidy_command "${_clang_tidy_exe}")
  if(_TCT_ARGS)
    list(JOIN _TCT_ARGS ";" _clang_tidy_args)
    set(_clang_tidy_command "${_clang_tidy_command};${_clang_tidy_args}")
  endif()
  set_target_properties(${target} PROPERTIES CXX_CLANG_TIDY "${_clang_tidy_command}")
endfunction()

function(target_cxx_version target)
  cmake_parse_arguments(_TCV "" "VERSION" "PUBLIC;PRIVATE;INTERFACE" ${ARGN})
  if(NOT _TCV_VERSION)
    message(FATAL_ERROR "target_cxx_version requires VERSION to be specified")
  endif()

  foreach(scope IN ITEMS PUBLIC PRIVATE INTERFACE)
    if(_TCV_${scope})
      target_compile_features(${target} ${scope} cxx_std_${_TCV_VERSION})
    endif()
  endforeach()
endfunction()

function(initialize_code_coverage)
  # Code coverage support is not required in this environment, so this is a no-op.
endfunction()

function(add_code_coverage_all_targets)
  # Placeholder to keep compatibility with the upstream build scripts.
endfunction()

function(target_code_coverage)
  # Placeholder to keep compatibility with the upstream build scripts.
endfunction()

function(configure_package)
  cmake_parse_arguments(_TCP "" "NAMESPACE" "TARGETS" ${ARGN})
  if(NOT _TCP_TARGETS)
    return()
  endif()

  foreach(target ${_TCP_TARGETS})
    if(TARGET ${target})
      install(TARGETS ${target} EXPORT ${target}_export)
      install(EXPORT ${target}_export
              NAMESPACE ${_TCP_NAMESPACE}::
              DESTINATION lib/cmake/${target})
      if(COMMAND ament_export_targets)
        ament_export_targets(${target}_export)
      endif()
    endif()
  endforeach()
endfunction()
