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

function(_tesseract_collect_tinyxml2_libraries target out_var)
  if(NOT TARGET "${target}")
    set(${out_var} "" PARENT_SCOPE)
    return()
  endif()

  set(_visited "${ARGN}")
  if(_visited)
    list(FIND _visited "${target}" _already)
    if(NOT _already EQUAL -1)
      set(${out_var} "" PARENT_SCOPE)
      return()
    endif()
  endif()

  list(APPEND _visited "${target}")

  set(_paths "")
  get_target_property(_location "${target}" IMPORTED_LOCATION)
  if(_location)
    list(APPEND _paths "${_location}")
  endif()

  get_target_property(_imported_configs "${target}" IMPORTED_CONFIGURATIONS)
  foreach(_config IN LISTS _imported_configs)
    string(TOUPPER "${_config}" _config_upper)
    foreach(_prop IMPORTED_LOCATION_${_config_upper} IMPORTED_IMPLIB_${_config_upper})
      get_target_property(_candidate "${target}" "${_prop}")
      if(_candidate)
        list(APPEND _paths "${_candidate}")
      endif()
    endforeach()
  endforeach()

  get_target_property(_interface_libs "${target}" INTERFACE_LINK_LIBRARIES)
  foreach(_lib IN LISTS _interface_libs)
    if(TARGET "${_lib}")
      _tesseract_collect_tinyxml2_libraries("${_lib}" _nested_paths ${_visited})
      if(_nested_paths)
        list(APPEND _paths ${_nested_paths})
      endif()
    elseif(_lib)
      list(APPEND _paths "${_lib}")
    endif()
  endforeach()

  if(_paths)
    list(REMOVE_DUPLICATES _paths)
  endif()

  set(${out_var} "${_paths}" PARENT_SCOPE)
endfunction()

function(tesseract_resolve_tinyxml2 out_target out_libraries)
  find_package(tinyxml2_vendor QUIET)
  find_package(TinyXML2 QUIET)

  set(_resolved_target "")
  set(_resolved_libraries)

  foreach(_candidate tinyxml2::tinyxml2 TinyXML2::TinyXML2)
    if(TARGET ${_candidate})
      set(_resolved_target ${_candidate})
      _tesseract_collect_tinyxml2_libraries(${_candidate} _candidate_libraries)
      if(_candidate_libraries)
        list(APPEND _resolved_libraries ${_candidate_libraries})
      endif()
      break()
    endif()
  endforeach()

  if(NOT _resolved_target AND TARGET tinyxml2_vendor::tinyxml2_vendor)
    _tesseract_collect_tinyxml2_libraries(tinyxml2_vendor::tinyxml2_vendor _vendor_libraries)
    if(_vendor_libraries)
      list(APPEND _resolved_libraries ${_vendor_libraries})
    endif()
    set(_resolved_target tinyxml2_vendor::tinyxml2_vendor)
  endif()

  if(NOT _resolved_libraries AND DEFINED TinyXML2_LIBRARIES AND TinyXML2_LIBRARIES)
    list(APPEND _resolved_libraries ${TinyXML2_LIBRARIES})
  endif()

  if(NOT _resolved_target AND _resolved_libraries)
    foreach(_library_candidate IN LISTS _resolved_libraries)
      if(_library_candidate STREQUAL "optimized" OR _library_candidate STREQUAL "debug"
         OR _library_candidate STREQUAL "general")
        continue()
      endif()
      if(_library_candidate MATCHES "^\\$<")
        continue()
      endif()
      set(_resolved_target "${_library_candidate}")
      break()
    endforeach()
  endif()

  if(NOT _resolved_libraries)
    find_library(_tinyxml2_library NAMES tinyxml2)
    if(_tinyxml2_library)
      list(APPEND _resolved_libraries "${_tinyxml2_library}")
      if(NOT _resolved_target)
        set(_resolved_target "${_tinyxml2_library}")
      endif()
    endif()
  endif()

  if(_resolved_libraries)
    list(REMOVE_DUPLICATES _resolved_libraries)
  endif()

  set(${out_target} "${_resolved_target}" PARENT_SCOPE)
  set(${out_libraries} "${_resolved_libraries}" PARENT_SCOPE)
endfunction()

function(_tesseract_normalize_dependency_entries out_var)
  # Group dependency arguments so multi-token entries such as
  # "Boost COMPONENTS filesystem" are preserved when emitting find_dependency
  # statements. The function treats each top-level list item as the start of a
  # dependency and extends it when encountering well-known modifiers (e.g.,
  # REQUIRED) or COMPONENTS clauses.
  set(_normalized_dependencies "")
  set(_current_dependency "")
  set(_in_components FALSE)

  foreach(_token IN LISTS ARGN)
    if(NOT _token)
      continue()
    endif()

    if(_token STREQUAL "COMPONENTS")
      if(_current_dependency)
        string(APPEND _current_dependency " COMPONENTS")
        set(_in_components TRUE)
      else()
        # If COMPONENTS appears without a base package just preserve the token.
        set(_current_dependency "COMPONENTS")
      endif()
      continue()
    endif()

    set(_start_new_dependency FALSE)
    if(_current_dependency)
      if(_in_components)
        if(_token MATCHES "^.+::.+$" OR _token MATCHES "^[A-Z].*")
          set(_start_new_dependency TRUE)
          set(_in_components FALSE)
        endif()
      else()
        if(_token MATCHES "^(REQUIRED|OPTIONAL|QUIET|MODULE|NO_MODULE|CONFIG|EXACT|GLOBAL)$"
           OR _token MATCHES "^[0-9].*")
          set(_start_new_dependency FALSE)
        else()
          set(_start_new_dependency TRUE)
        endif()
      endif()
    endif()

    if(_start_new_dependency)
      list(APPEND _normalized_dependencies "${_current_dependency}")
      set(_current_dependency "${_token}")
    else()
      if(_current_dependency)
        string(APPEND _current_dependency " ${_token}")
      else()
        set(_current_dependency "${_token}")
      endif()
    endif()
  endforeach()

  if(_current_dependency)
    list(APPEND _normalized_dependencies "${_current_dependency}")
  endif()

  set(${out_var} "${_normalized_dependencies}" PARENT_SCOPE)
endfunction()

function(_tesseract_configure_meta_package)
  cmake_parse_arguments(
    _TCMP
    ""
    "COMPONENT"
    "SUPPORTED_COMPONENTS;DEPENDENCIES;CFG_EXTRAS"
    ${ARGN})

  if(NOT _TCMP_COMPONENT)
    return()
  endif()

  if(NOT PROJECT_NAME)
    message(FATAL_ERROR "configure_package COMPONENT requires PROJECT_NAME to be set")
  endif()

  set(_tcmp_package "${PROJECT_NAME}")
  set(_tcmp_supported_components)
  foreach(_tcmp_component IN LISTS _TCMP_SUPPORTED_COMPONENTS)
    if(_tcmp_component)
      list(APPEND _tcmp_supported_components "${_tcmp_component}")
    endif()
  endforeach()
  if(NOT _tcmp_supported_components)
    list(APPEND _tcmp_supported_components "${_TCMP_COMPONENT}")
  endif()
  list(REMOVE_DUPLICATES _tcmp_supported_components)

  set(_tcmp_config_dir "${CMAKE_CURRENT_BINARY_DIR}/${_tcmp_package}_cmake")
  file(MAKE_DIRECTORY "${_tcmp_config_dir}")
  set(_tcmp_config_file "${_tcmp_config_dir}/${_tcmp_package}Config.cmake")
  file(WRITE "${_tcmp_config_file}" "include(CMakeFindDependencyMacro)\n")

  _tesseract_normalize_dependency_entries(_tcmp_dependencies ${_TCMP_DEPENDENCIES})
  foreach(_tcmp_dependency IN LISTS _tcmp_dependencies)
    if(_tcmp_dependency)
      file(APPEND "${_tcmp_config_file}" "find_dependency(${_tcmp_dependency})\n")
    endif()
  endforeach()

  file(APPEND "${_tcmp_config_file}" "set(_${_tcmp_package}_supported_components")
  foreach(_tcmp_component IN LISTS _tcmp_supported_components)
    file(APPEND "${_tcmp_config_file}" " \"${_tcmp_component}\"")
  endforeach()
  file(APPEND "${_tcmp_config_file}" ")\n")
  file(APPEND "${_tcmp_config_file}" "if(NOT ${_tcmp_package}_FIND_COMPONENTS)\n")
  file(APPEND "${_tcmp_config_file}" "  set(${_tcmp_package}_FIND_COMPONENTS \"${_TCMP_COMPONENT}\")\n")
  file(APPEND "${_tcmp_config_file}" "endif()\n")
  file(APPEND "${_tcmp_config_file}" "set(_${_tcmp_package}_missing_components)\n")
  file(APPEND "${_tcmp_config_file}" "foreach(_component IN LISTS ${_tcmp_package}_FIND_COMPONENTS)\n")
  file(APPEND
    "${_tcmp_config_file}"
    "  list(FIND _${_tcmp_package}_supported_components \"\${_component}\" _component_index)\n")
  file(APPEND "${_tcmp_config_file}" "  if(_component_index EQUAL -1)\n")
  file(APPEND "${_tcmp_config_file}" "    list(APPEND _${_tcmp_package}_missing_components \"\${_component}\")\n")
  file(APPEND "${_tcmp_config_file}" "    continue()\n")
  file(APPEND "${_tcmp_config_file}" "  endif()\n")
  file(APPEND
    "${_tcmp_config_file}"
    "  find_dependency(${_tcmp_package}_\${_component})\n")
  file(APPEND "${_tcmp_config_file}" "endforeach()\n")
  file(APPEND "${_tcmp_config_file}" "if(_${_tcmp_package}_missing_components)\n")
  file(APPEND
    "${_tcmp_config_file}"
    "  list(JOIN _${_tcmp_package}_missing_components \", \" _${_tcmp_package}_missing_components_str)\n")
  file(APPEND
    "${_tcmp_config_file}"
    "  set(${_tcmp_package}_NOT_FOUND_MESSAGE \"Unsupported components requested: \${_${_tcmp_package}_missing_components_str}\")\n")
  file(APPEND "${_tcmp_config_file}" "  set(${_tcmp_package}_FOUND FALSE)\n")
  file(APPEND "${_tcmp_config_file}" "  return()\n")
  file(APPEND "${_tcmp_config_file}" "endif()\n")
  file(APPEND "${_tcmp_config_file}" "set(${_tcmp_package}_FOUND TRUE)\n")

  set(_tcmp_extra_files "")
  foreach(_tcmp_extra IN LISTS _TCMP_CFG_EXTRAS)
    if(NOT _tcmp_extra)
      continue()
    endif()
    if(NOT IS_ABSOLUTE "${_tcmp_extra}")
      set(_tcmp_extra_source "${CMAKE_CURRENT_SOURCE_DIR}/${_tcmp_extra}")
    else()
      set(_tcmp_extra_source "${_tcmp_extra}")
    endif()
    if(NOT EXISTS "${_tcmp_extra_source}")
      message(FATAL_ERROR "configure_package could not locate extra file ${_tcmp_extra_source}")
    endif()
    get_filename_component(_tcmp_extra_filename "${_tcmp_extra_source}" NAME)
    set(_tcmp_extra_destination "${_tcmp_config_dir}/${_tcmp_extra_filename}")
    configure_file("${_tcmp_extra_source}" "${_tcmp_extra_destination}" COPYONLY)
    list(APPEND _tcmp_extra_files "${_tcmp_extra_destination}")
    file(APPEND
      "${_tcmp_config_file}"
      "include(\"\${CMAKE_CURRENT_LIST_DIR}/${_tcmp_extra_filename}\")\n")
  endforeach()

  set(_tcmp_version "${PROJECT_VERSION}")
  if(NOT _tcmp_version)
    set(_tcmp_version "0.0.0")
  endif()
  set(_tcmp_version_file "${_tcmp_config_dir}/${_tcmp_package}ConfigVersion.cmake")
  write_basic_package_version_file(
    "${_tcmp_version_file}"
    VERSION "${_tcmp_version}"
    COMPATIBILITY SameMajorVersion)

  set(_tcmp_install_files "${_tcmp_config_file}" "${_tcmp_version_file}")
  if(_tcmp_extra_files)
    list(APPEND _tcmp_install_files ${_tcmp_extra_files})
  endif()

  install(FILES ${_tcmp_install_files} DESTINATION lib/cmake/${_tcmp_package})
endfunction()

function(configure_package)
  cmake_parse_arguments(
    _TCP
    ""
    "NAMESPACE;COMPONENT"
    "TARGETS;DEPENDENCIES;CFG_EXTRAS;SUPPORTED_COMPONENTS"
    ${ARGN})
  if(NOT _TCP_TARGETS)
    if(_TCP_COMPONENT)
      _tesseract_configure_meta_package(
        COMPONENT ${_TCP_COMPONENT}
        SUPPORTED_COMPONENTS ${_TCP_SUPPORTED_COMPONENTS}
        DEPENDENCIES ${_TCP_DEPENDENCIES}
        CFG_EXTRAS ${_TCP_CFG_EXTRAS})
    endif()
    return()
  endif()

  include(CMakePackageConfigHelpers)

  foreach(target ${_TCP_TARGETS})
    if(TARGET ${target})
      get_target_property(_tcp_target_type ${target} TYPE)
      set(_tcp_has_library_target FALSE)
      if(_tcp_target_type AND NOT _tcp_target_type STREQUAL "EXECUTABLE")
        set(_tcp_has_library_target TRUE)
      endif()

      get_target_property(_tcp_target_source_dir ${target} SOURCE_DIR)
      if(NOT _tcp_target_source_dir)
        set(_tcp_target_source_dir ${CMAKE_CURRENT_SOURCE_DIR})
      endif()

      if(_TCP_NAMESPACE)
        set(_tcp_namespaced_target "${_TCP_NAMESPACE}::${target}")
        if(NOT TARGET ${_tcp_namespaced_target})
          if(_tcp_target_type STREQUAL "EXECUTABLE")
            add_executable(${_tcp_namespaced_target} ALIAS ${target})
          else()
            add_library(${_tcp_namespaced_target} ALIAS ${target})
          endif()
        endif()
      endif()

      # If a component is provided, consolidate all targets for that component
      # into a single export set named "<component>-targets" to avoid exporting
      # the same dependency multiple times across different export sets.
      if(_TCP_COMPONENT)
        set(_tcp_export_name "${_TCP_COMPONENT}-targets")
        set(_tcp_export_file "${_TCP_COMPONENT}-targets.cmake")
        set(_tcp_export_dest "lib/cmake/${target}")
      else()
        set(_tcp_export_name "${target}_export")
        set(_tcp_export_file "${target}-export.cmake")
        set(_tcp_export_dest "lib/cmake/${target}")
      endif()

      install(
        TARGETS ${target}
        EXPORT ${_tcp_export_name}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
      install(
        EXPORT ${_tcp_export_name}
        FILE ${_tcp_export_file}
        NAMESPACE ${_TCP_NAMESPACE}::
        DESTINATION ${_tcp_export_dest})
      export(
        EXPORT ${_tcp_export_name}
        NAMESPACE ${_TCP_NAMESPACE}::
        FILE ${_tcp_export_file})

      set(_tcp_config_dir "${CMAKE_CURRENT_BINARY_DIR}/${target}_cmake")
      file(MAKE_DIRECTORY "${_tcp_config_dir}")

      set(_tcp_config_file "${_tcp_config_dir}/${target}Config.cmake")
      file(WRITE "${_tcp_config_file}" "include(CMakeFindDependencyMacro)\n")
      _tesseract_normalize_dependency_entries(_tcp_dependencies ${_TCP_DEPENDENCIES})
      foreach(_tcp_dependency IN LISTS _tcp_dependencies)
        if(_tcp_dependency)
          file(APPEND "${_tcp_config_file}" "find_dependency(${_tcp_dependency})\n")
        endif()
      endforeach()
      file(APPEND
        "${_tcp_config_file}"
        "include(\"\${CMAKE_CURRENT_LIST_DIR}/${target}-export.cmake\")\n")

      set(_tcp_extra_files "")
      foreach(_tcp_extra IN LISTS _TCP_CFG_EXTRAS)
        if(NOT _tcp_extra)
          continue()
        endif()
        if(NOT IS_ABSOLUTE "${_tcp_extra}")
          set(_tcp_extra_source "${_tcp_target_source_dir}/${_tcp_extra}")
        else()
          set(_tcp_extra_source "${_tcp_extra}")
        endif()
        if(NOT EXISTS "${_tcp_extra_source}")
          message(FATAL_ERROR "configure_package could not locate extra file ${_tcp_extra_source}")
        endif()
        get_filename_component(_tcp_extra_filename "${_tcp_extra_source}" NAME)
        set(_tcp_extra_destination "${_tcp_config_dir}/${_tcp_extra_filename}")
        configure_file("${_tcp_extra_source}" "${_tcp_extra_destination}" COPYONLY)
        list(APPEND _tcp_extra_files "${_tcp_extra_destination}")
        file(APPEND
          "${_tcp_config_file}"
          "include(\"\${CMAKE_CURRENT_LIST_DIR}/${_tcp_extra_filename}\")\n")
      endforeach()

      set(_tcp_target_version "")
      get_target_property(_tcp_target_version ${target} VERSION)
      if(NOT _tcp_target_version OR _tcp_target_version STREQUAL "NOTFOUND")
        set(_tcp_target_version "${PROJECT_VERSION}")
      endif()
      if(NOT _tcp_target_version)
        set(_tcp_target_version "0.0.0")
      endif()

      set(_tcp_config_version_file "${_tcp_config_dir}/${target}ConfigVersion.cmake")
      write_basic_package_version_file(
        "${_tcp_config_version_file}"
        VERSION "${_tcp_target_version}"
        COMPATIBILITY SameMajorVersion)

      set(_tcp_install_files "${_tcp_config_file}" "${_tcp_config_version_file}")
      if(_tcp_extra_files)
        list(APPEND _tcp_install_files ${_tcp_extra_files})
      endif()
      install(FILES ${_tcp_install_files} DESTINATION ${_tcp_export_dest})

      if(COMMAND ament_export_targets)
        if(_TCP_NAMESPACE)
          if(_tcp_has_library_target)
            ament_export_targets(
              ${_tcp_export_name}
              HAS_LIBRARY_TARGET
              NAMESPACE ${_TCP_NAMESPACE}::)
          else()
            ament_export_targets(${_tcp_export_name} NAMESPACE ${_TCP_NAMESPACE}::)
          endif()
        else()
          ament_export_targets(${_tcp_export_name})
        endif()
      endif()
    endif()
  endforeach()
endfunction()
