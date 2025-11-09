# Recreate upstream CMake logic for Bullet collision manager and
# provide a fallback definition of the find_bullet() helper when the
# ros_industrial_cmake_boilerplate module is unavailable. The overlay
# is copied into the workspace before colcon builds the upstream
# package, so this file augments the upstream version.

cmake_minimum_required(VERSION 3.10)

find_package(tesseract_common REQUIRED CONFIG)
find_package(tesseract_geometry REQUIRED CONFIG)
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

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

set(TESSERACT_TINYXML2_LIBRARIES "")
unset(TESSERACT_TINYXML2_TARGET)

if(TARGET tinyxml2::tinyxml2)
  set(TESSERACT_TINYXML2_TARGET tinyxml2::tinyxml2)
elseif(TARGET TinyXML2::TinyXML2)
  set(TESSERACT_TINYXML2_TARGET TinyXML2::TinyXML2)
elseif(TARGET tinyxml2_vendor::tinyxml2_vendor)
  set(TESSERACT_TINYXML2_TARGET tinyxml2_vendor::tinyxml2_vendor)
endif()

if(NOT TESSERACT_TINYXML2_TARGET)
  find_library(TINYXML2_LIB tinyxml2 REQUIRED)
  set(TESSERACT_TINYXML2_TARGET "${TINYXML2_LIB}")
endif()

if(TESSERACT_TINYXML2_TARGET)
  if(COMMAND _tesseract_collect_tinyxml2_libraries)
    _tesseract_collect_tinyxml2_libraries("${TESSERACT_TINYXML2_TARGET}" TESSERACT_TINYXML2_LIBRARIES)
  endif()

  if(NOT TESSERACT_TINYXML2_LIBRARIES)
    list(APPEND TESSERACT_TINYXML2_LIBRARIES "${TESSERACT_TINYXML2_TARGET}")
  endif()
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

if(NOT COMMAND find_bullet)
  function(_tesseract_bullet_get_target_install_prefix target out_var)
    set(_location "")

    get_target_property(_location "${target}" IMPORTED_LOCATION)
    if(NOT _location)
      get_target_property(_imported_configs "${target}" IMPORTED_CONFIGURATIONS)
      foreach(_config IN LISTS _imported_configs)
        if(_location)
          break()
        endif()
        string(TOUPPER "${_config}" _config_upper)
        get_target_property(_candidate "${target}" "IMPORTED_LOCATION_${_config_upper}")
        if(NOT _candidate)
          get_target_property(_candidate "${target}" "IMPORTED_IMPLIB_${_config_upper}")
        endif()
        if(_candidate)
          set(_location "${_candidate}")
        endif()
      endforeach()
    endif()

    if(NOT _location)
      set(${out_var} "" PARENT_SCOPE)
      return()
    endif()

    list(GET _location 0 _resolved_location)
    if(NOT IS_ABSOLUTE "${_resolved_location}")
      get_filename_component(_resolved_location "${_resolved_location}" ABSOLUTE)
    endif()

    get_filename_component(_lib_dir "${_resolved_location}" DIRECTORY)
    set(_search_dir "${_lib_dir}")
    set(_previous "")
    while(NOT _search_dir STREQUAL _previous)
      if(IS_DIRECTORY "${_search_dir}/include")
        set(${out_var} "${_search_dir}" PARENT_SCOPE)
        return()
      endif()
      set(_previous "${_search_dir}")
      get_filename_component(_search_dir "${_search_dir}/.." ABSOLUTE)
    endwhile()

    set(${out_var} "${_lib_dir}" PARENT_SCOPE)
  endfunction()

  function(_tesseract_bullet_find_prefix_from_libraries out_var)
    set(_libs "${ARGN}")
    foreach(_lib IN LISTS _libs)
      if(_lib STREQUAL "debug" OR _lib STREQUAL "optimized" OR _lib STREQUAL "general")
        continue()
      endif()
      if(IS_ABSOLUTE "${_lib}" AND EXISTS "${_lib}")
        get_filename_component(_lib_dir "${_lib}" DIRECTORY)
        set(_search_dir "${_lib_dir}")
        set(_previous "")
        while(NOT _search_dir STREQUAL _previous)
          if(IS_DIRECTORY "${_search_dir}/include")
            set(${out_var} "${_search_dir}" PARENT_SCOPE)
            return()
          endif()
          set(_previous "${_search_dir}")
          get_filename_component(_search_dir "${_search_dir}/.." ABSOLUTE)
        endwhile()
        set(${out_var} "${_lib_dir}" PARENT_SCOPE)
        return()
      endif()
    endforeach()
    set(${out_var} "" PARENT_SCOPE)
  endfunction()

  function(_tesseract_bullet_normalize_includes out_var base_dir)
    set(_base_dir "${base_dir}")
    set(_normalized)
    foreach(_entry IN LISTS ARGN)
      if(_entry MATCHES "^\$<")
        list(APPEND _normalized "${_entry}")
      elseif(IS_ABSOLUTE "${_entry}")
        list(APPEND _normalized "${_entry}")
      elseif(_base_dir)
        get_filename_component(_abs "${_base_dir}/${_entry}" ABSOLUTE)
        list(APPEND _normalized "${_abs}")
      else()
        get_filename_component(_abs "${_entry}" ABSOLUTE)
        list(APPEND _normalized "${_abs}")
      endif()
    endforeach()
    if(_normalized)
      list(REMOVE_DUPLICATES _normalized)
    endif()
    set(${out_var} "${_normalized}" PARENT_SCOPE)
  endfunction()

  function(find_bullet)
    set(HACD_LIBRARY "")

    # Prefer the imported target exported by bullet's CONFIG package if it
    # is available (Ubuntu 22.04+).
    find_package(Bullet CONFIG QUIET)

    if(Bullet_FOUND)
      if(NOT TARGET Bullet3::Bullet)
        if(TARGET Bullet::Bullet)
          add_library(Bullet3::Bullet INTERFACE IMPORTED)
          set_target_properties(Bullet3::Bullet PROPERTIES INTERFACE_LINK_LIBRARIES Bullet::Bullet)

          get_target_property(_bullet_inc Bullet::Bullet INTERFACE_INCLUDE_DIRECTORIES)
          if(_bullet_inc)
            _tesseract_bullet_get_target_install_prefix(Bullet::Bullet _bullet_prefix)
            _tesseract_bullet_normalize_includes(_normalized_inc "${_bullet_prefix}" ${_bullet_inc})
            if(_normalized_inc)
              set_target_properties(Bullet3::Bullet PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_normalized_inc}")
            endif()
          endif()
        else()
          # Some distributions export individual bullet component targets
          # instead of the umbrella Bullet::Bullet library.
          set(_bullet_targets)
          foreach(component BulletDynamics BulletCollision LinearMath)
            if(TARGET Bullet::${component})
              list(APPEND _bullet_targets Bullet::${component})
            elseif(TARGET Bullet::${component}_Static)
              list(APPEND _bullet_targets Bullet::${component}_Static)
            endif()
          endforeach()

          if(_bullet_targets)
            add_library(Bullet3::Bullet INTERFACE IMPORTED)
            set_target_properties(Bullet3::Bullet PROPERTIES INTERFACE_LINK_LIBRARIES "${_bullet_targets}")

            set(_include_dirs)
            foreach(tgt IN LISTS _bullet_targets)
              get_target_property(_inc "${tgt}" INTERFACE_INCLUDE_DIRECTORIES)
              if(_inc)
                _tesseract_bullet_get_target_install_prefix("${tgt}" _component_prefix)
                _tesseract_bullet_normalize_includes(_component_includes "${_component_prefix}" ${_inc})
                if(_component_includes)
                  list(APPEND _include_dirs "${_component_includes}")
                endif()
              endif()
            endforeach()

            if(_include_dirs)
              list(REMOVE_DUPLICATES _include_dirs)
              set_target_properties(Bullet3::Bullet PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_include_dirs}")
            endif()
          endif()
        endif()
      endif()
    endif()

    if(NOT TARGET Bullet3::Bullet)
      # Fall back to the traditional FindBullet module. This is the path
      # taken on older Ubuntu releases where Bullet does not ship a config
      # file. We create an imported target that mimics the modern
      # Bullet3::Bullet interface expected by downstream code.
      find_package(Bullet REQUIRED)

      if(BULLET_LIBRARIES)
        add_library(Bullet3::Bullet INTERFACE IMPORTED)
        _tesseract_bullet_find_prefix_from_libraries(_bullet_prefix ${BULLET_LIBRARIES})
        _tesseract_bullet_normalize_includes(_bullet_includes "${_bullet_prefix}" ${BULLET_INCLUDE_DIRS})
        set_target_properties(Bullet3::Bullet PROPERTIES
                              INTERFACE_INCLUDE_DIRECTORIES "${_bullet_includes}"
                              INTERFACE_LINK_LIBRARIES "${BULLET_LIBRARIES}")
      else()
        message(FATAL_ERROR "Failed to locate Bullet libraries while configuring tesseract_collision")
      endif()
    endif()

    # Try to locate the HACD convex decomposition library. It is optional,
    # so populate the variable only when a library is discovered.
    find_library(_hacd_library NAMES HACD hacd)
    if(_hacd_library)
      set(HACD_LIBRARY "${_hacd_library}" PARENT_SCOPE)
    else()
      set(HACD_LIBRARY "" PARENT_SCOPE)
    endif()
  endfunction()
endif()

find_bullet()

# Create target for Bullet implementation
add_library(
  ${PROJECT_NAME}_bullet
  src/bullet_cast_bvh_manager.cpp
  src/bullet_cast_simple_manager.cpp
  src/bullet_collision_shape_cache.cpp
  src/bullet_discrete_bvh_manager.cpp
  src/bullet_discrete_simple_manager.cpp
  src/bullet_utils.cpp
  src/convex_hull_utils.cpp
  src/tesseract_compound_collision_algorithm.cpp
  src/tesseract_compound_compound_collision_algorithm.cpp
  src/tesseract_collision_configuration.cpp
  src/tesseract_convex_convex_algorithm.cpp
  src/tesseract_gjk_pair_detector.cpp)
target_link_libraries(
  ${PROJECT_NAME}_bullet
  PUBLIC
    ${PROJECT_NAME}_core
    Eigen3::Eigen
    Bullet3::Bullet
    ${TESSERACT_GEOM_TARGET}
    console_bridge::console_bridge
    octomap
    octomath
    ${TESSERACT_TINYXML2_TARGET})
target_compile_options(${PROJECT_NAME}_bullet PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_bullet PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_bullet PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_bullet PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_bullet ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(${PROJECT_NAME}_bullet PRIVATE ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_bullet
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../core/include>"
         "$<INSTALL_INTERFACE:include>")

# Create target for Bullet implementation
add_library(${PROJECT_NAME}_bullet_factories src/bullet_factories.cpp)
target_link_libraries(${PROJECT_NAME}_bullet_factories PUBLIC ${PROJECT_NAME}_bullet)
target_compile_options(${PROJECT_NAME}_bullet_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_bullet_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_bullet_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_bullet_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_bullet_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(${PROJECT_NAME}_bullet_factories PRIVATE ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_bullet_factories
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../core/include>"
         "$<INSTALL_INTERFACE:include>")

# Convex decomposition libraries
if(HACD_LIBRARY)
  add_library(${PROJECT_NAME}_hacd_convex_decomposition src/convex_decomposition_hacd.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_hacd_convex_decomposition
    PUBLIC
      ${PROJECT_NAME}_bullet
      Eigen3::Eigen
      Bullet3::Bullet
      ${TESSERACT_GEOM_TARGET}
      console_bridge::console_bridge
      ${HACD_LIBRARY})
  target_compile_options(${PROJECT_NAME}_hacd_convex_decomposition PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
  target_compile_options(${PROJECT_NAME}_hacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${PROJECT_NAME}_hacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
  target_cxx_version(${PROJECT_NAME}_hacd_convex_decomposition PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_clang_tidy(${PROJECT_NAME}_hacd_convex_decomposition ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_code_coverage(${PROJECT_NAME}_hacd_convex_decomposition PRIVATE ALL EXCLUDE ${COVERAGE_EXCLUDE} ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  target_include_directories(
    ${PROJECT_NAME}_hacd_convex_decomposition
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
           "$<INSTALL_INTERFACE:include>")
  install_targets(TARGETS ${PROJECT_NAME}_hacd_convex_decomposition COMPONENT bullet)
endif()

option(TESSERACT_COLLISION_BUILD_CREATE_CONVEX_HULL "Build the create_convex_hull utility" ON)

if(NOT MSVC AND TESSERACT_COLLISION_BUILD_CREATE_CONVEX_HULL)
  # Create target for creating convex hulls from meshes
  add_executable(create_convex_hull src/create_convex_hull.cpp)
  target_link_libraries(
    create_convex_hull
    PUBLIC
      ${PROJECT_NAME}_core
      ${PROJECT_NAME}_bullet
      Bullet3::Bullet
      Boost::boost
      Boost::program_options
      Eigen3::Eigen
      ${TESSERACT_COMMON_TARGET}
      ${TESSERACT_GEOM_TARGET}
      console_bridge::console_bridge
      octomap
      octomath
      ${TESSERACT_TINYXML2_TARGET}
      ${TESSERACT_TINYXML2_LIBRARIES})
  target_compile_options(create_convex_hull PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE} ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(create_convex_hull PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_cxx_version(create_convex_hull PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_clang_tidy(create_convex_hull ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_include_directories(
    create_convex_hull
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../core/include>")

  list(APPEND PACKAGE_LIBRARIES create_convex_hull)

  install_targets(TARGETS create_convex_hull COMPONENT bullet)
endif()

# Add factory library so contact_managers_factory can find these factories by defauult
set(CONTACT_MANAGERS_PLUGINS ${CONTACT_MANAGERS_PLUGINS} "${PROJECT_NAME}_bullet_factories" PARENT_SCOPE)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
        COMPONENT bullet
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN "*.inl"
        PATTERN ".svn" EXCLUDE)

# Plugin should not be export only installed
install(TARGETS ${PROJECT_NAME}_bullet_factories
        COMPONENT bullet
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

configure_component(COMPONENT bullet
                    NAMESPACE tesseract
                    TARGETS ${PROJECT_NAME}_bullet
                    DEPENDENCIES "tesseract_collision COMPONENTS core"
                    CFG_EXTRAS cmake/bullet-extras.cmake)

if(TESSERACT_PACKAGE)
  cpack_component(COMPONENT bullet
                  VERSION ${pkg_extracted_version}
                  DESCRIPTION "Tesseract Collision bullet components"
                  COMPONENT_DEPENDS core
                  LINUX_DEPENDS "libbullet-dev" "libbullet-extras-dev"
                  WINDOWS_DEPENDS "libbullet-dev" "libbullet-extras-dev")
endif()
