# Recreate upstream CMake logic for Bullet collision manager and
# provide a fallback definition of the find_bullet() helper when the
# ros_industrial_cmake_boilerplate module is unavailable. The overlay
# is copied into the workspace before colcon builds the upstream
# package, so this file augments the upstream version.

if(NOT COMMAND find_bullet)
  function(find_bullet)
    set(HACD_LIBRARY "")

    # Prefer the imported target exported by bullet's CONFIG package if it
    # is available (Ubuntu 22.04+).
    find_package(Bullet CONFIG QUIET)

    if(Bullet_FOUND)
      if(NOT TARGET Bullet3::Bullet)
        if(TARGET Bullet::Bullet)
          add_library(Bullet3::Bullet INTERFACE IMPORTED)
          set_target_properties(
            Bullet3::Bullet PROPERTIES
            INTERFACE_LINK_LIBRARIES Bullet::Bullet)

          get_target_property(_bullet_inc Bullet::Bullet INTERFACE_INCLUDE_DIRECTORIES)
          if(_bullet_inc)
            set_target_properties(Bullet3::Bullet PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_bullet_inc}")
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
            set_target_properties(
              Bullet3::Bullet PROPERTIES
              INTERFACE_LINK_LIBRARIES "${_bullet_targets}")

            set(_include_dirs)
            foreach(tgt IN LISTS _bullet_targets)
              get_target_property(_inc "${tgt}" INTERFACE_INCLUDE_DIRECTORIES)
              if(_inc)
                list(APPEND _include_dirs "${_inc}")
              endif()
            endforeach()

            if(_include_dirs)
              list(REMOVE_DUPLICATES _include_dirs)
              set_target_properties(
                Bullet3::Bullet PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_include_dirs}")
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
        set_target_properties(
          Bullet3::Bullet PROPERTIES
          INTERFACE_INCLUDE_DIRECTORIES "${BULLET_INCLUDE_DIRS}"
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
  PUBLIC ${PROJECT_NAME}_core
         Eigen3::Eigen
         Bullet3::Bullet
         tesseract::tesseract_geometry
         console_bridge::console_bridge
         octomap
         octomath)
target_compile_options(${PROJECT_NAME}_bullet PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_bullet PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_bullet PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_bullet PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_bullet ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  ${PROJECT_NAME}_bullet
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_bullet PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:include>")

# Create target for Bullet implementation
add_library(${PROJECT_NAME}_bullet_factories src/bullet_factories.cpp)
target_link_libraries(${PROJECT_NAME}_bullet_factories PUBLIC ${PROJECT_NAME}_bullet)
target_compile_options(${PROJECT_NAME}_bullet_factories PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_bullet_factories PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_bullet_factories PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_cxx_version(${PROJECT_NAME}_bullet_factories PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_clang_tidy(${PROJECT_NAME}_bullet_factories ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_code_coverage(
  ${PROJECT_NAME}_bullet_factories
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(
  ${PROJECT_NAME}_bullet_factories PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                          "$<INSTALL_INTERFACE:include>")

# Convex decomposition libraries
if(${HACD_LIBRARY})
  add_library(${PROJECT_NAME}_hacd_convex_decomposition src/convex_decomposition_hacd.cpp)
  target_link_libraries(
    ${PROJECT_NAME}_hacd_convex_decomposition
    PUBLIC ${PROJECT_NAME}_bullet
           Eigen3::Eigen
           Bullet3::Bullet
           tesseract::tesseract_geometry
           console_bridge::console_bridge
           ${HACD_LIBRARY})
  target_compile_options(${PROJECT_NAME}_hacd_convex_decomposition PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
  target_compile_options(${PROJECT_NAME}_hacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(${PROJECT_NAME}_hacd_convex_decomposition PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
  target_cxx_version(${PROJECT_NAME}_hacd_convex_decomposition PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_clang_tidy(${PROJECT_NAME}_hacd_convex_decomposition ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_code_coverage(
    ${PROJECT_NAME}_hacd_convex_decomposition
    PRIVATE
    ALL
    EXCLUDE ${COVERAGE_EXCLUDE}
    ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
  target_include_directories(
    ${PROJECT_NAME}_hacd_convex_decomposition PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                     "$<INSTALL_INTERFACE:include>")
  install_targets(TARGETS ${PROJECT_NAME}_hacd_convex_decomposition COMPONENT bullet)
endif()

if(NOT MSVC)
  # Create target for creating convex hulls from meshes
  add_executable(create_convex_hull src/create_convex_hull.cpp)
  target_link_libraries(
    create_convex_hull
    PUBLIC ${PROJECT_NAME}_core
           ${PROJECT_NAME}_bullet
           Bullet3::Bullet
           Boost::boost
           Boost::program_options
           Eigen3::Eigen
           tesseract::tesseract_common
           tesseract::tesseract_geometry
           console_bridge::console_bridge
           octomap
           octomath)
  target_compile_options(create_convex_hull PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                    ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
  target_compile_definitions(create_convex_hull PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
  target_cxx_version(create_convex_hull PRIVATE VERSION ${TESSERACT_CXX_VERSION})
  target_clang_tidy(create_convex_hull ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_include_directories(create_convex_hull PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

  list(APPEND PACKAGE_LIBRARIES create_convex_hull)

  install_targets(TARGETS create_convex_hull COMPONENT bullet)
endif()

# Add factory library so contact_managers_factory can find these factories by defauult
set(CONTACT_MANAGERS_PLUGINS ${CONTACT_MANAGERS_PLUGINS} "${PROJECT_NAME}_bullet_factories" PARENT_SCOPE)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  COMPONENT bullet
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN ".svn" EXCLUDE)

# Plugin should not be export only installed
install(
  TARGETS ${PROJECT_NAME}_bullet_factories
  COMPONENT bullet
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

configure_component(
  COMPONENT bullet
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_bullet
  DEPENDENCIES "tesseract_collision COMPONENTS core"
  CFG_EXTRAS cmake/bullet-extras.cmake)

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT bullet
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Collision bullet components"
    COMPONENT_DEPENDS core
    LINUX_DEPENDS "libbullet-dev" "libbullet-extras-dev"
    WINDOWS_DEPENDS "libbullet-dev" "libbullet-extras-dev")
endif()
