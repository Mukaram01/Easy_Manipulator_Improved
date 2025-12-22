include_guard(GLOBAL)

function(tesseract_task_composer_ensure_tinyxml2_target)
  # Tier A: Prefer the ROS vendor package which provides module mode find scripts and
  # normalizes targets when available.
  if(NOT TARGET tinyxml2::tinyxml2 AND NOT TARGET TinyXML2::TinyXML2)
    find_package(tinyxml2_vendor QUIET)
    find_package(TinyXML2 QUIET)
  endif()

  # Tier B: CONFIG mode when provided by the platform distribution.
  if(NOT TARGET tinyxml2::tinyxml2 AND NOT TARGET TinyXML2::TinyXML2)
    find_package(tinyxml2 CONFIG QUIET)
    find_package(TinyXML2 CONFIG QUIET)
  endif()

  # Tier C: Manual fallback to create an imported target from discovered headers and library.
  if(NOT TARGET tinyxml2::tinyxml2 AND NOT TARGET TinyXML2::TinyXML2)
    find_path(TINYXML2_INCLUDE_DIR NAMES tinyxml2.h)
    find_library(TINYXML2_LIBRARY NAMES tinyxml2)

    if(TINYXML2_INCLUDE_DIR AND TINYXML2_LIBRARY)
      add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
      set_target_properties(
        tinyxml2::tinyxml2
        PROPERTIES IMPORTED_LOCATION "${TINYXML2_LIBRARY}"
                   INTERFACE_INCLUDE_DIRECTORIES "${TINYXML2_INCLUDE_DIR}")
    endif()
  endif()

  # Normalize target names so downstream code can rely on either alias existing.
  if(TARGET TinyXML2::TinyXML2 AND NOT TARGET tinyxml2::tinyxml2)
    add_library(tinyxml2::tinyxml2 ALIAS TinyXML2::TinyXML2)
  elseif(TARGET tinyxml2::tinyxml2 AND NOT TARGET TinyXML2::TinyXML2)
    add_library(TinyXML2::TinyXML2 ALIAS tinyxml2::tinyxml2)
  endif()

  if(NOT TARGET tinyxml2::tinyxml2 AND NOT TARGET TinyXML2::TinyXML2)
    message(FATAL_ERROR "tinyxml2 target not available. Please install tinyxml2 or tinyxml2_vendor.")
  endif()
endfunction()
