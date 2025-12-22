include_guard(GLOBAL)

function(tesseract_task_composer_ensure_boost_stacktrace_target)
  if(NOT Boost_FOUND)
    find_package(Boost REQUIRED COMPONENTS filesystem system)
  endif()

  # Attempt to locate a usable stacktrace target. If none exist after a quiet lookup,
  # create a safe interface target to avoid configuration failures.
  set(_stacktrace_candidates
      Boost::stacktrace_backtrace
      Boost::stacktrace_addr2line
      Boost::stacktrace_basic
      Boost::stacktrace_noop)

  set(_stacktrace_target "")
  foreach(_candidate IN LISTS _stacktrace_candidates)
    if(TARGET "${_candidate}")
      set(_stacktrace_target "${_candidate}")
      break()
    endif()
  endforeach()

  if(NOT _stacktrace_target)
    find_package(Boost QUIET COMPONENTS stacktrace_backtrace stacktrace_addr2line stacktrace_basic stacktrace_noop)
    foreach(_candidate IN LISTS _stacktrace_candidates)
      if(TARGET "${_candidate}")
        set(_stacktrace_target "${_candidate}")
        break()
      endif()
    endforeach()
  endif()

  if(_stacktrace_target AND NOT _stacktrace_target STREQUAL "Boost::stacktrace_backtrace"
     AND NOT TARGET Boost::stacktrace_backtrace)
    add_library(Boost::stacktrace_backtrace ALIAS ${_stacktrace_target})
  elseif(NOT _stacktrace_target AND NOT TARGET Boost::stacktrace_backtrace)
    add_library(Boost::stacktrace_backtrace INTERFACE IMPORTED)
    if(TARGET Boost::boost)
      set_property(TARGET Boost::stacktrace_backtrace APPEND PROPERTY INTERFACE_LINK_LIBRARIES Boost::boost)
    endif()
    if(CMAKE_DL_LIBS)
      set_property(TARGET Boost::stacktrace_backtrace APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${CMAKE_DL_LIBS})
    endif()
  endif()
endfunction()
