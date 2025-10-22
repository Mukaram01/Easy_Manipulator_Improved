# Locate the TinyXML2 library and create an imported target compatible with the
# usage expected by Tesseract.

find_path(TinyXML2_INCLUDE_DIR tinyxml2.h
          PATH_SUFFIXES tinyxml2
          DOC "Directory containing tinyxml2.h")

find_library(TinyXML2_LIBRARY NAMES tinyxml2
             DOC "TinyXML2 library")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TinyXML2 DEFAULT_MSG
                                  TinyXML2_INCLUDE_DIR TinyXML2_LIBRARY)

if(TinyXML2_FOUND)
  set(TinyXML2_INCLUDE_DIRS ${TinyXML2_INCLUDE_DIR})
  set(TinyXML2_LIBRARIES ${TinyXML2_LIBRARY})

  if(NOT TARGET tinyxml2::tinyxml2)
    add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
    set_target_properties(tinyxml2::tinyxml2 PROPERTIES
                          IMPORTED_LOCATION "${TinyXML2_LIBRARY}"
                          INTERFACE_INCLUDE_DIRECTORIES "${TinyXML2_INCLUDE_DIR}")
  endif()
endif()

mark_as_advanced(TinyXML2_INCLUDE_DIR TinyXML2_LIBRARY)
