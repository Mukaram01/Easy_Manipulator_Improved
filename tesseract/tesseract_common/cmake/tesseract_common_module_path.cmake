# Ensure consumers of tesseract_common can locate the custom CMake modules
# that ship with the package (e.g. FindTinyXML2.cmake).  This mirrors the
# behaviour of the upstream project which appends its module directory to
# CMAKE_MODULE_PATH from its exported package configuration.
set(_tesseract_common_prefix "${CMAKE_CURRENT_LIST_DIR}/../..")
get_filename_component(_tesseract_common_prefix "${_tesseract_common_prefix}" ABSOLUTE)
set(_tesseract_common_module_dir "${_tesseract_common_prefix}/lib/cmake/tesseract_common")
if(EXISTS "${_tesseract_common_module_dir}" AND NOT "${_tesseract_common_module_dir}" IN_LIST CMAKE_MODULE_PATH)
  list(APPEND CMAKE_MODULE_PATH "${_tesseract_common_module_dir}")
endif()
unset(_tesseract_common_prefix)
unset(_tesseract_common_module_dir)
