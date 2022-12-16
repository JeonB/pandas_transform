# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_quick_start_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED quick_start_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(quick_start_FOUND FALSE)
  elseif(NOT quick_start_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(quick_start_FOUND FALSE)
  endif()
  return()
endif()
set(_quick_start_CONFIG_INCLUDED TRUE)

# output package information
if(NOT quick_start_FIND_QUIETLY)
  message(STATUS "Found quick_start: 0.1.0 (${quick_start_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'quick_start' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${quick_start_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(quick_start_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake;ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${quick_start_DIR}/${_extra}")
endforeach()
