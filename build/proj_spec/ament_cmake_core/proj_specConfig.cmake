# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_proj_spec_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED proj_spec_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(proj_spec_FOUND FALSE)
  elseif(NOT proj_spec_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(proj_spec_FOUND FALSE)
  endif()
  return()
endif()
set(_proj_spec_CONFIG_INCLUDED TRUE)

# output package information
if(NOT proj_spec_FIND_QUIETLY)
  message(STATUS "Found proj_spec: 0.0.0 (${proj_spec_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'proj_spec' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${proj_spec_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(proj_spec_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${proj_spec_DIR}/${_extra}")
endforeach()
