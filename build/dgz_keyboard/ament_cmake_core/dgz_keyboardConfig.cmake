# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_dgz_keyboard_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED dgz_keyboard_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(dgz_keyboard_FOUND FALSE)
  elseif(NOT dgz_keyboard_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(dgz_keyboard_FOUND FALSE)
  endif()
  return()
endif()
set(_dgz_keyboard_CONFIG_INCLUDED TRUE)

# output package information
if(NOT dgz_keyboard_FIND_QUIETLY)
  message(STATUS "Found dgz_keyboard: 0.0.0 (${dgz_keyboard_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'dgz_keyboard' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT dgz_keyboard_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(dgz_keyboard_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${dgz_keyboard_DIR}/${_extra}")
endforeach()