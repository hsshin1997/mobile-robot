# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_bot_one_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED bot_one_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(bot_one_FOUND FALSE)
  elseif(NOT bot_one_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(bot_one_FOUND FALSE)
  endif()
  return()
endif()
set(_bot_one_CONFIG_INCLUDED TRUE)

# output package information
if(NOT bot_one_FIND_QUIETLY)
  message(STATUS "Found bot_one: 0.0.0 (${bot_one_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'bot_one' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT bot_one_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(bot_one_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${bot_one_DIR}/${_extra}")
endforeach()
