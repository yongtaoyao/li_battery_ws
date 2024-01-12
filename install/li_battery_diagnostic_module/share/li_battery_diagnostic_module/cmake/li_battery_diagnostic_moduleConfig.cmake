# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_li_battery_diagnostic_module_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED li_battery_diagnostic_module_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(li_battery_diagnostic_module_FOUND FALSE)
  elseif(NOT li_battery_diagnostic_module_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(li_battery_diagnostic_module_FOUND FALSE)
  endif()
  return()
endif()
set(_li_battery_diagnostic_module_CONFIG_INCLUDED TRUE)

# output package information
if(NOT li_battery_diagnostic_module_FIND_QUIETLY)
  message(STATUS "Found li_battery_diagnostic_module: 0.0.1 (${li_battery_diagnostic_module_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'li_battery_diagnostic_module' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${li_battery_diagnostic_module_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(li_battery_diagnostic_module_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${li_battery_diagnostic_module_DIR}/${_extra}")
endforeach()
