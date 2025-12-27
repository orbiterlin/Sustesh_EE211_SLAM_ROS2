# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_EI_Final_Project_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED EI_Final_Project_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(EI_Final_Project_FOUND FALSE)
  elseif(NOT EI_Final_Project_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(EI_Final_Project_FOUND FALSE)
  endif()
  return()
endif()
set(_EI_Final_Project_CONFIG_INCLUDED TRUE)

# output package information
if(NOT EI_Final_Project_FIND_QUIETLY)
  message(STATUS "Found EI_Final_Project: 0.1.0 (${EI_Final_Project_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'EI_Final_Project' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${EI_Final_Project_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(EI_Final_Project_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${EI_Final_Project_DIR}/${_extra}")
endforeach()
