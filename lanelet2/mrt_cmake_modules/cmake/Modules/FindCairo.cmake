# - Try to find the cairo library
# Once done this will define
#
#  CAIRO_FOUND - system has cairo
#  CAIRO_INCLUDE_DIRS - the cairo include directory
#  CAIRO_LIBRARIES - Link these to use cairo
#
# Define CAIRO_MIN_VERSION for which version desired.
#

include(FindPkgConfig)

if(Cairo_FIND_REQUIRED)
    set(_pkgconfig_REQUIRED "REQUIRED")
else(Cairo_FIND_REQUIRED)
    set(_pkgconfig_REQUIRED "")
endif(Cairo_FIND_REQUIRED)

if(CAIRO_MIN_VERSION)
    pkg_search_module(CAIRO ${_pkgconfig_REQUIRED} cairo>=${CAIRO_MIN_VERSION})
else(CAIRO_MIN_VERSION)
    pkg_search_module(CAIRO ${_pkgconfig_REQUIRED} cairo)
endif(CAIRO_MIN_VERSION)

if(NOT CAIRO_FOUND AND NOT PKG_CONFIG_FOUND)
    find_path(CAIRO_INCLUDE_DIRS cairo.h)
    find_library(CAIRO_LIBRARIES cairo)

    # Report results
    if(CAIRO_LIBRARIES AND CAIRO_INCLUDE_DIRS)
        set(CAIRO_FOUND 1)
        if(NOT Cairo_FIND_QUIETLY)
            message(STATUS "Found Cairo: ${CAIRO_LIBRARIES}")
        endif(NOT Cairo_FIND_QUIETLY)
    else(CAIRO_LIBRARIES AND CAIRO_INCLUDE_DIRS)
        if(Cairo_FIND_REQUIRED)
            message(SEND_ERROR "Could not find Cairo")
        else(Cairo_FIND_REQUIRED)
            if(NOT Cairo_FIND_QUIETLY)
                message(STATUS "Could not find Cairo")
            endif(NOT Cairo_FIND_QUIETLY)
        endif(Cairo_FIND_REQUIRED)
    endif(CAIRO_LIBRARIES AND CAIRO_INCLUDE_DIRS)
endif(NOT CAIRO_FOUND AND NOT PKG_CONFIG_FOUND)

# Hide advanced variables from CMake GUIs
mark_as_advanced(CAIRO_LIBRARIES CAIRO_INCLUDE_DIRS)
