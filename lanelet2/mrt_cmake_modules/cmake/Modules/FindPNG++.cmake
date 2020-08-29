# - Find the PNG++ includes
#
# This module searches libpng++, the library for working with PNG images.
#
# It defines the following variables
#  PNG++_INCLUDE_DIRS
#  PNG_FOUND, If false, do not try to use PNG.
#  PNG_VERSION_STRING - the version of the PNG library found (since CMake 2.8.8)
#
# Since PNG depends on the ZLib compression library, none of the above will be
# defined unless ZLib can be found.

#=============================================================================
# Copyright 2002-2009 Kitware, Inc.
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)

if(PNG_FIND_QUIETLY)
    set(_FIND_ZLIB_ARG QUIET)
endif()
find_package(ZLIB ${_FIND_ZLIB_ARG})
find_package(PNG REQUIRED)

if(ZLIB_FOUND)
    find_path(PNG++_INCLUDE_DIR png.hpp /usr/local/include/png++ /usr/include/png++)

    set(PNG++_LIBRARIES ${PNG_LIBRARIES})

    if(PNG++_INCLUDE_DIR)
        set(PNG++_INCLUDE_DIRS ${PNG++_INCLUDE_DIR} ${ZLIB_INCLUDE_DIR})
        unset(PNG++_INCLUDE_DIR)
    endif()
endif()

# handle the QUIETLY and REQUIRED arguments and set PNG_FOUND to TRUE if
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PNG++ REQUIRED_VARS PNG++_INCLUDE_DIRS)
mark_as_advanced(PNG++_INCLUDE_DIRS)
