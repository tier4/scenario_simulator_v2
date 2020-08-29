#
# Copyright 2015 Ettus Research LLC
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# - Find libgps
# Find the Gpsd includes and client library
# This module defines
#  LIBGPS_INCLUDE_DIR, where to find gps.h
#  LIBGPS_LIBRARIES, the libraries needed by a GPSD client.
#  LIBGPS_FOUND, If false, do not try to use GPSD.
# also defined, but not for general use are
#  LIBGPS_LIBRARY, where to find the GPSD library.

include(FindPkgConfig)
pkg_check_modules(PC_GPSD "libgps")

if(PC_GPSD_FOUND)
    find_path(
        LIBGPS_INCLUDE_DIR
        NAMES gps.h
        HINTS ${PC_GPSD_INCLUDE_DIR})

    set(LIBGPS_NAMES ${LIBGPS_NAMES} gps)

    find_library(
        LIBGPS_LIBRARY
        NAMES ${LIBGPS_NAMES}
        HINTS ${PC_GPSD_LIBDIR})
endif(PC_GPSD_FOUND)

# handle the QUIETLY and REQUIRED arguments and set LIBGPS_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBGPS DEFAULT_MSG LIBGPS_LIBRARY LIBGPS_INCLUDE_DIR)

if(LIBGPS_FOUND)
    set(LIBGPS_LIBRARIES ${LIBGPS_LIBRARY})
endif(LIBGPS_FOUND)

mark_as_advanced(LIBGPS_LIBRARY LIBGPS_INCLUDE_DIR)
