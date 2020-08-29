# - Find NLopt
# Find the native NLopt includes and library
#
#  nlopt_INCLUDE_DIR - where to find nlopt.h, etc.
#  nlopt_LIBRARIES   - List of libraries when using nlopt.
#  nlopt_FOUND       - True if nlopt found.

if(nlopt_INCLUDE_DIR)
    # Already in cache, be silent
    set(nlopt_FIND_QUIETLY TRUE)
endif(nlopt_INCLUDE_DIR)

find_path(nlopt_INCLUDE_DIR nlopt.h)

set(nlopt_NAMES nlopt nlopt_cxx)
find_library(nlopt_LIBRARY NAMES ${nlopt_NAMES})

# handle the QUIETLY and REQUIRED arguments and set nlopt_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(nlopt DEFAULT_MSG nlopt_LIBRARY nlopt_INCLUDE_DIR)

if(nlopt_FOUND)
    set(nlopt_LIBRARIES ${nlopt_LIBRARY})
else(nlopt_FOUND)
    set(nlopt_LIBRARIES)
endif(nlopt_FOUND)

mark_as_advanced(nlopt_LIBRARY nlopt_INCLUDE_DIR)
