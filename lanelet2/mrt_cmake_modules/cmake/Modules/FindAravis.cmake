find_package(PkgConfig REQUIRED)
include(FindPkgConfig)
pkg_check_modules(PC_ARAVIS REQUIRED aravis-0.6)

set(Aravis_INCLUDE_DIRS ${PC_ARAVIS_INCLUDE_DIRS})

# Convert library paths to absolute ones.
foreach(LIB ${PC_ARAVIS_LIBRARIES})
    find_library(LIBRARY_ABS_${LIB} ${LIB} PATHS ${PC_ARAVIS_LIBRARY_DIRS})
    list(APPEND Aravis_LIBRARIES "${LIBRARY_ABS_${LIB}}")
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Aravis DEFAULT_MSG PC_ARAVIS_LIBRARIES PC_ARAVIS_LIBRARY_DIRS PC_ARAVIS_INCLUDE_DIRS)

mark_as_advanced(Aravis_LIBRARIES Aravis_INCLUDE_DIRS)
