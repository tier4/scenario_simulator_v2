set(PACKAGE_HEADER_FILES metis.h)
set(PACKAGE_LIBRARIES metis)

find_path(Metis_INCLUDE_DIR NAMES ${PACKAGE_HEADER_FILES})
find_library(Metis_LIBRARIES NAMES ${PACKAGE_LIBRARIES})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    Metis
    FOUND_VAR Metis_FOUND
    REQUIRED_VARS Metis_INCLUDE_DIR Metis_LIBRARIES)
