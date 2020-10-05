set(PACKAGE_HEADER_FILES leptonica/allheaders.h)
set(PACKAGE_LIBRARIES lept)

find_path(Leptonica_INCLUDE_DIR NAMES ${PACKAGE_HEADER_FILES})
find_library(Leptonica_LIBRARIES NAMES ${PACKAGE_LIBRARIES})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    Leptonica
    FOUND_VAR Leptonica_FOUND
    REQUIRED_VARS Leptonica_INCLUDE_DIR Leptonica_LIBRARIES)
