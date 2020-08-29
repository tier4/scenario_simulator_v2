set(PACKAGE_HEADER_FILES tesseract/apitypes.h)
set(PACKAGE_LIBRARIES tesseract)

find_path(Tesseract_INCLUDE_DIR NAMES ${PACKAGE_HEADER_FILES})
find_library(Tesseract_LIBRARIES NAMES ${PACKAGE_LIBRARIES})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    Tesseract
    FOUND_VAR Tesseract_FOUND
    REQUIRED_VARS Tesseract_INCLUDE_DIR Tesseract_LIBRARIES)
