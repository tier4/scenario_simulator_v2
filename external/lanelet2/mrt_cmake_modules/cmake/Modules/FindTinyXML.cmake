# - Find TinyXML
# Find the native TinyXML includes and library
#
#   TINYXML_FOUND       - True if TinyXML found.
#   TINYXML_INCLUDE_DIR - where to find tinyxml.h, etc.
#   TINYXML_LIBRARIES   - List of libraries when using TinyXML.
#

if(TINYXML_INCLUDE_DIR)
    # Already in cache, be silent
    set(TinyXML_FIND_QUIETLY TRUE)
endif(TINYXML_INCLUDE_DIR)

find_path(TINYXML_INCLUDE_DIR "tinyxml.h" PATH_SUFFIXES "tinyxml")

find_library(
    TINYXML_LIBRARIES
    NAMES "tinyxml"
    PATH_SUFFIXES "tinyxml")

# handle the QUIETLY and REQUIRED arguments and set TINYXML_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args("TinyXML" DEFAULT_MSG TINYXML_INCLUDE_DIR TINYXML_LIBRARIES)

mark_as_advanced(TINYXML_INCLUDE_DIR TINYXML_LIBRARIES)
