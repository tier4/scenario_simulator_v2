set(PACKAGE_HEADER_FILES cereal/cereal.hpp)

find_path(Cereal_INCLUDE_DIR NAMES ${PACKAGE_HEADER_FILES})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    Cereal
    FOUND_VAR Cereal_FOUND
    REQUIRED_VARS Cereal_INCLUDE_DIR)

add_definitions(-DHAS_CEREAL)
