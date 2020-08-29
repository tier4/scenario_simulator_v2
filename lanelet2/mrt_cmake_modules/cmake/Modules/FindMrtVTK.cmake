# find package component
if(MrtVTK_FIND_REQUIRED)
    find_package(VTK REQUIRED)
elseif(MrtVTK_FIND_QUIETLY)
    find_package(VTK QUIET)
else()
    find_package(VTK)
endif()

include(${VTK_USE_FILE})
