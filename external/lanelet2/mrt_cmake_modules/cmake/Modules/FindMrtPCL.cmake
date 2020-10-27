# protect agains multiple inclusion
if(TARGET mrt_pcl::pcl)
    return()
endif()

cmake_policy(PUSH)
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

# find package component
if(Mrtpcl_FIND_REQUIRED)
    find_package(PCL QUIET REQUIRED)
elseif(Mrtpcl_FIND_QUIETLY)
    find_package(PCL QUIET)
else()
    find_package(PCL QUIET)
endif()

add_library(mrt_pcl::pcl INTERFACE IMPORTED)

# Copied from FindAutoDeps.cmake
function(_cleanup_libraries var_name_libs)
    # replace "debug", "general" and "optimized" keywords in the libraries list with generator expressions
    list(LENGTH ${var_name_libs} size)
    foreach(idx RANGE ${size})
        if(${idx} EQUAL ${size})
            continue()
        endif()
        list(GET ${var_name_libs} ${idx} statement)
        if(${statement} STREQUAL "debug")
            math(EXPR next ${idx}+1)
            list(GET ${var_name_libs} ${next} lib)
            list(REMOVE_AT ${var_name_libs} ${next})
            list(INSERT ${var_name_libs} ${next} "$<$<CONFIG:DEBUG>:${lib}>")
        elseif(${statement} STREQUAL "optimized")
            math(EXPR next ${idx}+1)
            list(GET ${var_name_libs} ${next} lib)
            list(REMOVE_AT ${var_name_libs} ${next})
            list(INSERT ${var_name_libs} ${next} "$<$<NOT:$<CONFIG:DEBUG>>:${lib}>")
        endif()
    endforeach()
    if(size)
        list(REMOVE_ITEM ${var_name_libs} debug optimized general)
    endif()
    set(${var_name_libs}
        ${${var_name_libs}}
        PARENT_SCOPE)
endfunction()

_cleanup_libraries(PCL_LIBRARIES)

# Add PCL_NO_PRECOMPILE as this resolves Eigen issues.
set_target_properties(
    mrt_pcl::pcl
    PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${PCL_INCLUDE_DIRS}"
               INTERFACE_LINK_DIRECTORIES "${PCL_LIBRARY_DIRS}"
               INTERFACE_LINK_LIBRARIES "${PCL_LIBRARIES}"
               INTERFACE_COMPILE_DEFINITIONS "PCL_NO_PRECOMPILE")

cmake_policy(POP)
