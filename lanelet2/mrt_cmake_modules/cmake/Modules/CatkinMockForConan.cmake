# This file lets a conan build look like a catkin build for the rest of the tools. This is done by defining all
# the variables that the rest of the build system would expect
set(catkin_FOUND TRUE)
message(STATUS "testing: ${CATKIN_ENABLE_TESTING}")

conan_define_targets()
target_link_libraries(${PROJECT_NAME}::auto_deps_export INTERFACE ${CONAN_TARGETS})

set(CATKIN_DEVEL_PREFIX ${CMAKE_CURRENT_BINARY_DIR})
set(${PROJECT_NAME}_CATKIN_PACKAGE True) # avoids that export cmake files are generated, conan does this

function(catkin_package)

endfunction()
function(catkin_package_xml)

endfunction()
function(catkin_install_python programs)
    cmake_parse_arguments(ARG "OPTIONAL" "DESTINATION" "" ${ARGN})
    if(ARG_OPTIONAL)
        set(optional_flag "OPTIONAL")
    endif()
    foreach(file ${ARG_UNPARSED_ARGUMENTS})
        install(
            PROGRAMS "${file}"
            DESTINATION "${ARG_DESTINATION}"
            ${optional_flag})
    endforeach()
endfunction()
