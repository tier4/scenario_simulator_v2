# This file lets a conan build look like a catkin build for the rest of the tools. This is done by defining all
# the variables that the rest of the build system would expect
set(catkin_FOUND TRUE)
message(STATUS "testing: ${CATKIN_ENABLE_TESTING}")

conan_define_targets()
target_link_libraries(${PROJECT_NAME}::auto_deps_export INTERFACE ${CONAN_TARGETS})
if(CONAN_USER_PYTHON_DEV_CONFIG_python_exec)
    set(PYTHON_VERSION ${CONAN_USER_PYTHON_DEV_CONFIG_python_version})
    set(PYTHON_EXECUTABLE ${CONAN_USER_PYTHON_DEV_CONFIG_python_exec})
endif()

set(CATKIN_DEVEL_PREFIX ${CMAKE_CURRENT_BINARY_DIR})
set(${PROJECT_NAME}_CATKIN_PACKAGE True) # avoids that export cmake files are generated, conan does this

function(catkin_package_xml)

endfunction()
