# Detect Conan builds. In this case we don't have to do anything, conan already did the work for us.
if(CONAN_PACKAGE_NAME OR EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
    return()
endif()

#Add "watch" to package.xml. This is achieved by using configure_file. This is not necessary for catkin_make but
#if eclipse is used and the make target is used directly
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/package.xml"
               "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/package.xml" COPYONLY)

#gather dependencies from package.xml. The command runs in python with the ros environemnt
#variable set. This is used, because the python script is calling some ros tools to distinguish
#between catkin and non catkin packages.
set(_gather_cmd
    ${MRT_CMAKE_ENV} ${PYTHON_EXECUTABLE} ${MRT_CMAKE_MODULES_ROOT_PATH}/scripts/generate_cmake_dependency_file.py
    "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" "${MRT_CMAKE_MODULES_ROOT_PATH}/yaml/cmake.yaml"
    "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/auto_dep_vars.cmake")
execute_process(
    COMMAND ${_gather_cmd}
    RESULT_VARIABLE _GEN_DEPS_RES_
    ERROR_VARIABLE _GEN_DEPS_ERROR_)

if(NOT _GEN_DEPS_RES_ EQUAL 0)
    message(FATAL_ERROR "Gather depenencies failed: ${_GEN_DEPS_ERROR_}")
elseif(_GEN_DEPS_ERROR_)
    message(WARNING ${_GEN_DEPS_ERROR_})
endif()

#include the generated variable cmake file
include("${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/auto_dep_vars.cmake")
