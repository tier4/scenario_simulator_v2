include(${MRT_CMAKE_MODULES_CMAKE_PATH}/Modules/MrtTesting.cmake)

# care for clang-tidy flags
if(MRT_CLANG_TIDY STREQUAL "check")
    set(MRT_CLANG_TIDY_FLAGS "-extra-arg=-Wno-unknown-warning-option" "-header-filter=${PROJECT_SOURCE_DIR}/.*")
elseif(MRT_CLANG_TIDY STREQUAL "fix")
    set(MRT_CLANG_TIDY_FLAGS "-extra-arg=-Wno-unknown-warning-option" "-fix-errors"
                             "-header-filter=${PROJECT_SOURCE_DIR}/.*" "-format-style=file")
endif()
if(DEFINED MRT_CLANG_TIDY_FLAGS)
    if(${CMAKE_VERSION} VERSION_LESS "3.6.0")
        message(WARNING "Using clang-tidy requires at least CMAKE 3.6.0. Please upgrade CMake.")
    endif()
    find_package(ClangTidy)
    if(ClangTidy_FOUND)
        message(STATUS "Add clang tidy flags")
        set(CMAKE_CXX_CLANG_TIDY "${ClangTidy_EXE}" "${MRT_CLANG_TIDY_FLAGS}")
    else()
        message(WARNING "Failed to find clang-tidy. Is it installed?")
    endif()
endif()
unset(MRT_CLANG_TIDY_FLAGS)

# construct the mrt_sanitizer_lib_flags and  mrt_sanitizer_exe_flags target based on the configuation in the MRT_SANITIZER variable
if(NOT TARGET ${PROJECT_NAME}_sanitizer_lib_flags AND NOT TARGET ${PROJECT_NAME}_sanitizer_exe_flags)
    add_library(${PROJECT_NAME}_sanitizer_lib_flags INTERFACE)
    add_library(${PROJECT_NAME}_sanitizer_exe_flags INTERFACE)
    set(gcc_cxx "$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:GNU>,$<VERSION_GREATER:$<CXX_COMPILER_VERSION>,6.3>>")
    if(MRT_SANITIZER STREQUAL "checks" OR MRT_SANITIZER STREQUAL "check_race")
        target_compile_options(
            ${PROJECT_NAME}_sanitizer_lib_flags
            INTERFACE
                $<$<AND:${gcc_cxx},$<STREQUAL:${MRT_SANITIZER},checks>>:-fsanitize=undefined,bounds-strict,float-divide-by-zero,float-cast-overflow;-fsanitize-recover=alignment>
                $<$<AND:${gcc_cxx},$<STREQUAL:${MRT_SANITIZER},check_race>>:-fsanitize=thread,undefined,bounds-strict,float-divide-by-zero,float-cast-overflow>
                $<$<AND:${gcc_cxx},$<STREQUAL:${MRT_SANITIZER_RECOVER},no_recover>>:-fno-sanitize-recover=undefined,bounds-strict,float-divide-by-zero,float-cast-overflow>
        )
        target_compile_options(
            ${PROJECT_NAME}_sanitizer_exe_flags
            INTERFACE
                $<$<AND:${gcc_cxx},$<STREQUAL:${MRT_SANITIZER},checks>>:-fsanitize=address,leak,undefined,bounds-strict,float-divide-by-zero,float-cast-overflow;-fsanitize-recover=alignment>
                $<$<AND:${gcc_cxx},$<STREQUAL:${MRT_SANITIZER},check_race>>:-fsanitize=thread,undefined,float-divide-by-zero,float-cast-overflow>
                $<$<AND:${gcc_cxx},$<STREQUAL:${MRT_SANITIZER_RECOVER},no_recover>>:-fno-sanitize-recover=undefined,bounds-strict,float-divide-by-zero,float-cast-overflow>
        )
        target_link_options(
            ${PROJECT_NAME}_sanitizer_lib_flags
            INTERFACE
            $<$<AND:${gcc_cxx},$<STREQUAL:${MRT_SANITIZER},checks>>:-fsanitize=undefined,bounds-strict,float-divide-by-zero,float-cast-overflow;-fsanitize-recover=alignment>
            $<$<AND:${gcc_cxx},$<STREQUAL:${MRT_SANITIZER},check_race>>:-fsanitize=thread,undefined,bounds-strict,float-divide-by-zero,float-cast-overflow>
        )
        target_link_options(
            ${PROJECT_NAME}_sanitizer_exe_flags
            INTERFACE
            $<$<AND:${gcc_cxx},$<STREQUAL:${MRT_SANITIZER},checks>>:-fsanitize=address,leak,undefined,bounds-strict,float-divide-by-zero,float-cast-overflow;-fsanitize-recover=alignment>
            $<$<AND:${gcc_cxx},$<STREQUAL:${MRT_SANITIZER},check_race>>:-fsanitize=thread,undefined,float-divide-by-zero,float-cast-overflow>
        )
        # your brain just exploded due to all this? mine did too...
    endif()
endif() # create sanitizer target

# set install dirs (CMAKE_INSTALL_<dir>). We use them over catkins dirs, but use the same layout as catkin does by default.
include(GNUInstallDirs)
if(NOT CATKIN_DEVEL_PREFIX)
    set(CATKIN_DEVEL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/devel)
endif()
if(NOT EXISTS ${CATKIN_DEVEL_PREFIX}/include)
    # ensure that the devel include folder exists
    file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
endif()

# set the ros version
if(DEFINED ENV{ROS_VERSION})
    set(ROS_VERSION $ENV{ROS_VERSION})
endif()

# make sure catkin/ament are found so that PYTHON_EXECUTABLE (and CATKIN_ENV) is set.
if(NOT PYTHON_VERSION AND DEFINED $ENV{ROS_PYTHON_VERSION})
    set(PYTHON_VERSION
        $ENV{ROS_PYTHON_VERSION}
        CACHE STRING "Python version to use ('major.minor' or 'major')")
endif()

if(ROS_VERSION EQUAL 1)
    find_package(catkin REQUIRED)
    set(MRT_CMAKE_ENV sh ${CATKIN_ENV})
elseif(ROS_VERSION EQUAL 2)
    find_package(ament_cmake_core REQUIRED)
    if(NOT DEFINED BUILD_TESTING OR BUILD_TESTING)
        # our cmake template still relies on CATKIN_ENABLE_TESTING
        set(CATKIN_ENABLE_TESTING TRUE)
    endif()
else()
    set(CATKIN_ENABLE_TESTING TRUE)
endif()
# would be nicer to put this in a function, but cmake requires this at global scope
if(CATKIN_ENABLE_TESTING)
    enable_testing()
endif()

# set some global variables needed/modified by the functions below
# list of folders containing a folder "<PROJECT_NAME>" with headers required by this project. These will be installed by mrt_install.
set(${PROJECT_NAME}_LOCAL_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include;${CATKIN_DEVEL_PREFIX}/include")
set(${PROJECT_NAME}_PYTHON_API_TARGET "") # contains the list of python api targets
set(${PROJECT_NAME}_GENERATED_LIBRARIES "") # the list of library targets built and installed by the tools
set(${PROJECT_NAME}_MRT_TARGETS "") # list of all public installable targets created by the functions here

# define rosparam/rosinterface_handler macro for compability. Macros will be overriden by the actual macros defined by the packages, if existing.
macro(generate_ros_parameter_files)
    # handle pure dynamic reconfigure files
    foreach(_cfg ${ARGN})
        get_filename_component(_cfgext ${_cfg} EXT)
        if(_cfgext STREQUAL ".cfg")
            list(APPEND _${PROJECT_NAME}_pure_cfg_files "${_cfg}")
        else()
            list(APPEND _${PROJECT_NAME}_rosparam_other_param_files "${_cfg}")
        endif()
    endforeach()
    # generate dynamic reconfigure files
    if(_${PROJECT_NAME}_pure_cfg_files
       AND NOT TARGET ${PROJECT_NAME}_gencfg
       AND NOT (rosinterface_handler_FOUND_CATKIN_PROJECT OR rosinterface_handler_FOUND))
        if(dynamic_reconfigure_FOUND_CATKIN_PROJECT)
            generate_dynamic_reconfigure_options(${_${PROJECT_NAME}_pure_cfg_files})
        else()
            message(
                WARNING
                    "Dependency to dynamic_reconfigure is missing, or find_package(dynamic_reconfigure) was not called yet. Not building dynamic config files"
            )
        endif()
    endif()
    # if there are other config files, someone will have forgotten to include the rosparam/rosinterface handler
    if(_${PROJECT_NAME}_rosparam_other_param_files AND NOT (rosinterface_handler_FOUND_CATKIN_PROJECT
                                                            OR rosinterface_handler_FOUND))
        message(
            FATAL_ERROR
                "Dependency rosinterface_handler or rosparam_handler could not be found. Did you add it to your package.xml?"
        )
    endif()
endmacro()
macro(generate_ros_interface_files)
    # handle pure dynamic reconfigure files
    foreach(_cfg ${ARGN})
        get_filename_component(_cfgext ${_cfg} EXT)
        if(NOT _cfgext STREQUAL ".cfg")
            list(APPEND _${PROJECT_NAME}_rosif_other_param_files "${_cfg}")
        endif()
    endforeach()
    if(_${PROJECT_NAME}_rosif_other_param_files AND NOT (rosparam_handler_FOUND_CATKIN_PROJECT
                                                         OR rosinterface_handler_FOUND))
        message(
            FATAL_ERROR
                "Dependency rosinterface_handler or rosparam_handler could not be found. Did you add it to your package.xml?"
        )
    endif()
endmacro()

macro(_setup_coverage_info)
    setup_target_for_coverage(${PROJECT_NAME}-coverage coverage ${PROJECT_NAME}-pre-coverage)
    # make sure the target is built after running tests
    add_dependencies(run_tests ${PROJECT_NAME}-coverage)
    add_dependencies(${PROJECT_NAME}-coverage _run_tests_${PROJECT_NAME})
    if(TARGET ${PROJECT_NAME}-pre-coverage)
        add_dependencies(clean_test_results_${PROJECT_NAME} ${PROJECT_NAME}-pre-coverage)
        add_dependencies(${PROJECT_NAME}-pre-coverage tests)
    endif()
    if(MRT_ENABLE_COVERAGE GREATER 1)
        add_custom_command(
            TARGET ${PROJECT_NAME}-coverage
            POST_BUILD
            COMMAND firefox ${CMAKE_CURRENT_BINARY_DIR}/coverage/index.html > /dev/null 2>&1 &
            COMMENT "Showing coverage results")
    endif()
endmacro()

macro(_mrt_register_ament_python_hook)
    # ament is different in catkin that you have to register environment hooks for everything
    # code was taken from ament_cmake_python
    if(NOT _MRT_AMENT_PYTHON_HOOK_REGISTERED)
        find_package(ament_cmake_core QUIET REQUIRED)
        # use native separators in environment hook to match what pure Python packages do
        file(TO_NATIVE_PATH "${destination}" destination)

        # register information for .dsv generation
        _mrt_get_python_destination(PYTHON_INSTALL_DIR)
        set(AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_pythonpath "prepend-non-duplicate;PYTHONPATH;${destination}")
        ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_PYTHONPATH}")
        set(_MRT_AMENT_PYTHON_HOOK_REGISTERED
            TRUE
            PARENT_SCOPE)
    endif()
endmacro()

#
# Parses the package.xml and sets its information in form of cmake variables as ${PACKAGE_NAME}_<info>, where info is
# one of version, VERSION, MAINTAINER, PACKAGE_FORMAT, <BUILD/BUILD_EXPORT/TEST/EXEC/...>_DEPENDS, URL_<WEBSITE/BUGTRACKER/REPOSITORY>
#
# This function must be called before any other of the mrt_* functions can be called.
#
# @public
#
macro(mrt_parse_package_xml)
    # TODO: replace this by our own stuff.
    if(NOT COMMAND catkin_package_xml AND ROS_VERSION EQUAL 1)
        find_package(catkin REQUIRED)
    elseif(NOT COMMAND ament_package_xml AND ROS_VERSION EQUAL 2)
        find_package(ament_cmake_core REQUIRED)
    endif()
    if(ROS_VERSION EQUAL 1)
        catkin_package_xml()
    elseif(ROS_VERSION EQUAL 2)
        ament_package_xml()
    endif()
endmacro()

#
# Adds all the librares and targets that this target should be linked against. This includes dependencies found by AutoDeps, compiler flags and sanitizers.
# Also sets all local include directories usually required by the target
#
# This function is automatically called for all targets created with mrt_add_(executable/library/test/...).
#
# :param CUDA: indicates this is cuda and not C++ code
#
# Example:
# ::
#
#  mrt_add_links(my_target [CUDA]
#      )
#
# @public
#
function(mrt_add_links target)
    cmake_parse_arguments(ARG "CUDA;NO_SANITIZER;TEST" "" "" ${ARGN})
    get_target_property(target_type ${target} TYPE)

    # add dependencies
    if(ARG_CUDA)
        # this is a cuda target
        if(TARGET ${PROJECT_NAME}::auto_deps_cuda)
            target_link_libraries(${target} PRIVATE ${PROJECT_NAME}::auto_deps_cuda)
        endif()
    else()
        if(NOT target_type STREQUAL "INTERFACE_LIBRARY")
            if(TARGET ${PROJECT_NAME}::auto_deps)
                target_link_libraries(${target} PRIVATE ${PROJECT_NAME}::auto_deps)
            endif()
            if(TARGET ${PROJECT_NAME}::auto_deps_export)
                # we use the namespaced alias here, because this is how other subdirectories see it
                target_link_libraries(${target} PUBLIC ${PROJECT_NAME}::auto_deps_export)
            endif()
        else()
            if(NOT (MRT_PKG_VERSION AND MRT_PKG_VERSION VERSION_LESS "3.0.2") AND TARGET
                                                                                  ${PROJECT_NAME}::auto_deps_export)
                # On old cmakelists still using catkin_package, we cannot use the export target on interface libraries,
                # because catkin tries to interpret generator expressions as libraries. We instead export them though mrt_EXPORTED_LIBRARIES.
                target_link_libraries(${target} INTERFACE ${PROJECT_NAME}::auto_deps_export)
            endif()
        endif()

    endif()
    # add test depends
    if(ARG_TEST AND TARGET ${PROJECT_NAME}::auto_deps_test)
        target_link_libraries(${target} PRIVATE ${PROJECT_NAME}::auto_deps_test)
    endif()

    # add include dirs
    if(NOT target_type STREQUAL "INTERFACE_LIBRARY")
        # For convenience, targets within the project can omit the project name for including local headers
        if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME} AND NOT MRT_NO_LOCAL_INCLUDE)
            target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})
        endif()
        # set the include dir for installation and dependent targets.
        foreach(include ${${PROJECT_NAME}_LOCAL_INCLUDE_DIRS})
            if(EXISTS ${include})
                target_include_directories(${target} PUBLIC $<BUILD_INTERFACE:${include}>)
            endif()
        endforeach()
    else()
        # set the include dir for installation and dependent targets.
        foreach(include ${${PROJECT_NAME}_LOCAL_INCLUDE_DIRS})
            if(EXISTS ${include})
                target_include_directories(${target} INTERFACE $<BUILD_INTERFACE:${include}>)
            endif()
        endforeach()
    endif()
    if(NOT target_type STREQUAL "EXECUTABLE")
        target_include_directories(${target} INTERFACE $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
    endif()

    # add sanitizer flags if set
    if(NOT ARG_NO_SANITIZER)
        if(target_type STREQUAL "EXECUTABLE")
            if(TARGET ${PROJECT_NAME}_sanitizer_exe_flags)
                target_link_libraries(${target} PRIVATE ${PROJECT_NAME}_sanitizer_exe_flags)
            endif()
        elseif(NOT target_type STREQUAL "INTERFACE_LIBRARY")
            if(TARGET ${PROJECT_NAME}_sanitizer_lib_flags)
                target_link_libraries(${target} PRIVATE ${PROJECT_NAME}_sanitizer_lib_flags)
            endif()
        endif()
    endif()

    # add compile flags
    if(ARG_CUDA)
        # below cmake 3.17, setting cpp standard also affects the cuda standard. So we cannot use the normal compiler_flags.
        if(TARGET cuda_compiler_flags)
            target_link_libraries(${target} PRIVATE cuda_compiler_flags)
        endif()
    elseif(NOT target_type STREQUAL "INTERFACE_LIBRARY")
        if(TARGET ${PROJECT_NAME}_compiler_flags)
            target_link_libraries(${target} PUBLIC ${PROJECT_NAME}_compiler_flags)
        endif()
        if(TARGET ${PROJECT_NAME}_private_compiler_flags)
            target_link_libraries(${target} PRIVATE ${PROJECT_NAME}_private_compiler_flags)
        endif()
    else()
        if(NOT (MRT_PKG_VERSION AND MRT_PKG_VERSION VERSION_LESS "3.0.2") AND TARGET ${PROJECT_NAME}_compiler_flags)
            # We can not add flags with generator expression to interface libraries because catkin interprets them as libraries.
            # Its not a problem though, because catkin wouldn't export these flags anyways.
            target_link_libraries(${target} INTERFACE ${PROJECT_NAME}_compiler_flags)
        endif()
    endif() # interface library or not

    # set dependencies to exported targets like messages and such
    set(_deps ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    if(_deps)
        add_dependencies(${target} ${_deps})
    endif()
endfunction()

function(_mrt_get_python_destination output_var)
    if(DEFINED MRT_PYTHON_INSTALL_DESTINATION)
        set(${output_var}
            ${MRT_PYTHON_INSTALL_DESTINATION}
            PARENT_SCOPE)
        return()
    endif()
    if(PYTHON_VERSION)
        set(_python_version ${PYTHON_VERSION})
    elseif(DEFINED ENV{ROS_PYTHON_VERSION})
        set(_python_version $ENV{ROS_PYTHON_VERSION})
    endif()
    # finding python sets PYTHON_VERSION_<MAJOR/MINOR>
    if(CMAKE_VERSION VERSION_LESS 3.12)
        set(Python_ADDITIONAL_VERSIONS ${_python_version})
        find_package(PythonInterp REQUIRED)
    elseif(_python_version VERSION_LESS 3)
        find_package(Python2 REQUIRED)
    else()
        find_package(Python3 REQUIRED)
    endif()
    # seems ros2 found an even more complex way to determine the install location...
    if(NOT ROS_VERSION EQUAL 1)
        set(_python_code
            "from distutils.sysconfig import get_python_lib"
            "import os"
            "print(os.path.relpath(get_python_lib(prefix='${CMAKE_INSTALL_PREFIX}'), start='${CMAKE_INSTALL_PREFIX}').replace(os.sep, '/'))"
        )
        execute_process(
            COMMAND "${PYTHON_EXECUTABLE}" "-c" "${_python_code}"
            OUTPUT_VARIABLE _output
            RESULT_VARIABLE _result
            OUTPUT_STRIP_TRAILING_WHITESPACE)
        set(${output_var} ${_output})
    elseif(WIN32)
        set(${output_var} lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages)
    elseif(EXISTS "/etc/debian_version")
        if("${PYTHON_VERSION_MAJOR}" STREQUAL "3")
            set(${output_var} lib/python${PYTHON_VERSION_MAJOR}/dist-packages)
        else()
            set(${output_var} lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/dist-packages)
        endif()
    else()
        set(${output_var} lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages)
    endif()
    set(${output_var}
        ${${output_var}}
        PARENT_SCOPE)
    set(MRT_PYTHON_INSTALL_DESTINATION
        ${${output_var}}
        CACHE INTERNAL "Installation dir for python files")
endfunction()

# Glob for folders in the search directory.
function(mrt_glob_folders DIRECTORY_LIST SEARCH_DIRECTORY)
    if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
        file(GLOB DIRECTORIES "${SEARCH_DIRECTORY}/[^.]*")
    else()
        file(GLOB DIRECTORIES CONFIGURE_DEPENDS "${SEARCH_DIRECTORY}/[^.]*")
    endif()

    set(_DIRECTORY_LIST_ "")
    foreach(SRC_DIR ${DIRECTORIES})
        if(IS_DIRECTORY "${SRC_DIR}")
            get_filename_component(DIRECTORY_NAME "${SRC_DIR}" NAME)
            list(APPEND _DIRECTORY_LIST_ ${DIRECTORY_NAME})
        endif()
    endforeach()
    set(${DIRECTORY_LIST}
        ${_DIRECTORY_LIST_}
        PARENT_SCOPE)
endfunction()

# Deprecated function. Use 'mrt_glob_folders' instead.
macro(glob_folders)
    mrt_glob_folders(${ARGV})
endmacro()

# Globs for message files and calls add_message_files
macro(mrt_add_message_files folder_name)
    mrt_glob_files(_ROS_MESSAGE_FILES REL_FOLDER ${folder_name} ${folder_name}/*.msg)
    if(_ROS_MESSAGE_FILES)
        add_message_files(FILES ${_ROS_MESSAGE_FILES} DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${folder_name}")
        set(ROS_GENERATE_MESSAGES True)
    endif()
endmacro()

# Globs for service files and calls add_service_files
macro(mrt_add_service_files folder_name)
    mrt_glob_files(_ROS_SERVICE_FILES REL_FOLDER ${folder_name} ${folder_name}/*.srv)
    if(_ROS_SERVICE_FILES)
        add_service_files(FILES ${_ROS_SERVICE_FILES} DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${folder_name}")
        set(ROS_GENERATE_MESSAGES True)
    endif()
endmacro()

# Globs for action files and calls add_action_files
macro(mrt_add_action_files folder_name)
    mrt_glob_files(_ROS_ACTION_FILES REL_FOLDER ${folder_name} ${folder_name}/*.action)
    if(_ROS_ACTION_FILES)
        add_action_files(FILES ${_ROS_ACTION_FILES} DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/${folder_name}")
        set(ROS_GENERATE_ACTION True)
    endif()
endmacro()

# Deprecated function. Use one of 'mrt_add_message_files', 'mrt_add_service_files' or 'mrt_add_action_files'.
macro(glob_ros_files excecutable_name extension_name)
    mrt_glob_files(ROS_${excecutable_name}_FILES REL_FOLDER ${extension_name} "${extension_name}/*.${extension_name}")

    if(ROS_${excecutable_name}_FILES)
        #work around to execute a command wich name is given in a variable
        #write a file with the command, include it and delete the file again
        file(
            WRITE "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/_GLOB_ROS_TEMP_FILE.cmake"
            "${excecutable_name}(
            DIRECTORY \"${PROJECT_SOURCE_DIR}/${extension_name}\"
            FILES
            ${ROS_${excecutable_name}_FILES}
            )")
        include("${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/_GLOB_ROS_TEMP_FILE.cmake")
        file(REMOVE "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/_GLOB_ROS_TEMP_FILE.cmake")

        set(ROS_GENERATE_MESSAGES True)
    endif()
endmacro()

# Globs files in the currect project dir.
function(mrt_glob_files varname)
    cmake_parse_arguments(PARAMS "" "REL_FOLDER" "" ${ARGN})

    if(PARAMS_REL_FOLDER)
        set(RELATIVE_PATH "${PROJECT_SOURCE_DIR}/${PARAMS_REL_FOLDER}")
    else()
        set(RELATIVE_PATH "${PROJECT_SOURCE_DIR}")
    endif()

    if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
        file(
            GLOB files
            RELATIVE "${RELATIVE_PATH}"
            ${PARAMS_UNPARSED_ARGUMENTS})
    else()
        file(
            GLOB files
            RELATIVE "${RELATIVE_PATH}"
            CONFIGURE_DEPENDS ${PARAMS_UNPARSED_ARGUMENTS})
    endif()
    set(${varname}
        ${files}
        PARENT_SCOPE)
endfunction()

# Globs files recursivly in the currect project dir.
function(mrt_glob_files_recurse varname)
    cmake_parse_arguments(PARAMS "" "REL_FOLDER" "" ${ARGN})

    if(PARAMS_REL_FOLDER)
        set(RELATIVE_PATH "${PROJECT_SOURCE_DIR}/${PARAMS_REL_FOLDER}")
    else()
        set(RELATIVE_PATH "${PROJECT_SOURCE_DIR}")
    endif()

    if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
        file(
            GLOB_RECURSE files
            RELATIVE "${RELATIVE_PATH}"
            ${PARAMS_UNPARSED_ARGUMENTS})
    else()
        file(
            GLOB_RECURSE files
            RELATIVE "${RELATIVE_PATH}"
            CONFIGURE_DEPENDS ${PARAMS_UNPARSED_ARGUMENTS})
    endif()
    set(${varname}
        ${files}
        PARENT_SCOPE)
endfunction()

#
# Once upon a time this used to make non-code files known to IDEs that parse Cmake output. But as this
# messes up with the target determination mechanism used by most ides and garbages up the target view.
#
# Therefore this function is no longer used and only here for backwards compability.
#
# @public
#
function(mrt_add_to_ide files)

endfunction()

#
# Automatically sets up and installs python modules located under ``src/${PROJECT_NAME}``.
# Modules can afterwards simply be included using "import <project_name>" in python.
#
# The python folder (under src/${PROJECT_NAME}) is required to have an __init__.py file.
#
# On Ros1, the command will automatically generate a setup.py in your project folder.
# This file should not be commited, as it will be regenerated at every new CMAKE run.
# Due to restrictions imposed by catkin (searches hardcoded for this setup.py), the file cannot
# be placed elsewhere.
#
# Example:
# ::
#
#   mrt_python_module_setup()
#
# @public
#
function(mrt_python_module_setup)
    if(ARGN)
        message(FATAL_ERROR "mrt_python_module_setup() called with unused arguments: ${ARGN}")
    endif()
    set(module_folder ${PROJECT_SOURCE_DIR}/src/${PROJECT_NAME})
    if(NOT EXISTS "${module_folder}/__init__.py")
        return()
    endif()
    set(${PROJECT_NAME}_PYTHON_MODULE
        ${PROJECT_NAME}
        PARENT_SCOPE)
    if(ROS_VERSION EQUAL 1)
        if(NOT catkin_FOUND)
            find_package(catkin REQUIRED)
        endif()
        set(PKG_PYTHON_MODULE ${PROJECT_NAME})
        set(PACKAGE_DIR "src")
        configure_file(${MCM_TEMPLATE_DIR}/setup.py.in "${PROJECT_SOURCE_DIR}/setup.py" @ONLY)
        catkin_python_setup()
    else()
        _mrt_get_python_destination(python_destination)
        set(destination "${python_destination}/${PROJECT_NAME}")
        install(
            DIRECTORY "${module_folder}/"
            DESTINATION "${python_destination}/${PROJECT_NAME}"
            PATTERN "*.pyc" EXCLUDE
            PATTERN "__pycache__" EXCLUDE)
        # compile Python files
        install(
            CODE "execute_process(
            COMMAND
             \"${PYTHON_EXECUTABLE}\" \"-m\" \"compileall\"
             \"${CMAKE_INSTALL_PREFIX}/${python_destination}/${PROJECT_NAME}\"
             )")
        if(ROS_VERSION EQUAL 2)
            _mrt_register_ament_python_hook()
        endif()
    endif()
endfunction()

#
# Generates a python module from boost-python cpp files.
#
# Each <file>.cpp will become a seperate <file>.py submodule within <modulename>. After building and sourcing you can use the modules simply with "import <modulename>.<file>".
#
# The files are automatically linked with boost-python libraries and a python module is generated
# and installed from the resulting library. If this project declares any libraries with ``mrt_add_library()``, they will automatically be linked with this library.
#
# This function will define the compiler variable ``PYTHON_API_MODULE_NAME`` with the name of the generated library. This can be used in the ``BOOST_PYTHON_MODULE`` C++ Macro.
#
# .. note:: This function can only be called once per package.
#
# :param modulename: Name of the module needs to be passed as first parameter.
# :type modulename: string
# :param FILES: list of C++ files defining the BOOST-Python API.
# :type FILES: list of strings
#
# Example:
# ::
#
#   mrt_add_python_api( example_package
#       FILES python_api/python.cpp
#       )
#
# @public
#
function(mrt_add_python_api modulename)
    cmake_parse_arguments(MRT_ADD_PYTHON_API "" "" "FILES" ${ARGN})
    if(NOT MRT_ADD_PYTHON_API_FILES)
        return()
    endif()

    #set and check target name
    set(PYTHON_API_MODULE_NAME ${modulename})

    if("${PYTHON_API_MODULE_NAME}" STREQUAL "${PROJECT_NAME}")
        # mark that catkin_python_setup() was called and the setup.py file contains a package with the same name as the current project
        # in order to disable installation of generated __init__.py files in generate_messages() and generate_dynamic_reconfigure_options()
        set(${PROJECT_NAME}_CATKIN_PYTHON_SETUP_HAS_PACKAGE_INIT
            TRUE
            PARENT_SCOPE)
    endif()
    if(${PROJECT_NAME}_PYTHON_API_TARGET)
        message(
            FATAL_ERROR
                "mrt_add_python_api() was already called for this project. You can add only one python_api per project!"
        )
    endif()

    if(NOT pybind11_FOUND AND NOT BoostPython_FOUND)
        message(
            FATAL_ERROR
                "Missing dependency to pybind11 or boost python. Add either '<depend>pybind11-dev</depend>' or '<depend>libboost-python-dev</depend>' to 'package.xml'"
        )
    endif()

    if(pybind11_FOUND AND BoostPython_FOUND)
        message(FATAL_ERROR "Found pybind11 and boost python. Only one is allowed.")
    endif()

    # put in devel folder
    set(DEVEL_PREFIX ${CATKIN_DEVEL_PREFIX})
    _mrt_get_python_destination(python_destination)
    set(PYTHON_MODULE_DIR "${DEVEL_PREFIX}/${python_destination}/${PYTHON_API_MODULE_NAME}")

    # add library for each file
    foreach(API_FILE ${MRT_ADD_PYTHON_API_FILES})
        get_filename_component(SUBMODULE_NAME ${API_FILE} NAME_WE)
        set(TARGET_NAME "${PROJECT_NAME}-${PYTHON_API_MODULE_NAME}-${SUBMODULE_NAME}-pyapi")
        set(LIBRARY_NAME ${SUBMODULE_NAME})
        message(STATUS "Adding python api library \"${LIBRARY_NAME}\" to python module \"${PYTHON_API_MODULE_NAME}\"")

        if(pybind11_FOUND)
            pybind11_add_module(${TARGET_NAME} MODULE ${API_FILE})
            target_link_libraries(${TARGET_NAME} PRIVATE pybind11::module)
        elseif(BoostPython_FOUND)
            add_library(${TARGET_NAME} SHARED ${API_FILE})
            target_link_libraries(${TARGET_NAME} PRIVATE ${BoostPython_LIBRARIES} ${PYTHON_LIBRARY})
        endif()

        set_target_properties(${TARGET_NAME} PROPERTIES OUTPUT_NAME ${LIBRARY_NAME} PREFIX "")
        # Yes for all build types. Someone setting CMAKE_LIBRARY_OUTPUT_DIRS_DEBUG would screw everything...
        set_target_properties(
            ${TARGET_NAME}
            PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${PYTHON_MODULE_DIR}"
                       LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PYTHON_MODULE_DIR}"
                       LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PYTHON_MODULE_DIR}"
                       LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL "${PYTHON_MODULE_DIR}"
                       LIBRARY_OUTPUT_DIRECTORY_DEBUG "${PYTHON_MODULE_DIR}")

        target_compile_definitions(${TARGET_NAME} PRIVATE PYTHON_API_MODULE_NAME=${LIBRARY_NAME})
        mrt_add_links(${TARGET_NAME} NO_SANITIZER) # you really don't want to debug python...
        list(APPEND GENERATED_TARGETS ${TARGET_NAME})
    endforeach()
    if(NOT "${${PROJECT_NAME}_PYTHON_MODULE}" STREQUAL "${PYTHON_API_MODULE_NAME}")
        configure_file(${MCM_TEMPLATE_DIR}/__init__.py.in ${PYTHON_MODULE_DIR}/__init__.py)
        if(GENERATE_ENVIRONMENT_CACHE_COMMAND)
            # regenerate the environment cache. This makes sure PYTHONPATH is set correctly for nosetests
            safe_execute_process(COMMAND ${GENERATE_ENVIRONMENT_CACHE_COMMAND})
        endif()
    endif()

    # append to list of all targets in this project
    set(${PROJECT_NAME}_PYTHON_API_TARGET
        ${GENERATED_TARGETS}
        PARENT_SCOPE)
    if(ROS_VERSION EQUAL 1)
        # configure setup.py for install
        set(PKG_PYTHON_MODULE ${PYTHON_API_MODULE_NAME})
        set(PACKAGE_DIR ${DEVEL_PREFIX}/${python_destination})
        set(PACKAGE_DATA "*.so*")
        configure_file(${MCM_TEMPLATE_DIR}/setup.py.in "${CMAKE_CURRENT_BINARY_DIR}/setup.py" @ONLY)
        configure_file(${MCM_TEMPLATE_DIR}/python_api_install.py.in "${CMAKE_CURRENT_BINARY_DIR}/python_api_install.py"
                       @ONLY)
        install(CODE "execute_process(COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/python_api_install.py)")
    else()
        if(ROS_VERSION EQUAL 2)
            _mrt_register_ament_python_hook()
        endif()
        set(install_destination ${python_destination}/${PYTHON_API_MODULE_NAME})
        install(FILES ${PYTHON_MODULE_DIR}/__init__.py DESTINATION ${install_destination})
        install(TARGETS ${GENERATED_TARGETS} DESTINATION ${install_destination})
    endif()
endfunction()

#
# Adds a library.
#
# This command ensures the library is compiled with all necessary dependencies. If no files are passed, the command will return silently.
#
# .. note:: Make sure to call this after all messages and parameter generation CMAKE-Commands so that all dependencies are visible.
#
# The files are automatically added to the list of installable targets so that ``mrt_install`` can mark them for installation.
#
# :param libname: Name of the library to generate as first argument (without lib or .so)
# :type libname: string
# :param INCLUDES: Public include files needed to use the library, absolute or relative to ${PROJECT_SOURCE_DIR}
# :type INCLUDES: list of strings
# :param SOURCES: Source files and private headers to be added. If no source files are passed, a header-only library is assumed
# :type SOURCES: list of strings
# :param DEPENDS: List of extra (non-catkin, non-mrt) dependencies. This should only be required for including external projects.
# :type DEPENDS: list of strings
# :param LIBRARIES: Extra (non-catkin, non-mrt) libraries to link to. This should only be required for external projects
# :type LIBRARIES: list of strings
#
# Example:
# ::
#
#   mrt_add_library( example_package
#       INCLUDES include/example_package/myclass.h include/example_package/myclass2.h
#       SOURCES src/myclass.cpp src/myclass.cpp
#       )
#
# @public
#
function(mrt_add_library libname)
    set(LIBRARY_NAME ${libname})
    if(NOT LIBRARY_NAME)
        message(FATAL_ERROR "No library name specified for call to mrt_add_library!")
    endif()
    cmake_parse_arguments(MRT_ADD_LIBRARY "" "" "INCLUDES;SOURCES;DEPENDS;LIBRARIES" ${ARGN})
    set(LIBRARY_TARGET_NAME ${LIBRARY_NAME})

    # generate the library target
    if(NOT MRT_ADD_LIBRARY_SOURCES)
        # build a "header-only" library. This is done even when there are no files at all, so that pure message packages
        # have something other packages can depend on.
        message(STATUS "Adding header-only library with files ${MRT_ADD_LIBRARY_INCLUDES}")
        add_library(${LIBRARY_TARGET_NAME} INTERFACE)
    else()
        # normal library (potentially with cuda sources)
        foreach(SOURCE_FILE ${MRT_ADD_LIBRARY_SOURCES})
            get_filename_component(FILE_EXT ${SOURCE_FILE} EXT)
            if("${FILE_EXT}" STREQUAL ".cu")
                list(APPEND _MRT_CUDA_SOURCE_FILES "${SOURCE_FILE}")
            else()
                list(APPEND _MRT_CPP_SOURCE_FILES "${SOURCE_FILE}")
            endif()
        endforeach()

        # This is the easiest for a CUDA only library: Create an empty file.
        if(NOT _MRT_CPP_SOURCE_FILES)
            message(STATUS "CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")
            file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/empty.cpp" "")
            list(APPEND _MRT_CPP_SOURCE_FILES "${CMAKE_CURRENT_BINARY_DIR}/empty.cpp")
        endif()

        # generate the target
        message(
            STATUS
                "Adding library \"${LIBRARY_NAME}\" with source ${_MRT_CPP_SOURCE_FILES}, includes ${MRT_ADD_LIBRARY_INCLUDES}"
        )
        add_library(${LIBRARY_TARGET_NAME} ${MRT_ADD_LIBRARY_INCLUDES} ${_MRT_CPP_SOURCE_FILES})

        # we always build with fPIC to avoid problems when mixing static and shared libs
        set_property(TARGET ${LIBRARY_TARGET_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

        # extract the major version
        if(${PROJECT_NAME}_VERSION)
            string(REPLACE "." ";" versions ${${PROJECT_NAME}_VERSION})
            list(GET versions 0 version_major)
            set_target_properties(
                ${LIBRARY_TARGET_NAME}
                PROPERTIES OUTPUT_NAME ${LIBRARY_NAME}
                           SOVERSION ${version_major}
                           VERSION ${${PROJECT_NAME}_VERSION})
        endif()
        target_compile_options(${LIBRARY_TARGET_NAME} PRIVATE ${MRT_SANITIZER_CXX_FLAGS})
        if(MRT_ADD_LIBRARY_LIBRARIES)
            target_link_libraries(${LIBRARY_TARGET_NAME} PRIVATE ${MRT_ADD_LIBRARY_LIBRARIES})
        endif()
    endif()
    # link to python_api if existing (needs to be declared before this library)
    foreach(_py_api_target ${${PROJECT_NAME}_PYTHON_API_TARGET})
        target_link_libraries(${_py_api_target} PRIVATE ${LIBRARY_TARGET_NAME})
    endforeach()
    set(${PROJECT_NAME}_EXPORTED_TARGETS
        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${${PROJECT_NAME}_PYTHON_API_TARGET}
        PARENT_SCOPE)

    mrt_add_links(${LIBRARY_TARGET_NAME})

    # Add cuda target
    if(_MRT_CUDA_SOURCE_FILES)
        if(NOT DEFINED CUDA_FOUND)
            message(
                FATAL_ERROR
                    "Found CUDA source file but no dependency to CUDA. Please add <depend>cuda</depend> to your package.xml."
            )
        endif()

        # generate cuda target
        set(CUDA_TARGET_NAME _${LIBRARY_TARGET_NAME}_cuda)
        # NVCC does not like '-' in file names.
        string(REPLACE "-" "_" CUDA_TARGET_NAME ${CUDA_TARGET_NAME})

        message(STATUS "Adding library \"${CUDA_TARGET_NAME}\" with source ${_MRT_CUDA_SOURCE_FILES}")

        if(${CMAKE_VERSION} VERSION_LESS "3.9.0")
            cuda_add_library(${CUDA_TARGET_NAME} SHARED ${_MRT_CUDA_SOURCE_FILES})
        else()
            add_library(${CUDA_TARGET_NAME} SHARED ${_MRT_CUDA_SOURCE_FILES})
            # We cannot link to all libraries as nvcc does not unterstand all the flags
            # etc. which could be passed to target_link_libraries as a target. So the
            # dependencies were added to the mrt_CUDA_LIBRARIES variable.
            target_compile_options(${CUDA_TARGET_NAME} PRIVATE --compiler-options -fPIC)
            set_property(TARGET ${CUDA_TARGET_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
            set_property(TARGET ${CUDA_TARGET_NAME} PROPERTY CUDA_SEPARABLE_COMPILATION ON)
            mrt_add_links(${CUDA_TARGET_NAME} CUDA)
        endif()

        # link cuda library to normal library
        target_link_libraries(${LIBRARY_TARGET_NAME} PRIVATE ${CUDA_TARGET_NAME})
    endif()

    # append to list of all targets in this project
    set(${PROJECT_NAME}_GENERATED_LIBRARIES
        ${${PROJECT_NAME}_GENERATED_LIBRARIES} ${LIBRARY_TARGET_NAME} ${CUDA_TARGET_NAME}
        PARENT_SCOPE)
    set(${PROJECT_NAME}_MRT_TARGETS
        ${${PROJECT_NAME}_MRT_TARGETS} ${LIBRARY_TARGET_NAME} ${CUDA_TARGET_NAME}
        PARENT_SCOPE)
endfunction()

#
# Adds an executable.
#
# This command ensures the executable is compiled with all necessary dependencies.
#
# .. note:: Make sure to call this after all messages and parameter generation CMAKE-Commands so that all dependencies are visible.
#
# The files are automatically added to the list of installable targets so that ``mrt_install`` can mark them for installation.
#
# :param execname: name of the executable
# :type execname: string
# :param FOLDER: Folder containing the .cpp/.cc-files and .h/.hh/.hpp files for the executable, relative to ``${PROJECT_SOURCE_DIR}``.
# :type FOLDER: string
# :param FILES: List of extra source files to add. This or the FOLDER parameter is mandatory.
# :type FILES: list of strings
# :param DEPENDS: List of extra (non-catkin, non-mrt) dependencies. This should only be required for including external projects.
# :type DEPENDS: list of strings
# :param LIBRARIES: Extra (non-catkin, non-mrt) libraries to link to. This should only be required for external projects
# :type LIBRARIES: list of strings
#
# Example:
# ::
#
#   mrt_add_executable( example_package
#       FOLDER src/example_package
#       )
#
# @public
#
function(mrt_add_executable execname)
    set(EXEC_NAME ${execname})
    if(NOT EXEC_NAME)
        message(FATAL_ERROR "No executable name specified for call to mrt_add_executable()!")
    endif()
    cmake_parse_arguments(MRT_ADD_EXECUTABLE "" "FOLDER" "FILES;DEPENDS;LIBRARIES" ${ARGN})
    if(NOT MRT_ADD_EXECUTABLE_FOLDER AND NOT MRT_ADD_EXECUTABLE_FILES)
        message(FATAL_ERROR "No FOLDER or FILES argument passed to mrt_add_executable()!")
    endif()
    set(EXEC_TARGET_NAME ${PROJECT_NAME}-${EXEC_NAME}-exec)

    # get the files
    if(MRT_ADD_EXECUTABLE_FOLDER)
        mrt_glob_files_recurse(
            EXEC_SOURCE_FILES_INC "${MRT_ADD_EXECUTABLE_FOLDER}/*.h" "${MRT_ADD_EXECUTABLE_FOLDER}/*.hpp"
            "${MRT_ADD_EXECUTABLE_FOLDER}/*.hh" "${MRT_ADD_EXECUTABLE_FOLDER}/*.cuh")
        mrt_glob_files_recurse(EXEC_SOURCE_FILES_SRC "${MRT_ADD_EXECUTABLE_FOLDER}/*.cpp"
                               "${MRT_ADD_EXECUTABLE_FOLDER}/*.cc" "${MRT_ADD_EXECUTABLE_FOLDER}/*.cu")
    endif()
    if(MRT_ADD_EXECUTABLE_FILES)
        list(APPEND EXEC_SOURCE_FILES_SRC ${MRT_ADD_EXECUTABLE_FILES})
        list(REMOVE_DUPLICATES EXEC_SOURCE_FILES_SRC)
    endif()
    if(NOT EXEC_SOURCE_FILES_SRC)
        return()
    endif()

    # separate cuda files
    set(_MRT_CPP_SOURCE_FILES)
    set(_MRT_CUDA_SOURCE_FILES)
    foreach(SOURCE_FILE ${EXEC_SOURCE_FILES_SRC})
        get_filename_component(FILE_EXT ${SOURCE_FILE} EXT)
        if("${FILE_EXT}" STREQUAL ".cu")
            list(APPEND _MRT_CUDA_SOURCE_FILES "${SOURCE_FILE}")
        else()
            list(APPEND _MRT_CPP_SOURCE_FILES "${SOURCE_FILE}")
        endif()
    endforeach()

    # generate the target
    message(STATUS "Adding executable \"${EXEC_NAME}\" with source: ${_MRT_CPP_SOURCE_FILES}")
    add_executable(${EXEC_TARGET_NAME} ${EXEC_SOURCE_FILES_INC} ${_MRT_CPP_SOURCE_FILES})
    set_target_properties(${EXEC_TARGET_NAME} PROPERTIES OUTPUT_NAME ${EXEC_NAME})
    # seems superfluous but is necessary to avoid quirks with autmoc'ed files
    if(MRT_ADD_EXECUTABLE_FOLDER)
        target_include_directories(${EXEC_TARGET_NAME}
                                   PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/${MRT_ADD_EXECUTABLE_FOLDER}")
    endif()
    mrt_add_links(${EXEC_TARGET_NAME})
    if(MRT_ADD_EXECUTABLE_DEPENDS)
        add_dependencies(${EXEC_TARGET_NAME} ${MRT_ADD_EXECUTABLE_DEPENDS})
    endif()
    if(MRT_ADD_EXECUTABLE_LIBRARIES)
        target_link_libraries(${EXEC_TARGET_NAME} PRIVATE ${MRT_ADD_EXECUTABLE_LIBRARIES})
    endif()
    if(${PROJECT_NAME}_GENERATED_LIBRARIES)
        target_link_libraries(${EXEC_TARGET_NAME} PRIVATE ${${PROJECT_NAME}_GENERATED_LIBRARIES})
    endif()

    # Add cuda target
    if(_MRT_CUDA_SOURCE_FILES)
        if(NOT DEFINED CUDA_FOUND)
            message(
                FATAL_ERROR
                    "Found CUDA source file but no dependency to CUDA. Please add <depend>CUDA</depend> to your package.xml."
            )
        endif()

        # generate cuda target
        set(CUDA_TARGET_NAME _${EXEC_TARGET_NAME}_cuda)

        # NVCC does not like '-' in file names and because 'cuda_add_library' creates
        # a helper file which contains the target name, one has to replace '-'.
        string(REPLACE "-" "_" CUDA_TARGET_NAME ${CUDA_TARGET_NAME})

        if(${CMAKE_VERSION} VERSION_LESS "3.9.0")
            cuda_add_library(${CUDA_TARGET_NAME} STATIC ${_MRT_CUDA_SOURCE_FILES})
        else()
            message(STATUS "Adding ${_MRT_CUDA_SOURCE_FILES} files.")
            add_library(${CUDA_TARGET_NAME} SHARED ${_MRT_CUDA_SOURCE_FILES})
            # We cannot link to all libraries as nvcc does not unterstand all the flags
            # etc. which could be passed to target_link_libraries as a target. So the
            # dependencies were added to the mrt_CUDA_LIBRARIES variable.
            set_property(TARGET ${CUDA_TARGET_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
            set_property(TARGET ${CUDA_TARGET_NAME} PROPERTY CUDA_SEPARABLE_COMPILATION ON)
        endif()
        mrt_add_links(${CUDA_TARGET_NAME} CUDA)
        target_link_libraries(${CUDA_TARGET_NAME} PRIVATE ${MRT_ADD_EXECUTABLE_LIBRARIES})

        # link cuda library to executable
        target_link_libraries(${EXEC_TARGET_NAME} PRIVATE ${CUDA_TARGET_NAME})
        set(${PROJECT_NAME}_GENERATED_LIBRARIES
            ${${PROJECT_NAME}_GENERATED_LIBRARIES} ${CUDA_TARGET_NAME}
            PARENT_SCOPE)
    endif()

    # append to list of all targets in this project
    set(${PROJECT_NAME}_MRT_TARGETS
        ${${PROJECT_NAME}_MRT_TARGETS} ${EXEC_TARGET_NAME} ${CUDA_TARGET_NAME}
        PARENT_SCOPE)
endfunction()

#
# Adds a nodelet.
#
# This command ensures the nodelet is compiled with all necessary dependencies. Make sure to add lib{PROJECT_NAME}-{NAME}-nodelet to the ``nodelet_plugins.xml`` file.
#
# .. note:: Make sure to call this after all messages and parameter generation CMAKE-Commands so that all dependencies are visible.
#
# The files are automatically added to the list of installable targets so that ``mrt_install`` can mark them for installation.
#
# It requires a ``*_nodelet.cpp``-File to be present in this folder.
# The command will look for a ``*_node.cpp``-file and remove it from the list of files to avoid ``main()``-functions to be compiled into the library.
#
# :param nodeletname: base name of the nodelet (_nodelet will be appended to the base name to avoid conflicts with library packages)
# :type nodeletname: string
# :param FOLDER: Folder with cpp files for the executable, relative to ``${PROJECT_SOURCE_DIR}``
# :type FOLDER: string
# :param DEPENDS: List of extra (non-catkin, non-mrt) CMAKE dependencies. This should only be required for including external projects.
# :type DEPENDS: list of strings
# :param LIBRARIES: Extra (non-catkin, non-mrt) libraries to link to. This should only be required for external projects
# :type LIBRARIES: list of strings
# :param TARGETNAME: Choose the name of the internal CMAKE target. Will be autogenerated if not specified.
# :type TARGETNAME: string
#
# Example:
# ::
#
#   mrt_add_nodelet( example_package
#       FOLDER src/example_package
#       )
#
# The resulting entry in the ``nodelet_plugins.xml`` is thus: <library path="lib/libexample_package-example_package-nodelet">
#
# @public
#
function(mrt_add_nodelet nodeletname)

    set(NODELET_NAME ${nodeletname})
    if(NOT NODELET_NAME)
        message(FATAL_ERROR "No nodelet name specified for call to mrt_add_nodelet()!")
    endif()
    cmake_parse_arguments(MRT_ADD_NODELET "" "FOLDER;TARGETNAME" "DEPENDS;LIBRARIES" ${ARGN})
    if(NOT MRT_ADD_NODELET_TARGETNAME)
        set(NODELET_TARGET_NAME ${PROJECT_NAME}-${NODELET_NAME}-nodelet)
    else()
        set(NODELET_TARGET_NAME ${MRT_ADD_NODELET_TARGETNAME})
    endif()

    # get the files
    mrt_glob_files(NODELET_SOURCE_FILES_INC "${MRT_ADD_NODELET_FOLDER}/*.h" "${MRT_ADD_NODELET_FOLDER}/*.hpp"
                   "${MRT_ADD_EXECUTABLE_FOLDER}/*.hh" "${MRT_ADD_EXECUTABLE_FOLDER}/*.cuh")
    mrt_glob_files(NODELET_SOURCE_FILES_SRC "${MRT_ADD_NODELET_FOLDER}/*.cpp" "${MRT_ADD_EXECUTABLE_FOLDER}/*.cc"
                   "${MRT_ADD_EXECUTABLE_FOLDER}/*.cu")

    # Find nodelet
    mrt_glob_files(NODELET_CPP "${MRT_ADD_NODELET_FOLDER}/*_nodelet.cpp" "${MRT_ADD_NODELET_FOLDER}/*_nodelet.cc")
    if(NOT NODELET_CPP)
        return()
    endif()

    # Remove nodes (with their main) from src-files
    mrt_glob_files(NODE_CPP "${MRT_ADD_NODELET_FOLDER}/*_node.cpp" "${MRT_ADD_NODELET_FOLDER}/*_node.cc")
    if(NODE_CPP)
        list(REMOVE_ITEM NODELET_SOURCE_FILES_SRC ${NODE_CPP})
    endif()

    # determine library name

    # generate the target
    message(STATUS "Adding nodelet \"${NODELET_NAME}\"")
    mrt_add_library(
        ${NODELET_TARGET_NAME}
        INCLUDES ${NODELET_SOURCE_FILES_INC}
        SOURCES ${NODELET_SOURCE_FILES_SRC}
        DEPENDS ${MRT_ADD_NODELET_DEPENDS}
        LIBRARIES ${MRT_ADD_NODELET_LIBRARIES})

    # the nodelet library used to be referenced as lib"nodeletname" in ros. For backward compability we still provide a symlink to older versions
    if(MRT_PKG_VERSION VERSION_LESS 3.1)
        string(REGEX REPLACE "_node" "" OLD_NODELET_NAME ${nodeletname})
        string(REGEX REPLACE "_nodelet" "" OLD_NODELET_NAME ${OLD_NODELET_NAME})
        set(OLD_NODELET_NAME ${OLD_NODELET_NAME}_nodelet)
        add_custom_command(
            TARGET ${NODELET_TARGET_NAME}
            POST_BUILD
            COMMAND cd $<TARGET_FILE_DIR:${NODELET_TARGET_NAME}> && ln -sf $<TARGET_FILE_NAME:${NODELET_TARGET_NAME}>
                    lib${OLD_NODELET_NAME}.so)
        install(FILES $<TARGET_FILE_DIR:${NODELET_TARGET_NAME}>/lib${OLD_NODELET_NAME}.so
                DESTINATION ${CMAKE_INSTALL_LIBDIR})
    endif()
    # make nodelet headers available in unittests
    get_filename_component(nodelet_dir ${CMAKE_CURRENT_SOURCE_DIR}/${MRT_ADD_NODELET_FOLDER} DIRECTORY)
    target_include_directories(${NODELET_TARGET_NAME} INTERFACE $<BUILD_INTERFACE:${nodelet_dir}>)
    # pass lists on to parent scope. We are not exporting the library, because they are not supposed to be used by external projects
    set(${PROJECT_NAME}_GENERATED_LIBRARIES
        ${${PROJECT_NAME}_GENERATED_LIBRARIES}
        PARENT_SCOPE)
endfunction()

#
# Adds a node and a corresponding nodelet.
#
# This command ensures the node/nodelet are compiled with all necessary dependencies. Make sure to add lib<package_name>-<basename>-nodelet to the ``nodelet_plugins.xml`` file.
#
# .. note:: Make sure to call this after all messages and parameter generation CMAKE-Commands so that all dependencies are visible.
#
# The files are automatically added to the list of installable targets so that ``mrt_install`` can mark them for installation.
#
# It requires a ``*_nodelet.cpp`` file and a ``*_node.cpp`` file to be present in this folder. It will then compile a nodelet-library, create an executable from the ``*_node.cpp`` file and link the executable with the nodelet library.
#
# Unless the variable ``${MRT_NO_FAIL_ON_TESTS}`` is set, failing unittests will result in a failed build.
#
# :param basename: base name of the node/nodelet (_nodelet will be appended for the nodelet name to avoid conflicts with library packages)
# :type basename: string
# :param FOLDER: Folder with cpp files for the executable, relative to ``${PROJECT_SOURCE_DIR}``
# :type FOLDER: string
# :param DEPENDS: List of extra (non-catkin, non-mrt) CMAKE dependencies. This should only be required for including external projects.
# :type DEPENDS: list of strings
# :param LIBRARIES: Extra (non-catkin, non-mrt) libraries to link to. This should only be required for external projects
# :type LIBRARIES: list of strings
#
# Example:
# ::
#
#   mrt_add_node_and_nodelet( example_package
#       FOLDER src/example_package
#       )
#
# The resulting entry in the ``nodelet_plugins.xml`` is thus (for a package named example_package): <library path="lib/libexample_package-example_package-nodelet">
#
# @public
#
function(mrt_add_node_and_nodelet basename)
    cmake_parse_arguments(MRT_ADD_NN "" "FOLDER" "DEPENDS;LIBRARIES" ${ARGN})
    set(BASE_NAME ${basename})
    if(NOT BASE_NAME)
        message(FATAL_ERROR "No base name specified for call to mrt_add_node_and_nodelet()!")
    endif()
    set(NODELET_TARGET_NAME ${PROJECT_NAME}-${BASE_NAME}-nodelet)
    # add nodelet
    mrt_add_nodelet(
        ${BASE_NAME}
        FOLDER ${MRT_ADD_NN_FOLDER}
        TARGETNAME ${NODELET_TARGET_NAME}
        DEPENDS ${MRT_ADD_NN_DEPENDS}
        LIBRARIES ${MRT_ADD_NN_LIBRARIES})
    # pass lists on to parent scope
    set(${PROJECT_NAME}_GENERATED_LIBRARIES
        ${${PROJECT_NAME}_GENERATED_LIBRARIES}
        PARENT_SCOPE)
    set(${PROJECT_NAME}_MRT_TARGETS
        ${${PROJECT_NAME}_MRT_TARGETS}
        PARENT_SCOPE)

    # search the files we have to build with
    if(NOT TARGET ${NODELET_TARGET_NAME})
        unset(NODELET_TARGET_NAME)
        mrt_glob_files(NODE_CPP "${MRT_ADD_NN_FOLDER}/*.cpp" "${MRT_ADD_NN_FOLDER}/*.cc" "${MRT_ADD_NN_FOLDER}/*.cu")
    else()
        mrt_glob_files(NODE_CPP "${MRT_ADD_NN_FOLDER}/*_node.cpp" "${MRT_ADD_NN_FOLDER}/*_node.cc")
    endif()

    # find *_node file containing the main() and add the executable
    mrt_glob_files(NODE_H "${MRT_ADD_NN_FOLDER}/*.h" "${MRT_ADD_NN_FOLDER}/*.hpp" "${MRT_ADD_NN_FOLDER}/*.hh"
                   "${MRT_ADD_NN_FOLDER}/*.cuh")
    mrt_glob_files(NODE_MAIN "${MRT_ADD_NN_FOLDER}/*_node.cpp" "${MRT_ADD_NN_FOLDER}/*_node.cc")
    if(NODE_MAIN)
        mrt_add_executable(
            ${BASE_NAME}
            FILES ${NODE_CPP} ${NODE_H}
            DEPENDS ${MRT_ADD_NN_DEPENDS} ${NODELET_TARGET_NAME}
            LIBRARIES ${MRT_ADD_NN_LIBRARIES} ${NODELET_TARGET_NAME})
        # pass lists on to parent scope
        set(${PROJECT_NAME}_GENERATED_LIBRARIES
            ${${PROJECT_NAME}_GENERATED_LIBRARIES}
            PARENT_SCOPE)
        set(${PROJECT_NAME}_MRT_TARGETS
            ${${PROJECT_NAME}_MRT_TARGETS}
            PARENT_SCOPE)
    endif()
endfunction()

#
# Adds all rostests (identified by a .test file) contained in a folder as unittests.
#
# If a .cpp file exists with the same name, it will be added and comiled as a gtest test.
# Unittests can be run with "catkin run_tests" or similar. "-test" will be appended to the name of the test node to avoid conflicts (i.e. the type argument should then be <test ... type="mytest-test"/> in a mytest.test file).
#
# Unittests will always be executed with the folder as cwd. E.g. if the test folder contains a sub-folder "test_data", it can simply be accessed as "test_data".
#
# If coverage information is enabled (by setting ``MRT_ENABLE_COVARAGE`` to true), coverage analysis will be performed after unittests have run. The results can be found in the package's build folder in the folder "coverage".
#
# Unless the variable ``${MRT_NO_FAIL_ON_TESTS}`` is set, failing unittests will result in a failed build.
#
# :param folder: folder containing the tests (relative to ``${PROJECT_SOURCE_DIR}``) as first argument
# :type folder: string
# :param LIBRARIES: Additional (non-catkin, non-mrt) libraries to link to
# :type LIBRARIES: list of strings
# :param DEPENDS: Additional (non-catkin, non-mrt) dependencies (e.g. with catkin_download_test_data)
# :type DEPENDS: list of strings
#
# Example:
# ::
#
#   mrt_add_ros_tests( test
#       )
#
# @public
#
function(mrt_add_ros_tests folder)
    set(TEST_FOLDER ${folder})
    cmake_parse_arguments(MRT_ADD_ROS_TESTS "" "" "LIBRARIES;DEPENDS" ${ARGN})
    mrt_glob_files(_ros_tests "${TEST_FOLDER}/*.test")
    add_custom_target(${PROJECT_NAME}-rostest_test_files SOURCES ${_ros_tests})
    configure_file(${MCM_TEMPLATE_DIR}/test_utility.hpp.in ${PROJECT_BINARY_DIR}/tests/test/test_utility.hpp @ONLY)
    mrt_init_testing()

    foreach(_ros_test ${_ros_tests})
        get_filename_component(_test_name ${_ros_test} NAME_WE)
        # make sure we add only one -test to the target
        string(REGEX REPLACE "-test" "" TEST_NAME ${_test_name})
        set(TEST_NAME ${TEST_NAME}-test)
        set(TEST_TARGET_NAME ${PROJECT_NAME}-rostest-${TEST_NAME})
        # look for a matching .cpp
        if(EXISTS "${PROJECT_SOURCE_DIR}/${TEST_FOLDER}/${_test_name}.cpp")
            message(STATUS "Adding gtest-rostest \"${TEST_TARGET_NAME}\" with test file ${_ros_test}")
            mrt_add_rostest_gtest(${TEST_TARGET_NAME} ${_ros_test} "${TEST_FOLDER}/${_test_name}.cpp")
            mrt_add_links(${TEST_TARGET_NAME} TEST)
            target_include_directories(${TEST_TARGET_NAME} PRIVATE ${PROJECT_BINARY_DIR}/tests)
            if(${PROJECT_NAME}_GENERATED_LIBRARIES)
                target_link_libraries(${TEST_TARGET_NAME} PRIVATE ${${PROJECT_NAME}_GENERATED_LIBRARIES})
            endif()
            if(MRT_ADD_ROS_TESTS_DEPENDS)
                add_dependencies(${TEST_TARGET_NAME} ${MRT_ADD_ROS_TESTS_DEPENDS})
            endif()
            set_target_properties(${TEST_TARGET_NAME} PROPERTIES OUTPUT_NAME ${TEST_NAME})
            set(TARGET_ADDED True)
        else()
            message(STATUS "Adding plain rostest \"${_ros_test}\"")
            mrt_add_rostest(${TEST_TARGET_NAME} ${_ros_test})
        endif()
    endforeach()
endfunction()

#
# Adds all gtests (without a corresponding .test file) contained in a folder as unittests.
#
# :param folder: folder containing the tests (relative to ``${PROJECT_SOURCE_DIR}``) as first argument
# :type folder: string
# :param LIBRARIES: Additional (non-catkin, non-mrt) libraries to link to
# :type LIBRARIES: list of strings
# :param DEPENDS: Additional (non-catkin, non-mrt) dependencies (e.g. with catkin_download_test_data)
# :type DEPENDS: list of strings
#
#
# Unittests will be executed with the folder as cwd if ctest or the run_test target is used. E.g. if the test folder contains a sub-folder "test_data", it can simply be accessed as "test_data".
# Another way of getting the location of the project root folder path is to ``#include "test/test_utility.hpp"`` and use the variable ``<project_name>::test::projectRootDir``.
#
# Unless the variable ``${MRT_NO_FAIL_ON_TESTS}`` is set, failing unittests will result in a failed build.
#
# If coverage information is enabled (by setting ``MRT_ENABLE_COVARAGE`` to true), coverage analysis will be performed after unittests have run. The results can be found in the package's build folder in the folder "coverage".
#
# Example:
# ::
#
#   mrt_add_tests( test
#       )
#
# @public
#
function(mrt_add_tests folder)
    set(TEST_FOLDER ${folder})
    cmake_parse_arguments(MRT_ADD_TESTS "" "" "LIBRARIES;DEPENDS" ${ARGN})
    mrt_glob_files(_tests "${TEST_FOLDER}/*.cpp" "${TEST_FOLDER}/*.cc")
    configure_file(${MCM_TEMPLATE_DIR}/test_utility.hpp.in ${PROJECT_BINARY_DIR}/tests/test/test_utility.hpp @ONLY)
    mrt_init_testing()

    foreach(_test ${_tests})
        get_filename_component(_test_name ${_test} NAME_WE)
        # make sure we add only one -test to the target
        string(REGEX REPLACE "-test" "" TEST_TARGET_NAME ${_test_name})
        set(TEST_TARGET_NAME ${PROJECT_NAME}-gtest-${TEST_TARGET_NAME})
        # exclude cpp files with a test file (those are ros tests)
        if(NOT EXISTS "${PROJECT_SOURCE_DIR}/${TEST_FOLDER}/${_test_name}.test")
            message(
                STATUS
                    "Adding gtest unittest \"${TEST_TARGET_NAME}\" with working dir ${PROJECT_SOURCE_DIR}/${TEST_FOLDER}"
            )
            mrt_add_gtest(${TEST_TARGET_NAME} ${_test} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/${TEST_FOLDER})
            mrt_add_links(${TEST_TARGET_NAME} TEST)
            target_include_directories(${TEST_TARGET_NAME} PRIVATE ${PROJECT_BINARY_DIR}/tests)
            if(${PROJECT_NAME}_GENERATED_LIBRARIES)
                target_link_libraries(${TEST_TARGET_NAME} PRIVATE ${${PROJECT_NAME}_GENERATED_LIBRARIES})
            endif()
            if(MRT_ADD_TESTS_DEPENDS)
                add_dependencies(${TEST_TARGET_NAME} ${MRT_ADD_TESTS_DEPENDS})
            endif()
            set(TARGET_ADDED True)
        endif()
    endforeach()
endfunction()

# Adds python nosetest contained in a folder. Wraps the function catkin_add_nosetests.
#
# :param folder: folder containing the tests (relative to ``${PROJECT_SOURCE_DIR}``) as first argument
# :type folder: string
# :param DEPENDS: Additional (non-catkin, non-mrt) dependencies (e.g. with catkin_download_test_data)
# :type DEPENDS: list of strings
# :param DEPENDENCIES: Alias for DEPENDS
# :type DEPENDENCIES: list of strings
#
# Example:
# ::
#
#   mrt_add_nosetests(test)
#
# @public
#
function(mrt_add_nosetests folder)
    mrt_init_testing()
    set(TEST_FOLDER ${folder})
    cmake_parse_arguments(ARG "" "" "DEPENDS;DEPENDENCIES" ${ARGN})
    if(NOT IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${TEST_FOLDER})
        return()
    endif()

    message(STATUS "Adding nosetests in folder ${PROJECT_SOURCE_DIR}/${TEST_FOLDER}")
    _mrt_add_nosetests_impl(${TEST_FOLDER} DEPENDS ${ARG_DEPENDS} ${ARG_DEPENDENCIES})
endfunction()

function(_mrt_install_python source_file destination)
    if(ROS_VERSION EQUAL 1)
        # we can just use catkin
        catkin_install_python(PROGRAMS ${file} DESTINATION ${destination})
        return()
    endif()
    # for ament, we have to fix the shebang before we can install (ament does not provide this feature)

    # read file and check shebang line
    file(READ ${source_file} data)
    set(regex "^#![ \t]*/([^\r\n]+)/env[ \t]+python([\r\n])")
    string(REGEX MATCH "${regex}" shebang_line "${data}")
    string(LENGTH "${shebang_line}" length)
    string(SUBSTRING "${data}" 0 ${length} prefix)
    if("${shebang_line}" STREQUAL "${prefix}")
        # write modified file with modified shebang line
        get_filename_component(python_name ${PYTHON_EXECUTABLE} NAME)
        string(REGEX REPLACE "${regex}" "#!/\\1/env ${python_name}\\2" data "${data}")
        get_filename_component(filename ${source_file} NAME)
        set(rewritten_file "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/python_files")
        file(MAKE_DIRECTORY ${rewritten_file})
        set(rewritten_file "${rewritten_file}/${filename}")
        file(WRITE ${rewritten_file} "${data}")
    else()
        # Shebang did not match, install file unmodified
        set(rewritten_file "${source_file}")
    endif()
    # install (modified) file to destination
    # Install copy of file with re-written shebang to install space
    install(PROGRAMS "${rewritten_file}" DESTINATION "${destination}")
endfunction()

# Installs all relevant project files.
#
# All targets added by the mrt_add_<library/executable/nodelet/...> commands will be installed automatically when using this command. Other files/folders (launchfiles, scripts) need to be specified explicitly.
# Non existing files and folders will be silently ignored.
#
# If the project contains a file cmake/<project-name>-extras.cmake[.in], it will be automatically included by all depending projects. This allows to export cmake functions, etc. If the file ends with [.in], it will be configured by cmake
# and the variables "@DEVELSPACE@" "@INSTALLSPACE@" will be replaced by true or files depending on the build type, @PROJECT_SPACE_DIR@ to the devel/install folder and @PKT_CMAKE_DIR@ to the cmake folder
#
# :param PROGRAMS: List of all folders and files that are programs (python scripts will be indentified and treated separately). Files will be made executable.
# :type PROGRAMS: list of strings
# :param FILES: List of non-executable files and folders. Subfolders will be installed recursively.
# :type FILES: list of strings
# :param EXPORT_LIBS: List of non-mrt-created libraries that should be installed and exported
# :type EXPORT_LIBS: list of (interface) library targets
#
# Example:
# ::
#
#   mrt_install(
#       PROGRAMS scripts
#       FILES launch nodelet_plugins.xml
#       )
#
# @public
#
function(mrt_install)
    cmake_parse_arguments(MRT_INSTALL "" "" "PROGRAMS;FILES;EXPORT_LIBS" ${ARGN})
    # install targets
    set(export_targets ${${PROJECT_NAME}_GENERATED_LIBRARIES} ${EXPORT_EXPORT_LIBS})
    if(export_targets AND TARGET ${PROJECT_NAME}_compiler_flags)
        # export the public compiler flags of this project
        list(APPEND export_targets ${PROJECT_NAME}_compiler_flags)
    endif()

    # static libs require our private flags to be in the export set
    foreach(target ${export_targets})
        get_target_property(target_type ${target} TYPE)
        if(target_type STREQUAL "STATIC_LIBRARY")
            set(has_static True)
            break()
        endif()
    endforeach()
    if(has_static AND TARGET ${PROJECT_NAME}_private_compiler_flags)
        list(APPEND export_targets ${PROJECT_NAME}_private_compiler_flags)
    endif()
    if(has_static AND TARGET ${PROJECT_NAME}_sanitizer_lib_flags)
        list(APPEND export_targets ${PROJECT_NAME}_sanitizer_lib_flags)
    endif()

    set(install_bin ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME})

    # this is how catkin does it (instead of "bin"). We might think about adding a way to override this...
    if(export_targets)
        message(
            STATUS
                "Marking libraries \"${${PROJECT_NAME}_MRT_TARGETS}\" of package \"${PROJECT_NAME}\" for installation")
        install(
            TARGETS ${export_targets}
            EXPORT ${PROJECT_NAME}_EXPORTS
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${install_bin})
        set(package_exports ${PROJECT_NAME}_EXPORTS)
    endif()
    set(remaining_targets ${${PROJECT_NAME}_MRT_TARGETS})
    if(${PROJECT_NAME}_GENERATED_LIBRARIES)
        list(REMOVE_ITEM remaining_targets ${${PROJECT_NAME}_GENERATED_LIBRARIES})
    endif()
    if(remaining_targets)
        if(${PROJECT_NAME}_MRT_TARGETS)
            message(STATUS "Marking targets \"${remaining_targets}\" of package \"${PROJECT_NAME}\" for installation")
            install(
                TARGETS ${remaining_targets}
                ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
                LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
                RUNTIME DESTINATION ${install_bin})
        endif()
    endif()

    # install header
    if(EXISTS ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})
        message(
            STATUS
                "Marking HEADER FILES in \"include/${PROJECT_NAME}\" folder of package \"${PROJECT_NAME}\" for installation"
        )
        # the "/" at the end of the path matters!
        # TODO: Cmake 3.10(?) supports installing with "TYPE HEADER". That removes the need of "FILES_MATCHING PATTERN".
        install(
            DIRECTORY include/${PROJECT_NAME}/
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
            FILES_MATCHING
            PATTERN "*.h"
            PATTERN "*.hpp"
            PATTERN "*.hh"
            PATTERN "*.cuh")
    endif()

    # helper function for installing programs
    function(mrt_install_program program_path)
        get_filename_component(extension ${program_path} EXT)
        get_filename_component(program ${program_path} NAME)
        if("${extension}" STREQUAL ".py")
            message(STATUS "Marking PYTHON PROGRAM \"${program}\" of package \"${PROJECT_NAME}\" for installation")
            _mrt_install_python(${program_path} ${install_bin})
        else()
            message(STATUS "Marking PROGRAM \"${program}\" of package \"${PROJECT_NAME}\" for installation")
            install(PROGRAMS ${program_path} DESTINATION ${install_bin})
        endif()
    endfunction()

    # install programs
    foreach(ELEMENT ${MRT_INSTALL_PROGRAMS})
        if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${ELEMENT})
            mrt_glob_files(FILES "${PROJECT_SOURCE_DIR}/${ELEMENT}/[^.]*[^~]")
            foreach(FILE ${FILES})
                if(NOT IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${FILE})
                    mrt_install_program(${FILE})
                endif()
            endforeach()
        elseif(EXISTS ${PROJECT_SOURCE_DIR}/${ELEMENT})
            mrt_install_program(${ELEMENT})
        endif()
    endforeach()

    # install files
    foreach(ELEMENT ${MRT_INSTALL_FILES})
        if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${ELEMENT})
            message(
                STATUS "Marking SHARED CONTENT FOLDER \"${ELEMENT}\" of package \"${PROJECT_NAME}\" for installation")
            install(DIRECTORY ${ELEMENT} DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})
        elseif(EXISTS ${PROJECT_SOURCE_DIR}/${ELEMENT})
            message(STATUS "Marking FILE \"${ELEMENT}\" of package \"${PROJECT_NAME}\" for installation")
            install(FILES ${ELEMENT} DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})
        endif()
    endforeach()

    # export the config
    # ensure catkin_package wasnt already called before. It should already have generated a configuration.
    if(NOT ${PROJECT_NAME}_CATKIN_PACKAGE)
        set(${PROJECT_NAME}_CATKIN_PACKAGE TRUE)
        include(${MRT_CMAKE_MODULES_CMAKE_PATH}/Modules/ExportPackage.cmake)
        _mrt_export_package(
            EXPORTS ${package_exports}
            LIBRARIES ${export_targets}
            TARGETS ${remaining_targets} ${${PROJECT_NAME}_EXPORTED_TARGETS})
    endif()

    # generate environment files for ros2
    if(ROS_VERSION EQUAL 2)
        find_package(ament_cmake_core REQUIRED)
        ament_execute_extensions(ament_package)
    endif()
endfunction()
