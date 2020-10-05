@  |  @ # This file contains the different versions for which file stuff is used
@  |  @ # for ros, non-ros and header only library, library or executable
@  |  @ #
@  |  @ # The mask is as follows
@  |  @ # @12|34@
@  |  @ # pos1: non ros package
@  |  @ # pos2: ros package
@  |  @ # pos3: library
@  |  @ # pos4: executable
@  |  @ #
@  |  @ # Put a "x" at the position to enable this line
@  |  @ # E.g a non-ros library has the following mask: @x |x @
@xx|xx@ set(MRT_PKG_VERSION 4.0.0)
@xx|xx@ # Modify only if you know what you are doing!
@xx|xx@ cmake_minimum_required(VERSION 3.5.1)
@xx|xx@ project(${CMAKE_PACKAGE_NAME})
@xx|xx@ 
@xx|xx@ ###################
@xx|xx@ ## Find packages ##
@xx|xx@ ###################
@xx|xx@ find_package(mrt_cmake_modules REQUIRED)
@xx|xx@ include(UseMrtStdCompilerFlags)
@xx|xx@ include(GatherDeps)
@xx|xx@ 
@xx|xx@ # You can add a custom.cmake in order to add special handling for this package. E.g. you can do:
@xx|xx@ # list(REMOVE_ITEM DEPENDEND_PACKAGES <package name 1> <package name 2> ...)
@xx|xx@ # To remove libs which cannot be found automatically. You can also "find_package" other, custom dependencies there.
@xx|xx@ # You can also set PROJECT_INSTALL_FILES to install files that are not installed by default.
@ x|xx@ # For packages that generate ROS messages, you can do e.g. the following to ensure messages are found:
@ x|xx@ # set(PROJECT_MESSAGE_DEPENDS std_msgs geometry_msgs)
@xx|xx@ if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake")
@xx|xx@     include("${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake")
@xx|xx@ endif()
@xx|xx@ 
@xx|xx@ find_package(AutoDeps REQUIRED COMPONENTS ${DEPENDEND_PACKAGES})
@xx|xx@ 
@xx|xx@ mrt_parse_package_xml()
@xx|xx@ 
@xx|xx@ ########################
@xx|xx@ ## Add python modules ##
@xx|xx@ ########################
@xx|xx@ # This adds a python module if located under src/{PROJECT_NAME)
@xx|xx@ mrt_python_module_setup()
@xx|xx@ 
@xx|x @ mrt_glob_files(PROJECT_PYTHON_SOURCE_FILES_SRC "python_api/*.cpp")
@xx|x @ if (PROJECT_PYTHON_SOURCE_FILES_SRC)
@xx|x @     # Add a cpp-python api library. Make sure there are no name collisions with python modules in this project
@xx|x @     mrt_add_python_api( ${PROJECT_NAME}
@xx|x @         FILES ${PROJECT_PYTHON_SOURCE_FILES_SRC}
@xx|x @         )
@xx|x @ endif()
@xx|x @ 
@ x|xx@ ################################################
@ x|xx@ ## Declare ROS messages, services and actions ##
@ x|xx@ ################################################
@ x|xx@ 
@ x|xx@ # Add message, service and action files
@ x|xx@ mrt_add_message_files(msg)
@ x|xx@ mrt_add_service_files(srv)
@ x|xx@ mrt_add_action_files(action)
@ x|xx@ 
@ x|xx@ # Generate added messages and services with any dependencies listed here
@ x|xx@ if (ROS_GENERATE_MESSAGES)
@ x|xx@     generate_messages(
@ x|xx@         DEPENDENCIES ${PROJECT_MESSAGE_DEPENDS}
@ x|xx@         )
@ x|xx@ endif()
@ x|xx@ 
@ x|xx@ # Generate dynamic reconfigure options
@ x|xx@ mrt_glob_files(PARAMS_FILES "cfg/*.params" "cfg/*.cfg" "cfg/*.mrtcfg" "cfg/*.rosif")
@ x|xx@ if (PARAMS_FILES)
@ x|xx@     generate_ros_parameter_files(${PARAMS_FILES})
@ x|xx@     generate_ros_interface_files(${PARAMS_FILES})
@ x|xx@ endif()
@ x|xx@ 
@xx|x @ ############################
@xx|x @ ## Read source code files ##
@xx|x @ ############################
@xx|x @ mrt_glob_files_recurse(PROJECT_HEADER_FILES_INC "include/*.h" "include/*.hpp" "include/*.cuh")
@xx|x @ mrt_glob_files(PROJECT_SOURCE_FILES_INC "src/*.h" "src/*.hpp" "src/*.cuh")
@xx|x @ mrt_glob_files(PROJECT_SOURCE_FILES_SRC "src/*.cpp" "src/*.cu")
@xx|xx@ 
@xx|xx@ ###########
@xx|xx@ ## Build ##
@xx|xx@ ###########
@xx|x @ # Declare a cpp library
@xx|x @ mrt_add_library(${PROJECT_NAME}
@xx|x @     INCLUDES ${PROJECT_HEADER_FILES_INC} ${PROJECT_SOURCE_FILES_INC}
@xx|x @     SOURCES ${PROJECT_SOURCE_FILES_SRC}
@xx|x @     )
@xx| x@ mrt_glob_folders(SRC_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/src")
@xx| x@ if (SRC_DIRECTORIES)
@xx| x@     # Found subfolders, add executable for each subfolder
@xx| x@     foreach(SRC_DIR ${SRC_DIRECTORIES})
@x | x@         mrt_add_executable(${SRC_DIR} FOLDER "src/${SRC_DIR}")
@ x| x@         mrt_add_node_and_nodelet(${SRC_DIR} FOLDER "src/${SRC_DIR}")
@xx| x@     endforeach()
@xx| x@ else()
@xx| x@     # No subfolder found, add executable and python modules for src folder
@x | x@     mrt_add_executable(${PROJECT_NAME} FOLDER "src")
@ x| x@     mrt_add_node_and_nodelet(${PROJECT_NAME} FOLDER "src")
@xx| x@ endif()
@xx|xx@ 
@xx|xx@ #############
@xx|xx@ ## Install ##
@xx|xx@ #############
@xx|xx@ # Install all targets, headers by default and scripts and other files if specified (folders or files).
@xx|xx@ # This command also exports libraries and config files for dependent packages and this supersedes catkin_package.
@ x|xx@ mrt_install(PROGRAMS scripts FILES launch rviz maps res data nodelet_plugins.xml plugin_description.xml ${PROJECT_INSTALL_FILES})
@x |xx@ mrt_install(PROGRAMS scripts FILES res data ${PROJECT_INSTALL_FILES})
@xx|xx@ 
@xx|xx@ #############
@xx|xx@ ## Testing ##
@xx|xx@ #############
@xx|xx@ # Add test targets for cpp and python tests
@xx|xx@ if (CATKIN_ENABLE_TESTING)
@ x| x@     # Include header in src folder via src/<pkgname>/<myheader>.h
@ x|xx@     mrt_add_ros_tests(test)
@ x|x @     mrt_add_tests(test)
@x |xx@     mrt_add_tests(test)
@xx|xx@     mrt_add_nosetests(test)
@xx|xx@ endif()
