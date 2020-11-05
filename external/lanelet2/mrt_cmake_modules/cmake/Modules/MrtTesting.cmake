function(mrt_init_testing)
    # initialize target structure
    if(TARGET tests_${PROJECT_NAME})
        return() # mrt_init_testing was already called
    endif()
    if(NOT TARGET tests)
        if(NOT ROS_VERSION EQUAL 1)
            # out of ros1 tests are always built by default, so we add it to the all target
            set(all ALL)
        endif()
        add_custom_target(tests ${all})
    endif()
    if(NOT TARGET run_tests)
        # usually catkin does that for us
        add_custom_target(run_tests)
        enable_testing()
    endif()
    if(NOT TARGET run_tests_${PROJECT_NAME})
        add_custom_target(run_tests_${PROJECT_NAME})
        add_dependencies(run_tests run_tests_${PROJECT_NAME})
    endif()
    add_custom_target(tests_${PROJECT_NAME})

    # make sure all targets within this project are built first (also messages and python binaries)
    set(_combined_deps ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${${PROJECT_NAME}_MRT_TARGETS})
    if(_combined_deps)
        add_dependencies(tests ${_combined_deps})
    endif()
    set(_test_results ${CMAKE_BINARY_DIR}/test_results)
    if(DEFINED CATKIN_TEST_RESULTS_DIR)
        # support overriding to be compatible with catkin
        set(_test_results ${CATKIN_TEST_RESULTS_DIR})
    endif()
    set(MRT_TEST_RESULTS_DIR
        ${_test_results}
        CACHE INTERNAL "basedir where test xmls will be written (in subfolder <project_name>)")
    if(MRT_ENABLE_COVERAGE)
        # these variables are global (for compability with toplevel cmakelists)
        # this also means they might still be in the cache, even if mrt_enable_coverage is now different
        set(MRT_COVERAGE_DIR
            ${CMAKE_BINARY_DIR}/mrt_coverage
            CACHE INTERNAL "")
        set(coverage_dir ${MRT_COVERAGE_DIR})
        message(STATUS "Outputting coverage to ${coverage_dir}")
    endif()
    if(NOT TARGET init_tests)
        set(pre_test_cmd
            ${PYTHON_EXECUTABLE}
            ${MRT_CMAKE_MODULES_ROOT_PATH}/scripts/init_coverage.py
            ${PROJECT_NAME}
            ${CMAKE_BINARY_DIR}
            ${CMAKE_CURRENT_LIST_DIR}
            ${MRT_TEST_RESULTS_DIR}/${PROJECT_NAME}
            ${coverage_dir})
        add_custom_target(
            init_tests
            COMMAND ${pre_test_cmd}
            COMMENT "Initializing unittests"
            VERBATIM)
        if(MRT_ENABLE_COVERAGE AND MRT_ENABLE_COVERAGE GREATER 1)
            set(show_result "--show")
        endif()
        set(post_test_cmd
            ${PYTHON_EXECUTABLE}
            ${MRT_CMAKE_MODULES_ROOT_PATH}/scripts/eval_coverage.py
            ${CMAKE_SOURCE_DIR}
            ${CMAKE_BINARY_DIR}
            ${MRT_TEST_RESULTS_DIR}
            ${coverage_dir}
            ${show_result})
        if(NOT MRT_NO_FAIL_ON_TESTS)
            set(fail_on_test --fail_on_test)
        endif()
        add_custom_target(
            eval_tests
            COMMAND ${post_test_cmd} --coverage_stderr ${fail_on_test}
            COMMENT "Evaluating unittest results"
            VERBATIM)
        add_dependencies(init_tests tests)
        add_dependencies(eval_tests init_tests)
        add_dependencies(run_tests eval_tests)

        if(CMAKE_VERSION VERSION_GREATER "3.12")
            include(ProcessorCount)
            ProcessorCount(cores)
            set(make_jobs "--parallel ${cores}")
        endif()

        # configure ctest
        string(REPLACE ";" " " pre_test_cmd "${pre_test_cmd}")
        string(REPLACE ";" " " post_test_cmd "${post_test_cmd}")
        add_test(build_tests ${CATKIN_ENV} bash -c
                 "\"${CMAKE_COMMAND}\" --build ${CMAKE_BINARY_DIR} ${make_jobs} --target tests && ${pre_test_cmd}")
        configure_file(${MCM_TEMPLATE_DIR}/CTestCustom.cmake.in "${CMAKE_CURRENT_BINARY_DIR}/CTestCustom.cmake" @ONLY)
        configure_file(${MCM_TEMPLATE_DIR}/CTestCustom.cmake.in "${CMAKE_BINARY_DIR}/CTestCustom.cmake" @ONLY)
        configure_file(${CMAKE_ROOT}/Modules/DartConfiguration.tcl.in
                       ${CMAKE_CURRENT_BINARY_DIR}/CTestConfiguration.ini)
        configure_file(${CMAKE_ROOT}/Modules/DartConfiguration.tcl.in ${PROJECT_BINARY_DIR}/CTestConfiguration.ini)
    endif()
    # add the dependencies between targets
    add_dependencies(tests tests_${PROJECT_NAME})
endfunction()

# All paths have to be absolute
function(_mrt_run_test target_name working_dir result_xml_path)
    cmake_parse_arguments(ARG "REDIRECT_STDERR" "COVERAGE_DIR" "COMMAND" ${ARGN})
    set(cmd ${ARG_COMMAND})
    set(working_dir_arg "--working-dir" ${working_dir})
    if(ARG_COVERAGE_DIR)
        set(coverage_dir_arg "--coverage-dir" ${ARG_COVERAGE_DIR})
    endif()
    if(ARG_REDIRECT_STDERR)
        set(redirect_arg "--redirect-stderr")
    endif()

    set(run_test_cmd
        ${CATKIN_ENV}
        ${PYTHON_EXECUTABLE}
        ${MRT_CMAKE_MODULES_ROOT_PATH}/scripts/run_test.py
        ${result_xml_path}
        ${working_dir_arg}
        ${coverage_dir_arg}
        ${redirect_arg})

    # for ctest the command needs to return non-zero if any test failed
    set(run_test_cmd_fail ${run_test_cmd} "--return-code" ${cmd})
    add_test(NAME ${target_name} COMMAND ${run_test_cmd_fail})

    # dependency for ctest to build the tests first
    set_tests_properties(${target_name} PROPERTIES DEPENDS build_tests)

    # for the run_tests target the command needs to return zero so that testing is not aborted
    add_custom_target(
        run_test_${target_name}
        COMMAND ${run_test_cmd} ${cmd}
        VERBATIM)
    add_dependencies(eval_tests run_test_${target_name})
    add_dependencies(run_test_${target_name} init_tests)
endfunction()

function(_mrt_create_executable_gtest target file)
    if(NOT TARGET gtest_main)
        # gtest_vendor is used on ros2 and should be preferred if available
        find_package(gtest_vendor QUIET)
        # add googletest as subdir to this project
        find_file(
            gtest_sources "gtest.cc"
            PATH_SUFFIXES "../src/googletest/googletest/src" "googletest/googletest/src" "src"
            HINTS ${CMAKE_CURRENT_LIST_DIR} ${MRT_GTEST_DIR} ${gtest_vendor_BASE_DIR})
        if(NOT gtest_sources)
            message(FATAL_ERROR "Failed to find the source files of googletest!")
        endif()
        get_filename_component(gtest_src ${gtest_sources} PATH)
        get_filename_component(gtest_base ${gtest_src} PATH)
        if(NOT gtest_vendor_BASE_DIR)
            get_filename_component(gtest_base ${gtest_base} PATH)
        endif()
        if(NOT EXISTS ${gtest_base}/CMakeLists.txt)
            message(FATAL_ERROR "Failed to find googletest base directory at: ${gtest_base}!")
        endif()
        if(NOT ROS_VERSION EQUAL 2)
            set(exclude EXCLUDE_FROM_ALL) # ament has trouble if library targets are not built in the all target
        endif()
        # by default, googletest installs itself too. We have to disable this behaviour.
        set(INSTALL_GTEST
            OFF
            CACHE BOOL "Enable installation of googletest")
        add_subdirectory(${gtest_base} ${CMAKE_CURRENT_BINARY_DIR}/gtest ${exclude})
        if(NOT TARGET gtest_main)
            message(FATAL_ERROR "Gtest seems not to build gtest_main!")
        endif()
        if(gtest_vendor_BASE_DIR)
            # no, gtest_vendor doesn't set its own include dir *sigh*
            target_include_directories(gtest_main BEFORE PUBLIC "${gtest_vendor_BASE_DIR}/include")
        endif()
    endif()
    message(STATUS "Creating gtest '${target}' from ${file}")
    add_executable(${target} ${file})
    set_target_properties(${target} PROPERTIES EXCLUDE_FROM_ALL TRUE)
    target_link_libraries(${target} PRIVATE gtest_main)
endfunction()

function(mrt_add_gtest target file)
    mrt_init_testing()
    cmake_parse_arguments(ARG "" "WORKING_DIRECTORY" "" ${ARGN})
    _mrt_create_executable_gtest(${target} ${file})
    add_dependencies(tests_${PROJECT_NAME} ${target})
    set(result_xml_path ${MRT_TEST_RESULTS_DIR}/${PROJECT_NAME}/gtest-${target}.xml)
    set(cmd "$<TARGET_FILE:${target}> --gtest_color=yes --gtest_output=xml:${result_xml_path}")
    if(MRT_ENABLE_COVERAGE)
        set(coverage_arg COVERAGE_DIR ${MRT_COVERAGE_DIR}/${target})
    endif()
    _mrt_run_test(
        ${target} ${ARG_WORKING_DIRECTORY} ${result_xml_path}
        ${coverage_arg}
        COMMAND ${cmd})
endfunction()

function(mrt_add_rostest target launch_file)
    mrt_init_testing()
    get_filename_component(test_name ${launch_file} NAME_WE)
    set(result_xml_path "${MRT_TEST_RESULTS_DIR}/${PROJECT_NAME}/rostest_${test_name}.xml")
    find_program(ROSTEST_EXE rostest)
    if(NOT ROSTEST_EXE)
        message(FATAL_ERROR "rostest not found! Aborting...")
    endif()
    set(cmd
        "${PYTHON_EXECUTABLE} ${ROSTEST_EXE} --pkgdir=${PROJECT_SOURCE_DIR} --package=${PROJECT_NAME} --results-filename ${test_name}.xml --results-base-dir \"${MRT_TEST_RESULTS_DIR}\" ${CMAKE_CURRENT_LIST_DIR}/${launch_file}"
    )
    if(MRT_ENABLE_COVERAGE)
        set(coverage_arg COVERAGE_DIR ${MRT_COVERAGE_DIR}/${target})
    endif()

    get_filename_component(test_name ${launch_file} NAME_WE)
    # rostest appends "rostest-" to the file name. This behaviour is apparently undocumented...
    set(result_xml_path "${MRT_TEST_RESULTS_DIR}/${PROJECT_NAME}/rostest-${test_name}.xml")
    _mrt_run_test(
        ${target} ${CMAKE_CURRENT_BINARY_DIR} ${result_xml_path}
        ${coverage_arg}
        COMMAND ${cmd})
endfunction()

function(mrt_add_rostest_gtest target launch_file cpp_file)
    mrt_init_testing()
    _mrt_create_executable_gtest(${target} ${cpp_file})
    add_dependencies(tests_${PROJECT_NAME} ${target})
    mrt_add_rostest(${target} ${launch_file})
endfunction()

function(_mrt_add_nosetests_impl folder)
    find_program(
        NOSETESTS
        NAMES "nosetests${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}"
              "nosetests-${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}" "nosetests${PYTHON_VERSION_MAJOR}"
              "nosetests-${PYTHON_VERSION_MAJOR}" "nosetests")
    if(NOSETESTS)
        message(STATUS "Using Python nosetests: ${NOSETESTS}")
    else()
        if("${PYTHON_VERSION_MAJOR}" STREQUAL "3")
            message(STATUS "nosetests not found, Python tests can not be run (try installing package 'python3-nose')")
        else()
            message(STATUS "nosetests not found, Python tests can not be run (try installing package 'python-nose')")
        endif()
        return()
    endif()

    cmake_parse_arguments(_nose "" "TIMEOUT" "DEPENDS" ${ARGN})
    if(NOT _nose_TIMEOUT)
        set(_nose_TIMEOUT 60)
    endif()
    if(NOT _nose_TIMEOUT GREATER 0)
        message(FATAL_ERROR "nosetests() TIMEOUT argument must be a valid number of seconds greater than zero")
    endif()

    set(path ${CMAKE_CURRENT_LIST_DIR}/${folder})

    # strip PROJECT_SOURCE_DIR and PROJECT_BINARY_DIR prefix from output_file_name
    set(output_file_name ${folder})
    string(REPLACE "/" "." output_file_name ${output_file_name})
    string(REPLACE ":" "." output_file_name ${output_file_name})

    set(test_name nosetests-${PROJECT_NAME}-${output_file_name})
    message(STATUS "Adding nosetest ${test_name}")
    # check if coverage reports are being requested
    if(MRT_ENABLE_COVERAGE)
        # we dont want to enable coverage when there are only python rostests around (ie executable python files)
        # because these are not executed by nosetests.
        mrt_glob_files(has_init_py src/${PROJECT_NAME}/__init__.py)
        mrt_glob_files(has_test_py ${folder}/*.py)
        execute_process(COMMAND find ${path} -name *.py -type f -not -executable OUTPUT_VARIABLE test_py_files)
        execute_process(COMMAND find ${path} -name *.py -type f -executable OUTPUT_VARIABLE exec_py_files)
        if(has_init_py AND (test_py_files OR NOT exec_py_files))
            find_program(PY_COVERAGE NAMES "python-coverage${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}"
                                           "python-coverage${PYTHON_VERSION_MAJOR}" "python-coverage")
            if(PY_COVERAGE)
                message(STATUS "Enabling coverage for python tests")
                set(covarg
                    " --with-coverage --cover-erase --cover-inclusive --cover-package=src/${PROJECT_NAME} --cover-xml --cover-xml-file=\"${MRT_COVERAGE_DIR}/${test_name}_coverage.xml\""
                )
            endif()
        endif()
        set(cover_dir_arg COVERAGE_DIR ${MRT_COVERAGE_DIR}/${test_name})
    endif()

    set(output_path ${MRT_TEST_RESULTS_DIR}/${PROJECT_NAME})
    set(tests "--where=${path}")
    set(result_xml_path ${output_path}/${test_name}.xml)
    set(cmd
        "${NOSETESTS} -P --process-timeout=${_nose_TIMEOUT} ${tests} --with-xunit --xunit-file=\"${result_xml_path}\" ${covarg}"
    )
    _mrt_run_test(
        ${test_name} ${CMAKE_CURRENT_LIST_DIR} ${result_xml_path}
        ${coverage_arg}
        COMMAND ${cmd} ${cover_dir_arg}
        REDIRECT_STDERR)
    if(ARG_DEPENDS)
        add_dependencies(tests_{PROJECT_NAME} ${ARG_DEPENDS})
    endif()
    # make sure python api is compiled first
    if(${PROJECT_NAME}_PYTHON_API_TARGET)
        add_dependencies(tests_${PROJECT_NAME} ${${PROJECT_NAME}_PYTHON_API_TARGET})
        set_tests_properties(${test_name} PROPERTIES DEPENDS build_tests)
    endif()
endfunction()
