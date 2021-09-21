#
# Add a launch test
#
# :param scenario: name of the scenario you want to run
function(add_cpp_mock_scenario_test package_name scenario )
  set(cmd
    "ros2"
    "launch"
    "${scenario}"
    "--junit-xml=${_launch_test_RESULT_FILE}"
    "--package-name=${PROJECT_NAME}"
  )

  ament_add_test(
    "${_launch_test_TARGET}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/launch_test/${_launch_test_TARGET}.txt"
    RESULT_FILE "${_launch_test_RESULT_FILE}"
    TIMEOUT "${_launch_test_TIMEOUT}"
    ${_launch_test_UNPARSED_ARGUMENTS}
  )
endfunction()
