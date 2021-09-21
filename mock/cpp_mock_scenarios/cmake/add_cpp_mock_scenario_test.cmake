find_package(ament_cmake_test REQUIRED)

#
# Add a launch test
#
# :param scenario: name of the scenario you want to run
function(add_cpp_mock_scenario_test package_name scenario timeout)
  set(cmd
    "ros2"
    "launch"
    "package:=${package_name}"
    "scenario:=${scenario}"
    "timeout:=${timeout}"
  )

  ament_add_test(
    "${package_name}_${scenario}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/launch_test/${package_name}_${scenario}.txt"
    RESULT_FILE "${CMAKE_BINARY_DIR}/launch_test/${package_name}_${scenario}.xunit.xmml"
    TIMEOUT "${timeout}"
  )
endfunction()
