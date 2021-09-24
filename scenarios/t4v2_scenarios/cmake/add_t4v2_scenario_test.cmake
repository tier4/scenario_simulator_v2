# Copyright 2021 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

find_package(ament_cmake_test REQUIRED)

option(WITH_INTEGRATION_TEST "If true, run integration test with colcon test command" OFF)

#
# Add a launch test
#
# :param scenario: name of the scenario you want to run
function(add_t4v2_scenario_test testsuite_name package_name workflow global_frame_rate timeout)
  if(WITH_INTEGRATION_TEST)
    set(cmd
      "ros2"
      "launch"
      "scenario_test_runner"
      "scenario_test_runner.launch.py"
      "workflow:=${workflow}"
      "global_frame_rate:=${global_frame_rate}"
      "output_directory:=${CMAKE_BINARY_DIR}/test_results/${package_name}"
    )

    ament_add_test(
      "${package_name}_${scenario}"
      COMMAND ${cmd}
      OUTPUT_FILE "${CMAKE_BINARY_DIR}/test_results/${package_name}/scenario_test_runner/output.txt"
      RESULT_FILE "${CMAKE_BINARY_DIR}/test_results/${package_name}/scenario_test_runner/result.junit.xml"
      TIMEOUT "${timeout}"
    )
  endif()
endfunction()
