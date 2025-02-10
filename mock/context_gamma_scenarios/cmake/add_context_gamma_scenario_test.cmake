# Copyright 2021 TIER IV, Inc.
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
#
# Add a launch test
#
# :param scenario: name of the scenario you want to run
function(add_context_gamma_scenario_test package_name scenario timeout)
  set(cmd
    "ros2"
    "launch"
    "context_gamma_scenarios"
    "mock_test.launch.py"
    "package:=${package_name}"
    "scenario:=${scenario}"
    "timeout:=${timeout}"
    "junit_path:=${CMAKE_BINARY_DIR}/test_results/${package_name}/${package_name}_${scenario}.xunit.xml"
  )

  ament_add_test(
    "${package_name}_${scenario}"
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/test_results/${package_name}/${package_name}_${scenario}.output.txt"
    RESULT_FILE "${CMAKE_BINARY_DIR}/test_results/${package_name}/${package_name}_${scenario}.xunit.xml"
    TIMEOUT "30"
  )
endfunction()
