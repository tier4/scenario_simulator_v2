// Copyright 2025 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// R2 spike (SSV2_HEADLESS_EGO only): drive the OpenSCENARIO Interpreter's lifecycle manually,
// WITHOUT spinning an executor, to prove the headless pybind driving model:
//   make_shared<Interpreter>(NodeOptions overrides) -> configure() -> activate()
//     -> step() loop (evaluateFrame + exception handling + junit) -> deactivate().
// Success criteria: configure reaches inactive, activate reaches active (this is also the ODR
// runtime canary when the scenario has an Autoware ego — activate engages it via
// getEgoEntity(name) across the TU boundary), stepping advances the scenario to a terminal
// outcome, and result.junit.xml is produced. Takes the scenario path as argv[1].

#include <cstdio>
#include <lifecycle_msgs/msg/state.hpp>
#include <memory>
#include <openscenario_interpreter/openscenario_interpreter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

using lifecycle_msgs::msg::State;
using openscenario_interpreter::Interpreter;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <osc_path> [output_dir]\n", argv[0]);
    return 2;
  }
  const std::string osc_path = argv[1];
  const std::string output_dir = argc > 2 ? argv[2] : std::string("/tmp/headless_interp_check");

  rclcpp::NodeOptions options;
  options.append_parameter_override("osc_path", osc_path);
  options.append_parameter_override("local_frame_rate", 10.0);
  options.append_parameter_override("local_real_time_factor", 1.0);
  options.append_parameter_override("output_directory", output_dir);
  options.append_parameter_override("headless", true);
  options.append_parameter_override("consider_pose_by_road_slope", false);

  auto interpreter = std::make_shared<Interpreter>(options);

  interpreter->configure();
  std::printf("[configure] state=%s\n", interpreter->get_current_state().label().c_str());
  if (interpreter->get_current_state().id() != State::PRIMARY_STATE_INACTIVE) {
    std::printf("RESULT: FAIL (configure did not reach inactive; needs an executor spin?)\n");
    rclcpp::shutdown();
    return 1;
  }

  interpreter->activate();
  std::printf("[activate] state=%s\n", interpreter->get_current_state().label().c_str());
  if (interpreter->get_current_state().id() != State::PRIMARY_STATE_ACTIVE) {
    std::printf("RESULT: FAIL (activate did not reach active)\n");
    rclcpp::shutdown();
    return 1;
  }

  int steps = 0;
  constexpr int max_steps = 400;
  auto outcome = Interpreter::StepOutcome::running;
  while (outcome == Interpreter::StepOutcome::running && steps < max_steps) {
    outcome = interpreter->step();
    ++steps;
  }
  const bool terminated = outcome == Interpreter::StepOutcome::terminated;
  std::printf(
    "[step] frames=%d outcome=%s resultKind=%s\n", steps, terminated ? "terminated" : "max_steps",
    interpreter->resultKind().c_str());

  if (interpreter->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    interpreter->deactivate();
  }
  std::printf("[deactivate] state=%s\n", interpreter->get_current_state().label().c_str());

  std::printf(
    "RESULT: %s (lifecycle driven without executor spin over %d frames)\n",
    terminated ? "PASS" : "INCONCLUSIVE(hit max_steps)", steps);

  rclcpp::shutdown();
  return terminated ? 0 : 1;
}
