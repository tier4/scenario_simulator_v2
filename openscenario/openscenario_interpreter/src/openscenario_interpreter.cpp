// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#define NDEBUG
#define OPENSCENARIO_INTERPRETER_ALLOW_ATTRIBUTES_TO_BE_BLANK
// #define OPENSCENARIO_INTERPRETER_NO_EXTENSION

#include <openscenario_interpreter/openscenario_interpreter.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>
#include <string>

namespace openscenario_interpreter
{
Interpreter::Interpreter(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("openscenario_interpreter", options)
{
  declare_parameter<decltype(expect)>("expect", expect);
  declare_parameter<decltype(log_path)>("log_path", log_path);
  declare_parameter<decltype(osc_path)>("osc_path", osc_path);
  declare_parameter<decltype(step_time_ms)>("step_time_ms", 2);
}

Interpreter::Result Interpreter::on_configure(const rclcpp_lifecycle::State &)
{
  VERBOSE(">>> Configure");

  std::this_thread::sleep_for(std::chrono::seconds(1));

  get_parameter("expect", expect);
  VERBOSE("  expect: " << expect);

  get_parameter("log_path", log_path);
  log_path = log_path + "/result.junit.xml";
  VERBOSE("  log_path: " << log_path);

  get_parameter("osc_path", osc_path);
  VERBOSE("  osc_path: " << osc_path);

  get_parameter("step_time_ms", step_time_ms);
  VERBOSE("  step_time_ms: " << step_time_ms);

  try {
    VERBOSE("  Loading scenario " << osc_path);
    script.rebind<OpenScenario>(osc_path);
  } catch (const openscenario_interpreter::SyntaxError & error) {
    std::cerr << "\x1b[1;31m" << error.what() << std::endl;
    return Interpreter::Result::FAILURE;
  }

  static constexpr auto real_time_factor = 10.0;
  VERBOSE("  real_time_factor: " << real_time_factor);

  connect(
    shared_from_this(),
    script.as<OpenScenario>().scope.logic_file.string());
  VERBOSE("  connection established");

  initialize(real_time_factor, step_time_ms / 1000.0 * real_time_factor);
  VERBOSE("  simulator initialized");

  VERBOSE("<<< Configure");

  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_activate(const rclcpp_lifecycle::State &)
{
  VERBOSE(">>> Activate");

  timer = create_wall_timer(
    std::chrono::milliseconds(step_time_ms),
    [this]()
    {
      guard([this]()
      {
        if (script) {
          if (!script.as<OpenScenario>().complete()) {
            VERBOSE(">>> Evaluate");
            const auto result {
              script.as<OpenScenario>().evaluate()
            };
            VERBOSE("<<< Evaluate");
            VERBOSE("[Storyboard: " << boost::lexical_cast<std::string>(result) << "]");
            #ifndef NDEBUG
            RCLCPP_INFO(
              get_logger(),
              "[%d standby (=> %d) => %d running (=> %d) => %d complete]\n",
              openscenario_interpreter::standby_state.use_count() - 1,
              openscenario_interpreter::start_transition.use_count() - 1,
              openscenario_interpreter::running_state.use_count() - 1,
              openscenario_interpreter::stop_transition.use_count() - 1,
              openscenario_interpreter::complete_state.use_count() - 1);
            #endif
          } else {
            if (expect == "success") {
              report(SUCCESS, "intended-success");
            } else {
              report(FAILURE, "unintended-success", "expected " + expect);
            }
          }
        }
      });
    });

  VERBOSE("<<< Activate");

  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_deactivate(const rclcpp_lifecycle::State &)
{
  VERBOSE(">>> Deactivate");
  timer.reset();
  VERBOSE("<<< Deactivate");
  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_cleanup(const rclcpp_lifecycle::State &)
{
  VERBOSE(">>> Cleanup");
  connection.~API();
  VERBOSE("<<< Cleanup");
  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_shutdown(const rclcpp_lifecycle::State &)
{
  VERBOSE(">>> Shutdown");
  timer.reset();
  VERBOSE("<<< Shutdown");
  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_error(const rclcpp_lifecycle::State &)
{
  VERBOSE(">>> Error");
  timer.reset();
  VERBOSE("<<< Error");
  return Interpreter::Result::SUCCESS;
}
}  // namespace openscenario_interpreter

RCLCPP_COMPONENTS_REGISTER_NODE(openscenario_interpreter::Interpreter)
