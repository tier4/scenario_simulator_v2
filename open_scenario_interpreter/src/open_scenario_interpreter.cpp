// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

// #define NDEBUG
#undef NDEBUG
#include <open_scenario_interpreter/open_scenario_interpreter.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>
#include <string>

namespace open_scenario_interpreter
{
Interpreter::Interpreter(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("open_scenario_interpreter", options)
{
  declare_parameter<decltype(expect)>("expect", expect);
  declare_parameter<decltype(log_path)>("log_path", log_path);
  declare_parameter<decltype(map_path)>("map_path", map_path);
  declare_parameter<decltype(osc_path)>("osc_path", osc_path);
  declare_parameter<decltype(step_time_ms)>("step_time_ms", 2);
}

Interpreter::Result Interpreter::on_configure(const rclcpp_lifecycle::State &)
{
  std::this_thread::sleep_for(std::chrono::seconds(1));

  get_parameter("expect", expect);
  get_parameter("log_path", log_path);
  get_parameter("map_path", map_path);
  get_parameter("osc_path", osc_path);
  get_parameter("step_time_ms", step_time_ms);

  log_path = log_path + "/result.junit.xml";

  try {
    #ifndef NDEBUG
    RCLCPP_INFO(get_logger(), "Loading scenario \"%s\"", osc_path.c_str());
    #endif
    script.rebind<OpenScenario>(osc_path);
  } catch (const open_scenario_interpreter::SyntaxError & error) {
    #ifndef NDEBUG
    RCLCPP_ERROR(get_logger(), "\x1b[1;31m%s.\x1b[0m", error.what());
    #endif
    return Interpreter::Result::FAILURE;
  }

  static constexpr auto real_time_factor = 3.0;

  connect(
    shared_from_this(),
    script.as<OpenScenario>().scope.logic_file.string());

  initialize(real_time_factor, step_time_ms / 1000.0 * real_time_factor);

  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_activate(const rclcpp_lifecycle::State &)
{
  timer = create_wall_timer(
    std::chrono::milliseconds(step_time_ms),
    [this]()
    {
      guard([this]()
      {
        if (script) {
          if (!script.as<OpenScenario>().complete()) {
            const auto result {
              script.as<OpenScenario>().evaluate()
            };

            #ifndef NDEBUG
            RCLCPP_INFO(
              get_logger(), "[Storyboard: %s]", boost::lexical_cast<std::string>(result).c_str());
            #endif

            #ifndef NDEBUG
            RCLCPP_INFO(
              get_logger(),
              "[%d standby (=> %d) => %d running (=> %d) => %d complete]\n",
              open_scenario_interpreter::standby_state.use_count() - 1,
              open_scenario_interpreter::start_transition.use_count() - 1,
              open_scenario_interpreter::running_state.use_count() - 1,
              open_scenario_interpreter::stop_transition.use_count() - 1,
              open_scenario_interpreter::complete_state.use_count() - 1);
            #endif
          } else {
            if (expect == "success") {
              report(SUCCESS, "intended-success");
            } else {
              report(FAILURE, "unintended-failure", "expected " + expect);
            }
          }
        }
      });
    });

  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer.reset();
  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_cleanup(const rclcpp_lifecycle::State &)
{
  timer.reset();
  connection.~API();
  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_shutdown(const rclcpp_lifecycle::State &)
{
  timer.reset();
  return Interpreter::Result::SUCCESS;
}

Interpreter::Result Interpreter::on_error(const rclcpp_lifecycle::State &)
{
  timer.reset();
  return Interpreter::Result::SUCCESS;
}
}  // namespace open_scenario_interpreter

RCLCPP_COMPONENTS_REGISTER_NODE(open_scenario_interpreter::Interpreter)
