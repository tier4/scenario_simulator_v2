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

#include <open_scenario_interpreter/open_scenario_interpreter.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>
#include <string>

namespace open_scenario_interpreter
{
ScenarioRunner::ScenarioRunner(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("open_scenario_interpreter", options),
  port
  {
    8080
  }
{
  declare_parameter<decltype(scenario)>("scenario", scenario);
}

ScenarioRunner::Result ScenarioRunner::on_configure(const rclcpp_lifecycle::State &)
{
  using open_scenario_interpreter::ScenarioRunner;

  get_parameter("scenario", scenario);

  try {
    RCLCPP_INFO(get_logger(), "Loading scenario \"%s\"", scenario.c_str());
    evaluate.rebind<OpenScenario>(scenario, address, port);
  } catch (const open_scenario_interpreter::SyntaxError & error) {
    RCLCPP_ERROR(get_logger(), "\x1b[1;31m%s.\x1b[0m", error.what());
    return ScenarioRunner::Result::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Connecting simulator via %s:%d", address.c_str(), port);
  evaluate.as<OpenScenario>().init();

  return ScenarioRunner::Result::SUCCESS;
}

ScenarioRunner::Result ScenarioRunner::on_activate(const rclcpp_lifecycle::State &)
{
  timer = create_wall_timer(
    std::chrono::milliseconds(50),
    [&]()
    {
      try {
        if (!evaluate.as<OpenScenario>().complete()) {
          const auto result {evaluate.as<OpenScenario>()()};

          RCLCPP_INFO(
            get_logger(),
            "[Storyboard: %s]",
            boost::lexical_cast<std::string>(result).c_str());

          RCLCPP_INFO(
            get_logger(),
            "[%d standby (=> %d) => %d running (=> %d) => %d complete]\n",
            open_scenario_interpreter::standby_state.use_count() - 1,
            open_scenario_interpreter::start_transition.use_count() - 1,
            open_scenario_interpreter::running_state.use_count() - 1,
            open_scenario_interpreter::stop_transition.use_count() - 1,
            open_scenario_interpreter::complete_state.use_count() - 1);
        } else {
          deactivate();
        }
      } catch (const int command) {
        switch (command) {
          case EXIT_SUCCESS:
            RCLCPP_INFO(get_logger(), "\x1b[1;32mSimulation succeeded.\x1b[0m");
            deactivate();
            break;

          case EXIT_FAILURE:
            RCLCPP_INFO(get_logger(), "\x1b[1;31mSimulation failed.\x1b[0m");
            deactivate();

          default:
            break;
        }
      } catch (const open_scenario_interpreter::ImplementationFault & error) {
        RCLCPP_ERROR(get_logger(), "%s.", error.what());
        deactivate();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "%s.", error.what());
        deactivate();
      }
    });

  return ScenarioRunner::Result::SUCCESS;
}

ScenarioRunner::Result ScenarioRunner::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer.reset();
  return ScenarioRunner::Result::SUCCESS;
}

ScenarioRunner::Result ScenarioRunner::on_cleanup(const rclcpp_lifecycle::State &)
{
  evaluate = unspecified;
  return ScenarioRunner::Result::SUCCESS;
}

ScenarioRunner::Result ScenarioRunner::on_shutdown(const rclcpp_lifecycle::State &)
{
  return ScenarioRunner::Result::SUCCESS;
}

ScenarioRunner::Result ScenarioRunner::on_error(const rclcpp_lifecycle::State &)
{
  return ScenarioRunner::Result::SUCCESS;
}
}  // namespace open_scenario_interpreter

RCLCPP_COMPONENTS_REGISTER_NODE(open_scenario_interpreter::ScenarioRunner)
