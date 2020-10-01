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

#ifndef SCENARIO_RUNNER__SCENARIO_RUNNER_HPP_
#define SCENARIO_RUNNER__SCENARIO_RUNNER_HPP_

#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <scenario_runner/syntax/open_scenario.hpp>

#include <memory>
#include <string>

#define READ_PARAMETER(NAME) \
  declare_parameter<decltype(NAME)>(#NAME, NAME); \
  get_parameter(#NAME, NAME)

namespace scenario_runner
{
class ScenarioRunner
  : public rclcpp_lifecycle::LifecycleNode
{
  int port;

  std::string scenario;

  std::string address {"127.0.0.1"};

  Element evaluate;

  std::shared_ptr<rclcpp::TimerBase> timer;

public:
  explicit ScenarioRunner(const std::string & name = "scenario_runner")
  : rclcpp_lifecycle::LifecycleNode(
      name,
      rclcpp::NodeOptions().use_intra_process_comms(false)),
    port{8080},
    scenario{"$(scenario_runner)/example/dummy.xosc"}
  {
    READ_PARAMETER(port);
    READ_PARAMETER(scenario);
  }

  using Result = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  Result on_configure(const rclcpp_lifecycle::State &) override
  {
    // set rosparam map path

    using scenario_runner::ScenarioRunner;

    try {
      RCLCPP_INFO(get_logger(), "Loading scenario \"%s\"", scenario.c_str());
      evaluate.rebind<OpenScenario>(scenario, address, port);
    } catch (const scenario_runner::SyntaxError & error) {
      RCLCPP_ERROR(get_logger(), "\x1b[1;31m%s.\x1b[0m", error.what());
      return Result::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "Connecting simulator via %s:%d", address.c_str(), port);
    evaluate.as<OpenScenario>().init();

    return Result::SUCCESS;
  }

  Result on_activate(const rclcpp_lifecycle::State &) override
  {
    using std::literals::chrono_literals::operator"" ms;

    timer = create_wall_timer(
      50ms,
      [&]()
      {
        try {
          if (!evaluate.as<OpenScenario>().complete()) {
            std::cout << "[Storyboard: " << evaluate.as<OpenScenario>()() << "]" << std::endl;

            RCLCPP_INFO(
              get_logger(),
              "[%d standby (=> %d) => %d running (=> %d) => %d complete]",
              scenario_runner::standby_state.use_count() - 1,
              scenario_runner::start_transition.use_count() - 1,
              scenario_runner::running_state.use_count() - 1,
              scenario_runner::stop_transition.use_count() - 1,
              scenario_runner::complete_state.use_count() - 1);
          } else {
            deactivate();
          }
        } catch (const scenario_runner::Command & command) {
          switch (command) {
            case scenario_runner::Command::exitSuccess:
              RCLCPP_INFO(get_logger(), "Simulation succeeded.");
              deactivate();
              break;

            default:
            case scenario_runner::Command::exitFailure:
              RCLCPP_INFO(get_logger(), "Simulation failed.");
              deactivate();
          }
        } catch (const scenario_runner::ImplementationFault & error) {
          RCLCPP_ERROR(get_logger(), "%s.", error.what());
          throw;
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "%s.", error.what());
          throw;
        }
      }
    );

    return Result::SUCCESS;
  }

  Result on_deactivate(const rclcpp_lifecycle::State &) override
  {
    timer.reset();
    return Result::SUCCESS;
  }

  Result on_cleanup(const rclcpp_lifecycle::State &) override
  {
    evaluate = unspecified;
    return Result::SUCCESS;
  }

  Result on_shutdown(const rclcpp_lifecycle::State &) override
  {
    return Result::SUCCESS;
  }
};
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SCENARIO_RUNNER_HPP_
