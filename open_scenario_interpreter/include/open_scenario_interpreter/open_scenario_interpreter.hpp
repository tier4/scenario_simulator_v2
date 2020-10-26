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

#ifndef OPEN_SCENARIO_INTERPRETER__OPEN_SCENARIO_INTERPRETER_HPP_
#define OPEN_SCENARIO_INTERPRETER__OPEN_SCENARIO_INTERPRETER_HPP_

#include <junit_exporter/junit_exporter.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <open_scenario_interpreter/syntax/open_scenario.hpp>
#include <open_scenario_interpreter/utility/visibility.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

namespace open_scenario_interpreter
{
class ScenarioRunner
  : public rclcpp_lifecycle::LifecycleNode
{
  std::string expect;

  std::string osc_path;

  std::string map_path;

  std::string log_path;

  Element evaluate;

  std::shared_ptr<rclcpp::TimerBase> timer;

  junit_exporter::JunitExporter exporter;

  int step_time_ms;

  decltype(auto) report(
    junit_exporter::TestResult result,
    const std::string & type = "",
    const std::string & what = "")
  {
    exporter.addTestCase(
      evaluate.as<OpenScenario>().scope.scenario.string(),  // XXX DIRTY HACK!!!
      "scenario_testing", 0, result, type, what);

    #ifndef NDEBUG
    switch (result) {
      case junit_exporter::TestResult::ERROR:
        RCLCPP_ERROR(get_logger(), "\x1b[1;31mERROR: %s: %s", type.c_str(), what.c_str());
        break;

      case junit_exporter::TestResult::FAILURE:
        RCLCPP_INFO(get_logger(), "\x1b[1;31mFAILURE\x1b[0m");
        break;

      case junit_exporter::TestResult::SUCCESS:
        RCLCPP_INFO(get_logger(), "\x1b[1;32mSUCCESS\x1b[0m");
        break;
    }
    std::cout << std::flush;
    #endif

    exporter.write(log_path);
  }

public:
  OPEN_SCENARIO_INTERPRETER_PUBLIC
  explicit ScenarioRunner(const rclcpp::NodeOptions &);

  using Result = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  Result on_configure(const rclcpp_lifecycle::State &) override;

  Result on_activate(const rclcpp_lifecycle::State &) override;

  Result on_deactivate(const rclcpp_lifecycle::State &) override;

  Result on_cleanup(const rclcpp_lifecycle::State &) override;

  Result on_shutdown(const rclcpp_lifecycle::State &) override;

  Result on_error(const rclcpp_lifecycle::State &) override;
};
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__OPEN_SCENARIO_INTERPRETER_HPP_
