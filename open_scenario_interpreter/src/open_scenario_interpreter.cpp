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
: rclcpp_lifecycle::LifecycleNode("open_scenario_interpreter", options)
{
  declare_parameter<decltype(expect)>("expect", expect);
  declare_parameter<decltype(log_path)>("log_path", log_path);
  declare_parameter<decltype(osc_path)>("map_path", map_path);
  declare_parameter<decltype(osc_path)>("osc_path", osc_path);
}

ScenarioRunner::Result ScenarioRunner::on_configure(const rclcpp_lifecycle::State &)
{
  using open_scenario_interpreter::ScenarioRunner;

  std::this_thread::sleep_for(std::chrono::seconds(1));

  get_parameter("expect", expect);
  get_parameter("log_path", log_path);
  get_parameter("map_path", map_path);
  get_parameter("osc_path", osc_path);

  log_path = log_path + "/result.junit.xml";

  try {
    RCLCPP_INFO(get_logger(), "Loading scenario \"%s\"", osc_path.c_str());
    evaluate.rebind<OpenScenario>(osc_path);
  } catch (const open_scenario_interpreter::SyntaxError & error) {
    RCLCPP_ERROR(get_logger(), "\x1b[1;31m%s.\x1b[0m", error.what());
    return ScenarioRunner::Result::FAILURE;
  }

  Accessor::connect(shared_from_this(), map_path);
  Accessor::initialize(1.0, 0.02);

  evaluate.as<OpenScenario>().init();

  return ScenarioRunner::Result::SUCCESS;
}

ScenarioRunner::Result ScenarioRunner::on_activate(const rclcpp_lifecycle::State &)
{
  timer = create_wall_timer(
    std::chrono::milliseconds(50),
    [this]()
    {
      try {
        if (evaluate && !evaluate.as<OpenScenario>().complete()) {
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
        } else if (evaluate) {
          if (expect == "success") {
            std::string testcase = evaluate.as<OpenScenario>().scope.scenario.string();
            exporter.addTestCase(testcase, "scenario_testing", 0,
            junit_exporter::TestResult::SUCCESS);
            exporter.write(log_path);
          } else {
            RCLCPP_ERROR(get_logger(), "\x1b[1;32mFailure.\x1b[0m");
            std::string testcase = evaluate.as<OpenScenario>().scope.scenario.string();
            exporter.addTestCase(testcase, "scenario_testing", 0,
            junit_exporter::TestResult::FAILURE, "unexpected result",
            "testcase : " + testcase + " should end with " + expect);
            exporter.write(log_path);
          }
          evaluate.reset();
          deactivate();
        }
      } catch (const int command) {
        switch (command) {
          case EXIT_SUCCESS:
            if (expect == "success") {
              std::string testcase = evaluate.as<OpenScenario>().scope.scenario.string();
              exporter.addTestCase(testcase, "scenario_testing", 0,
              junit_exporter::TestResult::SUCCESS);
              exporter.write(log_path);
            } else {
              std::string testcase = evaluate.as<OpenScenario>().scope.scenario.string();
              exporter.addTestCase(testcase, "scenario_testing", 0,
              junit_exporter::TestResult::FAILURE, "unexpected result",
              "testcase : " + testcase + " should end with " + expect);
              exporter.write(log_path);
            }
            RCLCPP_INFO(get_logger(), "\x1b[1;32mSimulation succeeded.\x1b[0m");
            evaluate.reset();
            deactivate();
            break;

          case EXIT_FAILURE:
            if (expect == "failure") {
              std::string testcase = evaluate.as<OpenScenario>().scope.scenario.string();
              exporter.addTestCase(testcase, "scenario_testing", 0,
              junit_exporter::TestResult::SUCCESS);
              exporter.write(log_path);
            } else {
              std::string testcase = evaluate.as<OpenScenario>().scope.scenario.string();
              exporter.addTestCase(testcase, "scenario_testing", 0,
              junit_exporter::TestResult::FAILURE, "unexpected result",
              "testcase : " + testcase + " should end with " + expect);
              exporter.write(log_path);
            }
            RCLCPP_INFO(get_logger(), "\x1b[1;31mSimulation failed.\x1b[0m");
            evaluate.reset();
            deactivate();

          default:
            break;
        }
      } catch (const open_scenario_interpreter::ImplementationFault & error) {
        std::string testcase = evaluate.as<OpenScenario>().scope.scenario.string();
        if (expect == "error") {
          exporter.addTestCase(testcase, "scenario_testing", 0,
          junit_exporter::TestResult::SUCCESS);
          exporter.write(log_path);
        } else {
          std::string type = "scenario_runner::ImplementationFault";
          exporter.addTestCase(
            testcase, "scenario_testing", 0, junit_exporter::TestResult::ERROR, type, error.what());
          exporter.write(log_path);
        }
        RCLCPP_ERROR(get_logger(), "%s.", error.what());
        evaluate.reset();
        deactivate();
      } catch (const std::exception & error) {
        std::string testcase = evaluate.as<OpenScenario>().scope.scenario.string();
        if (expect == "error") {
          exporter.addTestCase(testcase, "scenario_testing", 0,
          junit_exporter::TestResult::SUCCESS);
          exporter.write(log_path);
        } else {
          std::string type = "std::exception";
          exporter.addTestCase(testcase, "scenario_testing", 0,
          junit_exporter::TestResult::ERROR, type, error.what());
          exporter.write(log_path);
        }
        RCLCPP_ERROR(get_logger(), "%s.", error.what());
        evaluate.reset();
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
