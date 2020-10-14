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

#include <scenario_runner/scenario_runner.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>
#include <string>

namespace scenario_runner
{
ScenarioRunner::ScenarioRunner(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("scenario_runner", options),
  port
  {
    8080
  }
{
  declare_parameter<decltype(scenario)>("scenario", scenario);
  declare_parameter<decltype(expect)>("expect", expect);
  declare_parameter<decltype(log_path)>("log_path", log_path);
}

ScenarioRunner::Result ScenarioRunner::on_configure(const rclcpp_lifecycle::State &)
{
  using scenario_runner::ScenarioRunner;

  get_parameter("scenario", scenario);
  get_parameter("expect", expect);
  get_parameter("log_path", log_path);
  log_path = log_path + "/result.junit.xml";

  try {
    RCLCPP_INFO(get_logger(), "Loading scenario \"%s\"", scenario.c_str());
    evaluate.rebind<OpenScenario>(scenario, address, port);
  } catch (const scenario_runner::SyntaxError & error) {
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
            scenario_runner::standby_state.use_count() - 1,
            scenario_runner::start_transition.use_count() - 1,
            scenario_runner::running_state.use_count() - 1,
            scenario_runner::stop_transition.use_count() - 1,
            scenario_runner::complete_state.use_count() - 1);
        } else {
          if(expect == "success")
          {
            std::string testcase = evaluate.as<OpenScenario>().global.scenario.string();
            exporter.addTestCase(testcase, testcase, 0,
              junit_exporter::TestResult::SUCCESS);
            exporter.write(log_path);
          }
          else
          {
            RCLCPP_ERROR(get_logger(), "\x1b[1;32mFailure.\x1b[0m");
            std::string testcase = evaluate.as<OpenScenario>().global.scenario.string();
            exporter.addTestCase(testcase, testcase, 0,
              junit_exporter::TestResult::FAILURE, "unexpected result",
              "testcase : " + testcase + " should end with "  + expect);
            exporter.write(log_path);
          }
          deactivate();
        }
      } catch (const int command) {
        switch (command) {
          case EXIT_SUCCESS:
            if(expect == "success")
           {
              std::string testcase = evaluate.as<OpenScenario>().global.scenario.string();
              exporter.addTestCase(testcase, testcase, 0,
                junit_exporter::TestResult::SUCCESS);
              exporter.write(log_path);
            }
            else
            {
              std::string testcase = evaluate.as<OpenScenario>().global.scenario.string();
              exporter.addTestCase(testcase, testcase, 0,
                junit_exporter::TestResult::FAILURE, "unexpected result",
                "testcase : " + testcase + " should end with "  + expect);
              exporter.write(log_path);
            }
            RCLCPP_INFO(get_logger(), "\x1b[1;32mSimulation succeeded.\x1b[0m");
            deactivate();
            break;

          case EXIT_FAILURE:
            if(expect == "failure")
            {
              std::string testcase = evaluate.as<OpenScenario>().global.scenario.string();
              exporter.addTestCase(testcase, testcase, 0,
                junit_exporter::TestResult::SUCCESS);
              exporter.write(log_path);
            }
            else
            {
              std::string testcase = evaluate.as<OpenScenario>().global.scenario.string();
              exporter.addTestCase(testcase, testcase, 0,
                junit_exporter::TestResult::FAILURE, "unexpected result",
                "testcase : " + testcase + " should end with " + expect);
              exporter.write(log_path);
            }
            RCLCPP_INFO(get_logger(), "\x1b[1;31mSimulation failed.\x1b[0m");
            deactivate();

          default:
            break;
        }
      } catch (const scenario_runner::ImplementationFault & error) {
        std::string testcase = evaluate.as<OpenScenario>().global.scenario.string();
        if(expect == "error"){
          exporter.addTestCase(testcase, testcase, 0,
            junit_exporter::TestResult::SUCCESS);
          exporter.write(log_path);
        }
        else{
          std::string type = "scenario_runner::ImplementationFault";
          exporter.addTestCase(testcase, testcase, 0,
            junit_exporter::TestResult::ERROR, type, error.what());
          exporter.write(log_path);
        }
        RCLCPP_ERROR(get_logger(), "%s.", error.what());
        deactivate();
      } catch (const std::exception & error) {
        std::string testcase = evaluate.as<OpenScenario>().global.scenario.string();
        if(expect == "error"){
          exporter.addTestCase(testcase, testcase, 0,
            junit_exporter::TestResult::SUCCESS);
          exporter.write(log_path);
        }
        else{
          std::string type = "std::exception";
          exporter.addTestCase(testcase, testcase, 0,
            junit_exporter::TestResult::ERROR, type, error.what());
          exporter.write(log_path);
        }
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
}  // namespace scenario_runner

RCLCPP_COMPONENTS_REGISTER_NODE(scenario_runner::ScenarioRunner)
