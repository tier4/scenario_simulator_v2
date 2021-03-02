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

#ifndef OPENSCENARIO_INTERPRETER__OPENSCENARIO_INTERPRETER_HPP_
#define OPENSCENARIO_INTERPRETER__OPENSCENARIO_INTERPRETER_HPP_

#include <junit_exporter/junit_exporter.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <openscenario_interpreter/syntax/openscenario.hpp>
#include <openscenario_interpreter/utility/verbose.hpp>
#include <openscenario_interpreter/utility/visibility.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

namespace openscenario_interpreter
{
class Interpreter
  : public rclcpp_lifecycle::LifecycleNode
{
  std::string expect;

  std::string output_directory;

  std::string osc_path;

  double real_time_factor;

  double frame_rate;

  Element script;

  std::shared_ptr<rclcpp::TimerBase> timer;

  junit_exporter::JunitExporter exporter;

  const junit_exporter::TestResult ERROR = junit_exporter::TestResult::ERROR;
  const junit_exporter::TestResult FAILURE = junit_exporter::TestResult::FAILURE;
  const junit_exporter::TestResult SUCCESS = junit_exporter::TestResult::SUCCESS;

  decltype(auto) report(
    const junit_exporter::TestResult & result,
    const std::string & type,
    const std::string & what = "")
  {
    VERBOSE("  appending current test case result");
    exporter.addTestCase(
      script.as<OpenScenario>().scope.scenario.string(),  // XXX DIRTY HACK!!!
      "scenario_testing", 0, result, type, what);

    switch (result) {
      case junit_exporter::TestResult::ERROR:
      case junit_exporter::TestResult::FAILURE:
        if (what.empty()) {
          std::cout << "\x1b[1;31mYield " << type.c_str() << "\x1b[0m" << std::endl;
        } else {
          std::cout << "\x1b[1;31mYield " << type.c_str() << " (" << what.c_str() << ")\x1b[0m" <<
            std::endl;
        }
        break;

      case junit_exporter::TestResult::SUCCESS:
        if (what.empty()) {
          std::cout << "\x1b[32mYield " << type.c_str() << "\x1b[0m" << std::endl;
        } else {
          std::cout << "\x1b[32mYield " << type.c_str() << " (" << what.c_str() << ")\x1b[0m" <<
            std::endl;
        }
        break;
    }
    VERBOSE("");

    exporter.write(output_directory + "/result.junit.xml");

    script.reset();

    while (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      VERBOSE("  waiting for change current state " << get_current_state().id() << " to active");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    deactivate();
  }

  template<typename Thunk>
  void withExceptionHandler(Thunk && thunk) try
  {
    return thunk();
  } catch (const int command) {
    switch (command) {
      case EXIT_SUCCESS:
        if (expect == "success") {
          report(SUCCESS, "intended-success");
        } else {
          report(FAILURE, "unintended-success", "expected " + expect);
        }
        break;

      case EXIT_FAILURE:
        if (expect == "failure") {
          report(SUCCESS, "intended-failure");
        } else {
          report(FAILURE, "unintended-failure", "expected " + expect);
        }
        break;

      default:
        break;
    }
  } catch (const openscenario_interpreter::SemanticError & error) {
    VERBOSE("  caught semantic-error");
    if (expect == "error") {
      report(SUCCESS, "intended-error");
    } else {
      report(ERROR, "semantic-error", error.what());
    }
  } catch (const openscenario_interpreter::ImplementationFault & error) {
    VERBOSE("  caught implementation-fault");
    if (expect == "error") {
      report(SUCCESS, "intended-error");
    } else {
      report(ERROR, "implementation-fault", error.what());
    }
  } catch (const std::exception & error) {
    VERBOSE(" caught standard exception");
    if (expect == "error") {
      report(SUCCESS, "intended-error");
    } else {
      report(ERROR, "unexpected-standard-exception", error.what());
    }
  } catch (...) {
    VERBOSE(" caught unknown exception");
    report(ERROR, "unexpected-unknown-exception");
  }

public:
  OPENSCENARIO_INTERPRETER_PUBLIC
  explicit Interpreter(const rclcpp::NodeOptions &);

  using Result = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  Result on_configure(const rclcpp_lifecycle::State &) override;

  Result on_activate(const rclcpp_lifecycle::State &) override;

  Result on_deactivate(const rclcpp_lifecycle::State &) override;

  Result on_cleanup(const rclcpp_lifecycle::State &) override;

  Result on_shutdown(const rclcpp_lifecycle::State &) override;

  Result on_error(const rclcpp_lifecycle::State &) override;
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__OPENSCENARIO_INTERPRETER_HPP_
