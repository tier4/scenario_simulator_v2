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

#include <exception>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <memory>
#include <openscenario_interpreter/console/escape_sequence.hpp>
#include <openscenario_interpreter/syntax/openscenario.hpp>
#include <openscenario_interpreter/utility/execution_timer.hpp>
#include <openscenario_interpreter/utility/visibility.hpp>
#include <openscenario_interpreter_msgs/msg/context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <simple_junit/junit5.hpp>
#include <simple_junit/test_suites.hpp>  // DEPRECATED
#include <string>
#include <utility>

namespace openscenario_interpreter
{
class Interpreter : public rclcpp_lifecycle::LifecycleNode
{
  using Context = openscenario_interpreter_msgs::msg::Context;

  const rclcpp_lifecycle::LifecyclePublisher<Context>::SharedPtr publisher_of_context;

  String intended_result;
  double local_frame_rate;
  double local_real_time_factor;
  String osc_path;
  String output_directory;

  Element script;

  std::shared_ptr<rclcpp::TimerBase> timer;

  // [[deprecated]] junit::TestSuites test_suites;

  common::JUnit5 simple_test_suites;

  const junit::TestResult ERROR = junit::TestResult::ERROR;
  const junit::TestResult FAILURE = junit::TestResult::FAILURE;
  const junit::TestResult SUCCESS = junit::TestResult::SUCCESS;

  junit::TestResult current_result;

  std::string current_error_type;
  std::string current_error_what;

  ExecutionTimer<> execution_timer;

  void reset()
  {
    current_result = junit::TestResult::FAILURE;
    current_error_type = "Failure";
    current_error_what =
      "The simulation time has exceeded the time specified by the scenario_test_runner";
  }

  void report(const junit::TestResult &, const std::string &, const std::string & = "");

#define CATCH(TYPE)                       \
  catch (const TYPE & error)              \
  {                                       \
    if (intended_result == "error") {     \
      report(SUCCESS, #TYPE);             \
    } else {                              \
      report(ERROR, #TYPE, error.what()); \
    }                                     \
  }

  template <typename Thunk>
  void guard(Thunk && thunk)
  {
    try {
      return thunk();
    }

    catch (const SpecialAction<EXIT_SUCCESS> &)  // from CustomCommandAction::exitSuccess
    {
      if (intended_result == "success") {
        report(SUCCESS, "Success");
      } else {
        report(FAILURE, "UnintendedSuccess", "Expected " + intended_result);
      }
    }

    catch (const SpecialAction<EXIT_FAILURE> &)  // from CustomCommandAction::exitFailure
    {
      if (intended_result == "failure") {
        report(SUCCESS, "IntendedFailure");
      } else {
        report(FAILURE, "Failure", "Expected " + intended_result);
      }
    }

    CATCH(AutowareError)
    CATCH(SemanticError)
    CATCH(SimulationError)
    CATCH(SyntaxError)
    CATCH(InternalError)  // NOTE: InternalError MUST BE LAST OF CATCH STATEMENTS.

    catch (...)  // FINAL BARRIER
    {
      report(ERROR, "UnknownError", "An unknown exception has occurred");
    }
  }

#undef CATCH

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
