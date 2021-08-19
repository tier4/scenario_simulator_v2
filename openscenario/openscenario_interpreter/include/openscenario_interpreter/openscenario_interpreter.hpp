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

#include <boost/variant.hpp>
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
#include <utility>

#define INTERPRETER_INFO_STREAM(...) \
  RCLCPP_INFO_STREAM(get_logger(), "\x1b[32m" << __VA_ARGS__ << "\x1b[0m")

// NOTE: Error on simulation is not error of the interpreter; so we print error messages into INFO_STREAM.
#define INTERPRETER_ERROR_STREAM(...) \
  RCLCPP_INFO_STREAM(get_logger(), "\x1b[1;31m" << __VA_ARGS__ << "\x1b[0m")

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

  common::JUnit5 results;

  boost::variant<common::junit::Pass, common::junit::Failure, common::junit::Error> result;

  ExecutionTimer<> execution_timer;

  template <typename T, typename... Ts>
  auto set(Ts &&... xs) -> void
  {
    result = T(std::forward<decltype(xs)>(xs)...);
  }

  template <typename ExceptionHandler, typename Thunk>
  auto withExceptionHandler(ExceptionHandler && handle, Thunk && thunk) -> decltype(auto)
  {
    try {
      return thunk();
    }

    catch (const SpecialAction<EXIT_SUCCESS> & action)  // from CustomCommandAction::exitSuccess
    {
      if (intended_result == "success") {
        set<common::Pass>();
      } else {
        set<common::Failure>("UnintendedSuccess", "Expected " + intended_result);
      }

      return handle(action);
    }

    catch (const SpecialAction<EXIT_FAILURE> & action)  // from CustomCommandAction::exitFailure
    {
      if (intended_result == "failure") {
        set<common::Pass>();
      } else {
        set<common::Failure>("Failure", "Expected " + intended_result);
      }

      return handle(action);
    }

    catch (const AutowareError & error) {
      isAnErrorIntended() ? set<common::junit::Pass>()
                          : set<common::junit::Error>("AutowareError", error.what());
      return handle(error);
    }

    catch (const SemanticError & error) {
      isAnErrorIntended() ? set<common::junit::Pass>()
                          : set<common::junit::Error>("SemanticError", error.what());
      return handle(error);
    }

    catch (const SimulationError & error) {
      isAnErrorIntended() ? set<common::junit::Pass>()
                          : set<common::junit::Error>("SimulationError", error.what());
      return handle(error);
    }

    catch (const SyntaxError & error) {
      isAnErrorIntended() ? set<common::junit::Pass>()
                          : set<common::junit::Error>("SyntaxError", error.what());
      return handle(error);
    }

    catch (const std::runtime_error & error)  // NOTE: MUST BE LAST OF CATCH STATEMENTS.
    {
      isAnErrorIntended() ? set<common::junit::Pass>()
                          : set<common::junit::Error>("InternalError", error.what());
      return handle(error);
    }

    catch (...)  // FINAL BARRIER
    {
      set<common::junit::Error>("UnknownError", "An unknown exception has occurred");
      return handle();
    }
  }

  auto isAnErrorIntended() const -> bool;

  auto publishCurrentContext() const -> void;

public:
  OPENSCENARIO_INTERPRETER_PUBLIC
  explicit Interpreter(const rclcpp::NodeOptions &);

  using Result = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  auto on_configure(const rclcpp_lifecycle::State &) -> Result override;

  auto on_activate(const rclcpp_lifecycle::State &) -> Result override;

  auto on_deactivate(const rclcpp_lifecycle::State &) -> Result override;

  auto on_cleanup(const rclcpp_lifecycle::State &) -> Result override;

  auto on_shutdown(const rclcpp_lifecycle::State &) -> Result override;

  auto on_error(const rclcpp_lifecycle::State &) -> Result override;
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__OPENSCENARIO_INTERPRETER_HPP_
