// Copyright 2015 TIER IV, Inc. All rights reserved.
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
#include <chrono>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <memory>
#include <openscenario_interpreter/console/escape_sequence.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/custom_command_action.hpp>
#include <openscenario_interpreter/syntax/open_scenario.hpp>
#include <openscenario_interpreter/syntax/scenario_definition.hpp>
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

// NOTE: Error on simulation is not error of the interpreter; so we print error messages into
// INFO_STREAM.
#define INTERPRETER_ERROR_STREAM(...) \
  RCLCPP_INFO_STREAM(get_logger(), "\x1b[1;31m" << __VA_ARGS__ << "\x1b[0m")

namespace openscenario_interpreter
{
class Interpreter : public rclcpp_lifecycle::LifecycleNode,
                    private SimulatorCore::ConditionEvaluation,
                    private SimulatorCore::NonStandardOperation
{
  using Context = openscenario_interpreter_msgs::msg::Context;

  const rclcpp_lifecycle::LifecyclePublisher<Context>::SharedPtr publisher_of_context;

  double local_frame_rate;

  double local_real_time_factor;

  String osc_path;

  String output_directory;

  bool record;

  std::shared_ptr<OpenScenario> script;

  std::list<std::shared_ptr<ScenarioDefinition>> scenarios;

  std::shared_ptr<rclcpp::TimerBase> timer;

  common::JUnit5 results;

  boost::variant<common::junit::Pass, common::junit::Failure, common::junit::Error> result;

  ExecutionTimer<> execution_timer;

  using Result = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  bool waiting_for_engagement_to_be_completed = false;  // NOTE: DIRTY HACK!!!

public:
  OPENSCENARIO_INTERPRETER_PUBLIC
  explicit Interpreter(const rclcpp::NodeOptions &);

  ~Interpreter() override;

  auto currentLocalFrameRate() const -> std::chrono::milliseconds;

  auto currentScenarioDefinition() const -> const std::shared_ptr<ScenarioDefinition> &;

  auto engage() const -> void;

  auto engageable() const -> bool;

  auto engaged() const -> bool;

  auto makeCurrentConfiguration() const -> traffic_simulator::Configuration;

  auto on_activate(const rclcpp_lifecycle::State &) -> Result override;

  auto on_cleanup(const rclcpp_lifecycle::State &) -> Result override;

  auto on_configure(const rclcpp_lifecycle::State &) -> Result override;

  auto on_deactivate(const rclcpp_lifecycle::State &) -> Result override;

  auto on_error(const rclcpp_lifecycle::State &) -> Result override;

  auto on_shutdown(const rclcpp_lifecycle::State &) -> Result override;

  auto publishCurrentContext() const -> void;

  auto reset() -> void;

  template <typename T, typename... Ts>
  auto set(Ts &&... xs) -> void
  {
    result = T(std::forward<decltype(xs)>(xs)...);

    results.name = boost::filesystem::path(osc_path).parent_path().parent_path().string();

    const auto suite_name = boost::filesystem::path(osc_path).parent_path().filename().string();

    const auto case_name = boost::filesystem::path(osc_path).stem().string();

    boost::apply_visitor(
      overload(
        [&](const common::junit::Pass & it) {
          results.testsuite(suite_name).testcase(case_name).pass.push_back(it);
        },
        [&](const common::junit::Failure & it) {
          results.testsuite(suite_name).testcase(case_name).failure.push_back(it);
        },
        [&](const common::junit::Error & it) {
          results.testsuite(suite_name).testcase(case_name).error.push_back(it);
        }),
      result);

    results.write_to(
      (boost::filesystem::path(output_directory) / "result.junit.xml").c_str(), "  ");
  }

  template <typename ExceptionHandler, typename Thunk>
  auto withExceptionHandler(ExceptionHandler && handle, Thunk && thunk) -> decltype(auto)
  {
    try {
      return thunk();
    }

    catch (const SpecialAction<EXIT_SUCCESS> & action)  // from CustomCommandAction::exitSuccess
    {
      set<common::junit::Pass>();
      return handle(action);
    }

    catch (const SpecialAction<EXIT_FAILURE> & action)  // from CustomCommandAction::exitFailure
    {
      set<common::junit::Failure>("Simulation failure", "Expected success");
      return handle(action);
    }

    catch (const AutowareError & error) {
      set<common::junit::Error>("AutowareError", error.what());
      return handle(error);
    }

    catch (const SemanticError & error) {
      set<common::junit::Error>("SemanticError", error.what());
      return handle(error);
    }

    catch (const SimulationError & error) {
      set<common::junit::Error>("SimulationError", error.what());
      return handle(error);
    }

    catch (const SyntaxError & error) {
      set<common::junit::Error>("SyntaxError", error.what());
      return handle(error);
    }

    catch (const std::exception & error)  // NOTE: MUST BE LAST OF CATCH STATEMENTS.
    {
      set<common::junit::Error>("InternalError", error.what());
      return handle(error);
    }

    catch (...)  // FINAL BARRIER
    {
      set<common::junit::Error>("UnknownError", "An unknown exception has occurred");
      return handle();
    }
  }

  template <typename TimeoutHandler, typename Thunk>
  auto withTimeoutHandler(TimeoutHandler && handle, Thunk && thunk) -> decltype(auto)
  {
    if (const auto time = execution_timer.invoke("", thunk); currentLocalFrameRate() < time) {
      handle(execution_timer.getStatistics(""));
    }
  }

  auto defaultTimeoutHandler() const
  {
    /*
       Ideally, the scenario should be terminated with an error if the total
       time for the ScenarioDefinition evaluation and the traffic_simulator's
       updateFrame exceeds the time allowed for a single frame. However, we
       have found that many users are in environments where it is not possible
       to run the simulator stably at 30 FPS (the default setting) while
       running Autoware. In order to prioritize comfortable daily use, we
       decided to give up full reproducibility of the scenario and only provide
       warnings.
    */

    return [this](const auto & statistics) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Your machine is not powerful enough to run the scenario at the specified frame rate ("
          << local_frame_rate << " Hz). We recommend that you reduce the frame rate to "
          << 1000.0 / statistics.template max<std::chrono::milliseconds>().count() << " or less.");
    };
  }
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__OPENSCENARIO_INTERPRETER_HPP_
