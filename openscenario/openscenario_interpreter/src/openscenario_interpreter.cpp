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

#define OPENSCENARIO_INTERPRETER_NO_EXTENSION

#include <algorithm>
#include <nlohmann/json.hpp>
#include <openscenario_interpreter/openscenario_interpreter.hpp>
#include <openscenario_interpreter/record.hpp>
#include <openscenario_interpreter/syntax/object_controller.hpp>
#include <openscenario_interpreter/syntax/parameter_value_distribution.hpp>
#include <openscenario_interpreter/syntax/scenario_definition.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/utility/overload.hpp>

#define DECLARE_PARAMETER(IDENTIFIER) \
  declare_parameter<decltype(IDENTIFIER)>(#IDENTIFIER, IDENTIFIER)

#define GET_PARAMETER(IDENTIFIER) get_parameter(#IDENTIFIER, IDENTIFIER)

namespace openscenario_interpreter
{
Interpreter::Interpreter(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("openscenario_interpreter", options),
  publisher_of_context(create_publisher<Context>("context", rclcpp::QoS(1).transient_local())),
  local_frame_rate(30),
  local_real_time_factor(1.0),
  osc_path(""),
  output_directory("/tmp"),
  record(false)
{
  DECLARE_PARAMETER(local_frame_rate);
  DECLARE_PARAMETER(local_real_time_factor);
  DECLARE_PARAMETER(osc_path);
  DECLARE_PARAMETER(output_directory);
  DECLARE_PARAMETER(record);
}

Interpreter::~Interpreter() {}

auto Interpreter::currentLocalFrameRate() const -> std::chrono::milliseconds
{
  return std::chrono::milliseconds(static_cast<unsigned int>(1 / local_frame_rate * 1000));
}

auto Interpreter::currentScenarioDefinition() const -> const std::shared_ptr<ScenarioDefinition> &
{
  return scenarios.front();
}

auto Interpreter::makeCurrentConfiguration() const -> traffic_simulator::Configuration
{
  const auto logic_file = currentScenarioDefinition()->road_network.logic_file;

  auto configuration = traffic_simulator::Configuration(
    logic_file.isDirectory() ? logic_file : logic_file.filepath.parent_path());
  {
    configuration.auto_sink = false;
    configuration.scenario_path = osc_path;

    // XXX DIRTY HACK!!!
    if (not logic_file.isDirectory() and logic_file.filepath.extension() == ".osm") {
      configuration.lanelet2_map_file = logic_file.filepath.filename().string();
    }
  }

  return configuration;
}

auto Interpreter::on_configure(const rclcpp_lifecycle::State &) -> Result
{
  return withExceptionHandler(
    [](auto &&...) {
      return Interpreter::Result::FAILURE;  // => Unconfigured
    },
    [this]() {
      /*
         The scenario_test_runner that launched this node considers that "the
         scenario is not expected to finish" or "an abnormality has occurred
         that prevents the interpreter from terminating itself" after the
         specified time (specified by --global-timeout), and deactivates this
         node.
      */
      result = common::junit::Failure(
        "Timeout",
        "The simulation time has exceeded the time specified by the scenario_test_runner.");

      std::this_thread::sleep_for(std::chrono::seconds(1));  // NOTE: Wait for parameters to be set.

      GET_PARAMETER(local_frame_rate);
      GET_PARAMETER(local_real_time_factor);
      GET_PARAMETER(osc_path);
      GET_PARAMETER(output_directory);
      GET_PARAMETER(record);

      script = std::make_shared<OpenScenario>(osc_path);

      if (script->category.is<ScenarioDefinition>()) {
        scenarios = {std::dynamic_pointer_cast<ScenarioDefinition>(script->category)};
      } else if (script->category.is<ParameterValueDistribution>()) {
        throw Error(
          "ParameterValueDistribution cannot be processed by openscenario_interpreter alone. "
          "Give a preprocessed scenario using openscenario_preprocessor together");
      } else {
        throw SyntaxError(
          "Unsupported member of OpenScenarioCategory group is defined in the scenario file");
      }

      return Interpreter::Result::SUCCESS;  // => Inactive
    });
}

auto Interpreter::engage() const -> void
{
  for (const auto & [name, scenario_object] : currentScenarioDefinition()->entities) {
    if (
      scenario_object.template as<ScenarioObject>().is_added and
      scenario_object.template as<ScenarioObject>().object_controller.isUserDefinedController()) {
      asFieldOperatorApplication(name).engage();
    }
  }
}

auto Interpreter::engageable() const -> bool
{
  return std::all_of(
    std::cbegin(currentScenarioDefinition()->entities),
    std::cend(currentScenarioDefinition()->entities), [this](const auto & each) {
      const auto & [name, scenario_object] = each;
      return not scenario_object.template as<ScenarioObject>().is_added or
             not scenario_object.template as<ScenarioObject>()
                   .object_controller.isUserDefinedController() or
             asFieldOperatorApplication(name).engageable();
    });
}

auto Interpreter::engaged() const -> bool
{
  return std::all_of(
    std::cbegin(currentScenarioDefinition()->entities),
    std::cend(currentScenarioDefinition()->entities), [this](const auto & each) {
      const auto & [name, scenario_object] = each;
      return not scenario_object.template as<ScenarioObject>().is_added or
             not scenario_object.template as<ScenarioObject>()
                   .object_controller.isUserDefinedController() or
             asFieldOperatorApplication(name).engaged();
    });
}

auto Interpreter::on_activate(const rclcpp_lifecycle::State &) -> Result
{
  auto evaluate_storyboard = [this]() {
    withExceptionHandler(
      [this](auto &&...) {
        publishCurrentContext();
        deactivate();
      },
      [this]() {
        withTimeoutHandler(defaultTimeoutHandler(), [this]() {
          if (std::isnan(evaluateSimulationTime())) {
            if (not waiting_for_engagement_to_be_completed and engageable()) {
              engage();
              waiting_for_engagement_to_be_completed = true;  // NOTE: DIRTY HACK!!!
            } else if (engaged()) {
              activateNonUserDefinedControllers();
              waiting_for_engagement_to_be_completed = false;  // NOTE: DIRTY HACK!!!
            }
          } else if (currentScenarioDefinition()) {
            currentScenarioDefinition()->evaluate();
          } else {
            throw Error("No script evaluable.");
          }

          SimulatorCore::update();

          publishCurrentContext();
        });
      });
  };

  if (scenarios.empty()) {
    return Result::FAILURE;
  } else {
    return withExceptionHandler(
      [this](auto &&...) {
        publishCurrentContext();
        reset();
        return Interpreter::Result::FAILURE;  // => Inactive
      },
      [&]() {
        if (record) {
          // clang-format off
          record::start(
            "-a",
            "-o", boost::filesystem::path(osc_path).replace_extension("").string(),
            "-x", "/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/intersection");
          // clang-format on
        }

        SimulatorCore::activate(
          shared_from_this(), makeCurrentConfiguration(), local_real_time_factor,
          1 / local_frame_rate * local_real_time_factor);

        /*
           DIRTY HACK!

           Since traffic_simulator is initially in an undefined internal state,
           it will not have the necessary information to transition from
           Autoware's INITIALIZING state to the WAITING_FOR_ROUTE state unless
           it calls updateFrame at least once. This means that the simulation
           cannot start at exactly zero simulation time, which is a serious
           problem that must be solved in the future.
        */
        SimulatorCore::update();

        execution_timer.clear();

        publisher_of_context->on_activate();

        assert(publisher_of_context->is_activated());

        if (currentScenarioDefinition()) {
          currentScenarioDefinition()->storyboard.init.evaluateInstantaneousActions();
        } else {
          throw Error("No script evaluable.");
        }

        timer = create_wall_timer(currentLocalFrameRate(), evaluate_storyboard);

        return Interpreter::Result::SUCCESS;  // => Active
      });
  }
}

auto Interpreter::on_deactivate(const rclcpp_lifecycle::State &) -> Result
{
  reset();

  return Interpreter::Result::SUCCESS;  // => Inactive
}

auto Interpreter::on_cleanup(const rclcpp_lifecycle::State &) -> Result
{
  scenarios.clear();
  script.reset();
  return Interpreter::Result::SUCCESS;  // => Unconfigured
}

auto Interpreter::on_error(const rclcpp_lifecycle::State &) -> Result
{
  reset();

  return Interpreter::Result::SUCCESS;  // => Unconfigured
}

auto Interpreter::on_shutdown(const rclcpp_lifecycle::State &) -> Result
{
  timer.reset();
  scenarios.clear();
  script.reset();
  SimulatorCore::deactivate();
  return Interpreter::Result::SUCCESS;  // => Finalized
}

auto Interpreter::publishCurrentContext() const -> void
{
  Context context;
  {
    nlohmann::json json;
    context.stamp = now();
    context.data = (json << *script).dump();
    context.time = evaluateSimulationTime();
  }

  publisher_of_context->publish(context);
}

auto Interpreter::reset() -> void
{
  timer.reset();  // Stop scenario evaluation

  if (publisher_of_context->is_activated()) {
    publisher_of_context->on_deactivate();
  }

  SimulatorCore::deactivate();

  scenarios.pop_front();

  // NOTE: Error on simulation is not error of the interpreter; so we print error messages into
  // INFO_STREAM.
  boost::apply_visitor(
    overload(
      [&](const common::junit::Pass & result) { RCLCPP_INFO_STREAM(get_logger(), result); },
      [&](const common::junit::Failure & result) { RCLCPP_INFO_STREAM(get_logger(), result); },
      [&](const common::junit::Error & result) { RCLCPP_INFO_STREAM(get_logger(), result); }),
    result);

  if (record) {
    record::stop();
  }
}
}  // namespace openscenario_interpreter
