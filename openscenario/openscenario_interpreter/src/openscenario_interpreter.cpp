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
#include <boost/json.hpp>
#include <openscenario_interpreter/openscenario_interpreter.hpp>
#include <openscenario_interpreter/record.hpp>
#include <openscenario_interpreter/syntax/object_controller.hpp>
#include <openscenario_interpreter/syntax/parameter_value_distribution.hpp>
#include <openscenario_interpreter/syntax/scenario_definition.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
#include <openscenario_interpreter/syntax/speed_condition.hpp>
#include <openscenario_interpreter/utility/overload.hpp>
#include <status_monitor/status_monitor.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>

#define DECLARE_PARAMETER(IDENTIFIER) \
  declare_parameter<decltype(IDENTIFIER)>(#IDENTIFIER, IDENTIFIER)

#define GET_PARAMETER(IDENTIFIER) get_parameter(#IDENTIFIER, IDENTIFIER)

namespace openscenario_interpreter
{
Interpreter::Interpreter(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("openscenario_interpreter", options),
  publisher_of_context(create_publisher<Context>("context", rclcpp::QoS(1).transient_local())),
  evaluate_time_publisher(create_publisher<tier4_simulation_msgs::msg::UserDefinedValue>(
    "/simulation/interpreter/execution_time_ms/evaluate", rclcpp::QoS(1).transient_local())),
  update_time_publisher(create_publisher<tier4_simulation_msgs::msg::UserDefinedValue>(
    "/simulation/interpreter/execution_time_ms/update", rclcpp::QoS(1).transient_local())),
  output_time_publisher(create_publisher<tier4_simulation_msgs::msg::UserDefinedValue>(
    "/simulation/interpreter/execution_time_ms/output", rclcpp::QoS(1).transient_local())),
  local_frame_rate(30),
  local_real_time_factor(1.0),
  osc_path(""),
  output_directory("/tmp"),
  publish_empty_context(false),
  record(false),
  record_storage_id("")
{
  DECLARE_PARAMETER(local_frame_rate);
  DECLARE_PARAMETER(local_real_time_factor);
  DECLARE_PARAMETER(osc_path);
  DECLARE_PARAMETER(output_directory);
  DECLARE_PARAMETER(publish_empty_context);
  DECLARE_PARAMETER(record);
  DECLARE_PARAMETER(record_storage_id);

  declare_parameter<std::string>("speed_condition", "legacy");
  SpeedCondition::compatibility =
    boost::lexical_cast<Compatibility>(get_parameter("speed_condition").as_string());
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
  const auto is_directly_lanelet2_map_file =
    not logic_file.isDirectory() and logic_file.filepath.extension() == ".osm";
  const auto map_files_path =
    is_directly_lanelet2_map_file ? logic_file.filepath.parent_path() : logic_file;

  // XXX DIRTY HACK!!!
  if (is_directly_lanelet2_map_file) {
    return traffic_simulator::Configuration(
      map_files_path, logic_file.filepath.filename().string(), osc_path);
  } else {
    return traffic_simulator::Configuration(map_files_path, osc_path);
  }
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
      GET_PARAMETER(publish_empty_context);
      GET_PARAMETER(record);
      GET_PARAMETER(record_storage_id);

      script = std::make_shared<OpenScenario>(osc_path);

      // CanonicalizedLaneletPose is also used on the OpenScenarioInterpreter side as NativeLanePose.
      // so canonicalization takes place here - it uses the value of the consider_pose_by_road_slope parameter
      traffic_simulator::lanelet_pose::CanonicalizedLaneletPose::setConsiderPoseByRoadSlope([&]() {
        if (not has_parameter("consider_pose_by_road_slope")) {
          declare_parameter("consider_pose_by_road_slope", false);
        }
        return get_parameter("consider_pose_by_road_slope").as_bool();
      }());

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
      scenario_object.template is<ScenarioObject>() and
      scenario_object.template as<ScenarioObject>().is_added and
      scenario_object.template as<ScenarioObject>().object_controller.isAutoware()) {
      NonStandardOperation::engage(name);
    }
  }
}

auto Interpreter::engageable() const -> bool
{
  return std::all_of(
    std::cbegin(currentScenarioDefinition()->entities),
    std::cend(currentScenarioDefinition()->entities), [this](const auto & each) {
      const auto & [name, scenario_object] = each;
      return not scenario_object.template is<ScenarioObject>() or
             not scenario_object.template as<ScenarioObject>().is_added or
             not scenario_object.template as<ScenarioObject>().object_controller.isAutoware() or
             NonStandardOperation::isEngageable(name);
    });
}

auto Interpreter::engaged() const -> bool
{
  return std::all_of(
    std::cbegin(currentScenarioDefinition()->entities),
    std::cend(currentScenarioDefinition()->entities), [this](const auto & each) {
      const auto & [name, scenario_object] = each;
      return not scenario_object.template is<ScenarioObject>() or
             not scenario_object.template as<ScenarioObject>().is_added or
             not scenario_object.template as<ScenarioObject>().object_controller.isAutoware() or
             NonStandardOperation::isEngaged(name);
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
        const auto evaluate_time = execution_timer.invoke("evaluate", [&]() {
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
        });

        const auto update_time =
          execution_timer.invoke("update", [&]() { SimulatorCore::update(); });

        const auto output_time =
          execution_timer.invoke("output", [&]() { publishCurrentContext(); });

        tier4_simulation_msgs::msg::UserDefinedValue msg;
        msg.type.data = tier4_simulation_msgs::msg::UserDefinedValueType::DOUBLE;
        msg.value =
          std::to_string(std::chrono::duration<double, std::milli>(evaluate_time).count());
        evaluate_time_publisher->publish(msg);
        msg.value = std::to_string(std::chrono::duration<double, std::milli>(update_time).count());
        update_time_publisher->publish(msg);
        msg.value = std::to_string(std::chrono::duration<double, std::milli>(output_time).count());
        output_time_publisher->publish(msg);

        if (auto time_until_trigger = timer->time_until_trigger(); time_until_trigger.count() < 0) {
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
          RCLCPP_WARN_STREAM(
            get_logger(),
            "Your machine is not powerful enough to run the scenario at the specified frame rate ("
              << local_frame_rate << " Hz). Current frame execution exceeds "
              << -time_until_trigger.count() / 1.e6 << " milliseconds.");
        }
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
          if (record_storage_id == "") {
            record::start(
              "-a", "-o", boost::filesystem::path(osc_path).replace_extension("").string());
          } else {
            record::start(
              "-a", "-o", boost::filesystem::path(osc_path).replace_extension("").string(), "-s",
              record_storage_id);
          }
        }

        SimulatorCore::activate(
          shared_from_this(), makeCurrentConfiguration(), local_real_time_factor, local_frame_rate);

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
        evaluate_time_publisher->on_activate();
        update_time_publisher->on_activate();
        output_time_publisher->on_activate();

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
    boost::json::monotonic_resource mr;
    boost::json::object json(&mr);
    context.stamp = now();
    if (publish_empty_context) {
      context.data = "";
    } else {
      context.data = boost::json::serialize(json << *script);
    }
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
  if (evaluate_time_publisher->is_activated()) {
    evaluate_time_publisher->on_deactivate();
  }
  if (update_time_publisher->is_activated()) {
    update_time_publisher->on_deactivate();
  }
  if (output_time_publisher->is_activated()) {
    output_time_publisher->on_deactivate();
  }

  if (not has_parameter("initialize_duration")) {
    declare_parameter<int>("initialize_duration", 30);
  }

  /*
     Although we have not analyzed the details yet, the process of deactivating
     the simulator core takes quite a long time (especially the
     traffic_simulator::API::despawnEntities function is slow). During the
     process, the interpreter becomes unresponsive, which often resulted in the
     status monitor thread judging the interpreter as "not good". Therefore, we
     will increase the threshold of the judgment only during the process of
     deactivating the simulator core.

     The threshold value here is set to the value of initialize_duration, but
     there is no rationale for this; it should be larger than the original
     threshold value of the status monitor and long enough for the simulator
     core to be deactivated.
  */
  common::status_monitor.overrideThreshold(
    std::chrono::seconds(get_parameter("initialize_duration").as_int()), SimulatorCore::deactivate);

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
