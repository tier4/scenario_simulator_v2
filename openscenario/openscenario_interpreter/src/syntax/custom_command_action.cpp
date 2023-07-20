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

#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

#include <iterator>  // std::distance
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/posix/fork_exec.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/content.hpp>
#include <openscenario_interpreter/regex/function_call_expression.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/custom_command_action.hpp>
#include <openscenario_interpreter/syntax/storyboard_element.hpp>
#include <openscenario_interpreter_msgs/msg/context.hpp>
#include "rclcpp/rclcpp.hpp"
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
template <auto Version>
struct ApplyFaultInjectionAction : public CustomCommand
{
  using CustomCommand::CustomCommand;

  static auto node() -> rclcpp::Node &
  {
    static rclcpp::Node node{"custom_command_action", "simulation"};
    return node;
  }

  static auto publisher() -> rclcpp::Publisher<tier4_simulation_msgs::msg::SimulationEvents> &
  {
    static auto publisher = node().create_publisher<tier4_simulation_msgs::msg::SimulationEvents>(
      "/simulation/events", rclcpp::QoS(1).reliable());
    return *publisher;
  }

  auto start(const Scope &) -> void override
  {
    static_assert(0 < Version and Version <= 2);

    auto makeFaultInjectionEvent = [](const auto & level, const auto & name) {
      tier4_simulation_msgs::msg::FaultInjectionEvent fault_injection_event;
      fault_injection_event.level = level;
      fault_injection_event.name = name;
      return fault_injection_event;
    };

    tier4_simulation_msgs::msg::SimulationEvents simulation_events;

    simulation_events.stamp = node().now();

    if constexpr (Version == 1) {
      for (const auto & event : parameters) {
        simulation_events.fault_injection_events.push_back(
          makeFaultInjectionEvent(tier4_simulation_msgs::msg::FaultInjectionEvent::ERROR, event));
      }
    } else {
      auto makeFaultInjectionEventLevel = [](const auto & level) {
        if (level == "OK") {
          return tier4_simulation_msgs::msg::FaultInjectionEvent::OK;
        } else if (level == "WARN" or level == "WARNING") {
          return tier4_simulation_msgs::msg::FaultInjectionEvent::WARN;
        } else if (level == "ERROR") {
          return tier4_simulation_msgs::msg::FaultInjectionEvent::ERROR;
        } else if (level == "STALE") {
          return tier4_simulation_msgs::msg::FaultInjectionEvent::STALE;
        } else {
          throw Error(
            "FaultInjectionAction@v2 expects error level to be given as first argument, but ",
            level,
            " was given. This is not a valid error level specification. Valid error levels are OK, "
            "WARN, ERROR, and STALE.");
        }
      };

      simulation_events.fault_injection_events.push_back(
        makeFaultInjectionEvent(makeFaultInjectionEventLevel(parameters[0]), parameters[1]));
    }

    publisher().publish(simulation_events);
  }
};

template <auto Version>
struct ApplyRequestToCorporateCommandAction : public CustomCommand,
                                              public SimulatorCore::NonStandardOperation
{
  using CustomCommand::CustomCommand;

  auto start(const Scope &) -> void override
  {
    static_assert(0 < Version and Version <= 1);
    sendCooperateCommand(parameters.at(0), parameters.at(1));
  }
};

struct ApplyV2ITrafficSignalStateAction : public CustomCommand,
                                          public SimulatorCore::NonStandardOperation
{
  using CustomCommand::CustomCommand;

  auto start(const Scope &) -> void override
  {
    auto unquote = [](auto s) {
      std::stringstream(s) >> std::quoted(s);
      return s;
    };

    switch (parameters.size()) {
      case 3:
        resetV2ITrafficLightPublishRate(boost::lexical_cast<double>(parameters[2]));
        [[fallthrough]];

      case 2:
        for (auto & traffic_light :
             getV2ITrafficLights(boost::lexical_cast<std::int64_t>(parameters[0]))) {
          traffic_light.get().clear();
          traffic_light.get().set(unquote(parameters.at(1)));
        }
        break;

      default:
        throw Error(
          "An unexpected number of arguments were passed to V2ITrafficSignalStateAction. Expected "
          "2 or 3 arguments, but actually passed ",
          parameters.size(), ".");
    }
  }
};

struct ApplyWalkStraightAction : public CustomCommand, private SimulatorCore::ActionApplication
{
  using CustomCommand::CustomCommand;

  auto start(const Scope & scope) -> void override
  {
    for (const auto & actor : parameters) {
      applyWalkStraightAction(actor);
    }

    for (const auto & actor : scope.actors) {
      applyWalkStraightAction(actor);
    }
  };
};

struct DebugError : public CustomCommand
{
  using CustomCommand::CustomCommand;

  auto start(const Scope &) -> void override { throw Error(__FILE__, ":", __LINE__); }
};

struct DebugSegmentationFault : public CustomCommand
{
  using CustomCommand::CustomCommand;

  auto start(const Scope &) -> void override
  {
    [[maybe_unused]] auto x =
      *reinterpret_cast<std::add_pointer_t<int>>(0);  // NOTE: Access null-pointer explicitly.
  }
};

struct DummyLongRunningAction : public CustomCommand, private SimulatorCore::ConditionEvaluation
{
  double end_time = std::numeric_limits<double>::max();

  using CustomCommand::CustomCommand;

  auto accomplished() noexcept -> bool override { return end_time < evaluateSimulationTime(); }

  auto endsImmediately() const -> bool override { return false; }

  auto start(const Scope & scope) -> void override
  {
    end_time = evaluateSimulationTime() + boost::lexical_cast<double>(parameters.at(0));

    if (not scope.name.empty()) {
      if (auto e = scope.ref(scope.name); e.is_also<StoryboardElement>()) {
        e.as<StoryboardElement>().addTransitionCallback(
          StoryboardElementState::runningState, [name = scope.name](auto &&) {
            std::cout << "[" << std::setprecision(2) << evaluateSimulationTime() << "s] " << name
                      << " transitions to runningState" << std::endl;
          });
        e.as<StoryboardElement>().addTransitionCallback(
          StoryboardElementState::completeState, [name = scope.name](auto &&) {
            std::cout << "[" << std::setprecision(2) << evaluateSimulationTime() << "s] " << name
                      << " transitions to completeState" << std::endl;
          });
      }
    }
  }
};

struct ExitSuccess : public CustomCommand
{
  using CustomCommand::CustomCommand;

  auto start(const Scope &) -> void override { throw SpecialAction<EXIT_SUCCESS>(); }
};

struct ExitFailure : public CustomCommand
{
  struct Condition
  {
    std::string current_evaluation;
    std::string current_value;
    std::string type;
  };

  struct ConditionGroup
  {
    std::vector<Condition> conditions;
  };

  struct ConditionGroups
  {
    std::string groups_name;
    std::vector<ConditionGroup> condition_groups;
  };

  using CustomCommand::CustomCommand;
  using Context = openscenario_interpreter_msgs::msg::Context;
  using json = nlohmann::json;
  using ConditionGroupsCollection = std::vector<ConditionGroups>;

  rclcpp::Subscription<Context>::SharedPtr subscription_;
  std::string message_;
  ConditionGroupsCollection condition_groups_collection_; 

  auto start(const Scope &) -> void override {
    auto node = std::make_shared<rclcpp::Node>("exit_failure");
    // Subscribe to the topic
    subscription_ = node->create_subscription<Context>(
        "/simulation/context",
        rclcpp::QoS(10).transient_local(),
        [this](Context::UniquePtr msg_ptr) {
          if (!msg_ptr) return;

          YAML::Node data;
          try {
            data = YAML::Load(msg_ptr->data);
          } catch (const std::exception & e) {
            throw std::runtime_error(std::string("Failed to load YAML: ") + e.what());
          }

          int unnamed_event_counter = 1;
          condition_groups_collection_.clear();

          auto stories = data["OpenSCENARIO"]["Storyboard"]["Story"];
          for (const auto & story : stories) {
            for (const auto & act : story["Act"]) {
              for (const auto & maneuver_group : act["ManeuverGroup"]) {
                for (const auto & maneuver : maneuver_group["Maneuver"]) {
                  for (const auto & event : maneuver["Event"]) {
                    if (event["StartTrigger"] && event["StartTrigger"]["ConditionGroup"]) {
                      std::string event_name;
                      try {
                        event_name = event["name"].as<std::string>();
                      } catch (const YAML::BadConversion & e) {
                        event_name = "";
                      }
                      if (event_name.empty()) {
                        event_name = "ConditionGroup" + std::to_string(unnamed_event_counter++);
                      }

                      ConditionGroups condition_groups_msg;
                      condition_groups_msg.groups_name = event_name;

                      for (const auto & condition_group_node : event["StartTrigger"]["ConditionGroup"]) {
                        ConditionGroup condition_group_msg;

                        for (const auto & condition_node : condition_group_node["Condition"]) {
                          Condition condition_msg;
                          condition_msg.current_evaluation = condition_node["currentEvaluation"].as<std::string>();
                          condition_msg.current_value = condition_node["currentValue"].as<std::string>();
                          condition_msg.type = condition_node["type"].as<std::string>();

                          condition_group_msg.conditions.push_back(condition_msg);
                        }

                        condition_groups_msg.condition_groups.push_back(condition_group_msg);
                      }
                      condition_groups_collection_.push_back(condition_groups_msg);
                    }
                  }
                }
              }
            }
          }

          std::ostringstream context_ss;
          for (const auto & condition_groups : condition_groups_collection_) {
            context_ss << std::fixed << std::setprecision(0)
                      << "Condition Groups Name: " << condition_groups.groups_name << std::endl;
            for (const auto & condition_group : condition_groups.condition_groups) {
              for (const auto & condition : condition_group.conditions) {
                context_ss << "  Current Evaluation: " << condition.current_evaluation << std::endl
                          << "  Current Value: " << condition.current_value << std::endl
                          << "  Type: " << condition.type << std::endl;
              }
            }
            context_ss << std::endl;
          }
          throw std::runtime_error(context_ss.str());
        });

    // Spin until a message is received
    for (int i = 0; i < 10 && message_.empty(); ++i) {
      rclcpp::spin_some(node);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    throw SpecialAction<EXIT_FAILURE>();
  }
};


struct PrintParameter : public CustomCommand
{
  using CustomCommand::CustomCommand;

  auto start(const Scope & scope) -> void override
  {
    for (auto && parameter : parameters) {
      std::cout << parameter << " = " << scope.ref(parameter) << std::endl;
    }
  }
};

struct TestCommand : public CustomCommand
{
  using CustomCommand::CustomCommand;

  auto start(const Scope &) -> void override
  {
    std::cout << "test" << std::endl;

    for (auto iter = std::cbegin(parameters); iter != std::cend(parameters); ++iter) {
      std::cout << "  parameters[" << std::distance(std::cbegin(parameters), iter)
                << "] = " << *iter << std::endl;
    }
  }
};

struct ForkExecCommand : public CustomCommand
{
  const std::string type;

  const std::string content;

  explicit ForkExecCommand(const std::string & type, const std::string & content)
  : type(type), content(content)
  {
  }

  auto start(const Scope &) -> void override { fork_exec(type, content); }
};

auto makeCustomCommand(const std::string & type, const std::string & content)
  -> std::shared_ptr<CustomCommand>
{
#define ELEMENT(NAME, TYPE)                                                             \
  std::make_pair(NAME, [](auto &&... xs) {                                              \
    return std::shared_ptr<CustomCommand>(new TYPE(std::forward<decltype(xs)>(xs)...)); \
  })

  static const std::unordered_map<
    std::string, std::function<std::shared_ptr<CustomCommand>(const std::vector<std::string> &)>>
    commands{
      ELEMENT("FaultInjectionAction", ApplyFaultInjectionAction<1>),
      ELEMENT("FaultInjectionAction@v1", ApplyFaultInjectionAction<1>),
      ELEMENT("FaultInjectionAction@v2", ApplyFaultInjectionAction<2>),
      ELEMENT("RequestToCooperateCommandAction@v1", ApplyRequestToCorporateCommandAction<1>),
      ELEMENT("V2ITrafficSignalStateAction", ApplyV2ITrafficSignalStateAction),
      ELEMENT("WalkStraightAction", ApplyWalkStraightAction),
      ELEMENT("debugError", DebugError),
      ELEMENT("debugSegmentationFault", DebugSegmentationFault),  // DEPRECATED
      ELEMENT("dummyLongRunningAction", DummyLongRunningAction),
      ELEMENT("exitFailure", ExitFailure),
      ELEMENT("exitSuccess", ExitSuccess),
      ELEMENT("printParameter", PrintParameter),
      ELEMENT("test", TestCommand),
    };
#undef ELEMENT

  if (type == ":") {
    return std::make_shared<CustomCommand>();
  } else if (std::smatch result;
             std::regex_match(type, result, FunctionCallExpression::pattern()) and
             commands.find(result[1]) != std::end(commands)) {
    return commands.at(result[1])(FunctionCallExpression::splitParameters(result[3]));
  } else {
    return std::make_shared<ForkExecCommand>(type, content);
  }
}

CustomCommandAction::CustomCommandAction(const pugi::xml_node & node, const Scope & scope)
: Scope(scope),
  type(readAttribute<String>("type", node, local())),
  content(readContent<String>(node, local())),
  command(makeCustomCommand(type, content))
{
}
}  // namespace syntax
}  // namespace openscenario_interpreter
