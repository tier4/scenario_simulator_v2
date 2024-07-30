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

#include <iterator>  // std::distance
#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/posix/fork_exec.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/content.hpp>
#include <openscenario_interpreter/regex/function_call_expression.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/custom_command_action.hpp>
#include <openscenario_interpreter/syntax/storyboard_element.hpp>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
template <auto Version>
struct ApplyFaultInjectionAction : public CustomCommand
{
  static auto node() -> auto &
  {
    static auto node = []() {
      auto options = rclcpp::NodeOptions();
      options.use_global_arguments(false);
      return rclcpp::Node(
        "custom_command_action_" + boost::lexical_cast<std::string>(std::this_thread::get_id()),
        "simulation", options);
    }();
    return node;
  }

  static auto publisher() -> auto &
  {
    static auto publisher =
      node().template create_publisher<tier4_simulation_msgs::msg::SimulationEvents>(
        "events", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    return *publisher;
  }

  template <typename... Ts>
  explicit ApplyFaultInjectionAction(Ts &&... xs) : CustomCommand(std::forward<decltype(xs)>(xs)...)
  {
    publisher();
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
struct ApplyPseudoTrafficSignalDetectorConfidenceSetAction
: public CustomCommand,
  public SimulatorCore::NonStandardOperation
{
  using CustomCommand::CustomCommand;

  auto start(const Scope &) -> void override
  {
    static_assert(0 < Version and Version <= 1);
    if (parameters.size() == 2) {
      setConventionalTrafficLightConfidence(
        std::stoi(parameters.at(0)), std::stod(parameters.at(1)));
    } else {
      throw Error(
        "An unexpected number of arguments were passed to "
        "PseudoTrafficSignalDetectorConfidenceSetAction. "
        "Expected 2 arguments, but actually passed ",
        parameters.size(), ".");
    }
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
    if (parameters.size() == 2) {
      sendCooperateCommand(parameters.at(0), parameters.at(1));
    } else {
      throw Error(
        "An unexpected number of arguments were passed to RequestToCooperateCommandAction. "
        "Expected 2 arguments, but actually passed ",
        parameters.size(), ".");
    }
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
      actor.apply([&](const auto & object) { applyWalkStraightAction(object); });
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
  using CustomCommand::CustomCommand;

  auto start(const Scope &) -> void override { throw SpecialAction<EXIT_FAILURE>(); }
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
#define ELEMENT(NAME, TYPE) \
  std::make_pair(           \
    NAME, [](auto &&... xs) { return std::make_shared<TYPE>(std::forward<decltype(xs)>(xs)...); })

  static const std::unordered_map<
    std::string, std::function<std::shared_ptr<CustomCommand>(const std::vector<std::string> &)>>
    commands{
      // clang-format off
      ELEMENT("FaultInjectionAction", ApplyFaultInjectionAction<1>),
      ELEMENT("FaultInjectionAction@v1", ApplyFaultInjectionAction<1>),
      ELEMENT("FaultInjectionAction@v2", ApplyFaultInjectionAction<2>),
      ELEMENT("PseudoTrafficSignalDetectorConfidenceSetAction@v1", ApplyPseudoTrafficSignalDetectorConfidenceSetAction<1>),
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
      // clang-format on
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
