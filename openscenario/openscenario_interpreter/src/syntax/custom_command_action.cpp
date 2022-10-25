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

#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/posix/fork_exec.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/content.hpp>
#include <openscenario_interpreter/regex/function_call_expression.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/custom_command_action.hpp>
#include <openscenario_interpreter/syntax/storyboard_element.hpp>

#include <iterator>  // std::distance
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{

struct ApplyFaultInjection : public ICustomCommand
{
  using ICustomCommand::ICustomCommand;

private:
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

public:
  auto start(const Scope &) -> int override
  {
    auto & events = parameters;
    const auto now = node().now();

    auto makeFaultInjectionEvent = [](const auto & name) {
      tier4_simulation_msgs::msg::FaultInjectionEvent fault_injection_event;
      {
        fault_injection_event.level = tier4_simulation_msgs::msg::FaultInjectionEvent::ERROR;
        fault_injection_event.name = name;
      }

      return fault_injection_event;
    };

    auto makeFaultInjectionEvents = [&](const std::vector<std::string> & events) {
      tier4_simulation_msgs::msg::SimulationEvents simulation_events;
      {
        simulation_events.stamp = now;

        for (const auto & event : events) {
          simulation_events.fault_injection_events.push_back(makeFaultInjectionEvent(event));
        }
      }

      return simulation_events;
    };

    publisher().publish(makeFaultInjectionEvents(events));

    return events.size();
  }
};

struct ApplyWalkStraightAction : public ICustomCommand, private SimulatorCore::ActionApplication
{
  using ICustomCommand::ICustomCommand;

  auto start(const Scope & scope) -> int override
  {
    for (const auto & actor : parameters) {
      applyWalkStraightAction(actor);
    }

    for (const auto & actor : scope.actors) {
      applyWalkStraightAction(actor);
    }

    return scope.actors.size();
  };
};

struct DebugError : public ICustomCommand
{
  using ICustomCommand::ICustomCommand;

  auto start(const Scope &) -> int override { throw Error(__FILE__, ":", __LINE__); }
};

struct DebugSegmentationFault : public ICustomCommand
{
  using ICustomCommand::ICustomCommand;

  auto start(const Scope &) -> int override
  {
    return *reinterpret_cast<std::add_pointer<int>::type>(0);
  }
};

struct DummyLongRunningAction : public ICustomCommand, private SimulatorCore::ConditionEvaluation
{
  using ICustomCommand::ICustomCommand;

  auto accomplished() noexcept -> bool override { return end_time < evaluateSimulationTime(); }

  auto endsImmediately() const -> bool override { return false; }

  auto start(const Scope & scope) -> int override
  {
    auto duration = std::stod(parameters.at(0));
    auto now = evaluateSimulationTime();
    end_time = now + duration;

    if (!scope.name.empty()) {
      auto e = scope.ref(scope.name);
      if (e.is_also<StoryboardElement>()) {
        auto & storyboard_element = e.as<StoryboardElement>();
        storyboard_element.addTransitionCallback(
          StoryboardElementState::runningState, [name = scope.name](auto &&) {
            std::cout << "[" << std::setprecision(2) << evaluateSimulationTime() << "s]" << name
                      << " transitions to runningState" << std::endl;
          });
        storyboard_element.addTransitionCallback(
          StoryboardElementState::completeState, [name = scope.name](auto &&) {
            std::cout << "[" << std::setprecision(2) << evaluateSimulationTime() << "s]" << name
                      << " transitions to completeState" << std::endl;
          });
      }
    }

    return 0;
  }

private:
  double end_time = std::numeric_limits<double>::max();
};

struct ExitSuccess : public ICustomCommand
{
  using ICustomCommand::ICustomCommand;

  auto start(const Scope &) -> int override { throw SpecialAction<EXIT_SUCCESS>(); }
};

struct ExitFailure : public ICustomCommand
{
  using ICustomCommand::ICustomCommand;

  auto start(const Scope &) -> int override { throw SpecialAction<EXIT_FAILURE>(); }
};

struct PrintParameter : public ICustomCommand
{
  using ICustomCommand::ICustomCommand;

  auto start(const Scope & scope) -> int override
  {
    for (auto && parameter : parameters) {
      std::cout << parameter << " = " << scope.ref(parameter) << std::endl;
    }

    return parameters.size();
  }
};

struct TestCommand : public ICustomCommand
{
  using ICustomCommand::ICustomCommand;

  auto start(const Scope &) -> int override
  {
    std::cout << "test" << std::endl;

    for (auto iter = std::cbegin(parameters); iter != std::cend(parameters); ++iter) {
      std::cout << "  parameters[" << std::distance(std::cbegin(parameters), iter)
                << "] = " << *iter << std::endl;
    }

    return parameters.size();
  }
};

struct ForkExecCommand : public ICustomCommand
{
  ForkExecCommand(const std::string & type, const std::string & content)
  : type(type), content(content)
  {
  }

  auto start(const Scope &) -> int override { return fork_exec(type, content); }

private:
  std::string type;
  std::string content;
};

namespace
{
auto dispatchCustomCommand(const std::string & type, const std::string & content)
  -> std::shared_ptr<ICustomCommand>
{
  std::smatch result{};

#define ELEMENT(NAME, TYPE)                                                  \
  std::make_pair(NAME, [](std::vector<std::string> parameters) {             \
    return std::shared_ptr<ICustomCommand>(new TYPE(std::move(parameters))); \
  })

  static const std::unordered_map<
    std::string, std::function<std::shared_ptr<ICustomCommand>(std::vector<std::string>)>>
    commands{
      ELEMENT("FaultInjectionAction", ApplyFaultInjection),
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
    return std::make_shared<ICustomCommand>();
  } else if (
    std::regex_match(type, result, FunctionCallExpression::pattern()) and
    commands.find(result[1]) != std::end(commands)) {
    return commands.at(result[1])(FunctionCallExpression::splitParameters(result[3]));
  } else {
    return std::make_shared<ForkExecCommand>(type, content);
  }
}
}  // namespace

CustomCommandAction::CustomCommandAction(const pugi::xml_node & node, const Scope & scope)
: Scope(scope),
  type(readAttribute<String>("type", node, local())),
  content(readContent<String>(node, local())),
  entity(dispatchCustomCommand(type, content))
{
}

}  // namespace syntax
}  // namespace openscenario_interpreter
