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

#include <iterator>  // std::distance
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{

struct ApplyFaultInjection : public CustomCommandActionBase
{
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
  auto start(const std::vector<std::string> & events, const Scope &) const -> int override
  {
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

struct ApplyWalkStraightAction : public CustomCommandActionBase, private SimulatorCore::ActionApplication
{
  auto start(const std::vector<std::string> & actors, const Scope & scope) const -> int override
  {
    for (const auto & actor : actors) {
      applyWalkStraightAction(actor);
    }

    for (const auto & actor : scope.actors) {
      applyWalkStraightAction(actor);
    }

    return scope.actors.size();
  };
};

struct DebugError : public CustomCommandActionBase
{
  auto start(const std::vector<std::string> &, const Scope &) const -> int override
  {
    throw Error(__FILE__, ":", __LINE__);
  }
};

struct DebugSegmentationFault : public CustomCommandActionBase
{
  auto start(const std::vector<std::string> &, const Scope &) const -> int override
  {
    return *reinterpret_cast<std::add_pointer<int>::type>(0);
  }
};

struct ExitSuccess : public CustomCommandActionBase
{
  auto start(const std::vector<std::string> &, const Scope &) const -> int override
  {
    throw SpecialAction<EXIT_SUCCESS>();
  }
};

struct ExitFailure : public CustomCommandActionBase
{
  auto start(const std::vector<std::string> &, const Scope &) const -> int override
  {
    throw SpecialAction<EXIT_FAILURE>();
  }
};

struct PrintParameter : public CustomCommandActionBase
{
  auto start(const std::vector<std::string> & parameters, const Scope & scope) const -> int override
  {
    for (auto && parameter : parameters) {
      std::cout << parameter << " = " << scope.ref(parameter) << std::endl;
    }

    return parameters.size();
  }
};

struct TestCommand : public CustomCommandActionBase
{
  auto start(const std::vector<std::string> & values, const Scope &) const -> int override
  {
    std::cout << "test" << std::endl;

    for (auto iter = std::cbegin(values); iter != std::cend(values); ++iter) {
      std::cout << "  values[" << std::distance(std::cbegin(values), iter) << "] = " << *iter
                << std::endl;
    }

    return values.size();
  }
};

CustomCommandAction::CustomCommandAction(const pugi::xml_node & node, const Scope & scope)
: Scope(scope),
  type(readAttribute<String>("type", node, local())),
  content(readContent<String>(node, local())),
  base(new CustomCommandActionBase())
{
}

auto CustomCommandAction::start() -> void
{
#define ELEMENT(NAME, TYPE) \
  std::make_pair(NAME, [] { return std::shared_ptr<CustomCommandActionBase>(new TYPE()); })

  static const std::unordered_map<std::string, std::function<std::shared_ptr<CustomCommandActionBase>()>> commands{
    ELEMENT("FaultInjectionAction", ApplyFaultInjection),
    ELEMENT("WalkStraightAction", ApplyWalkStraightAction),
    ELEMENT("debugError", DebugError),
    ELEMENT("debugSegmentationFault", DebugSegmentationFault),  // DEPRECATED
    ELEMENT("exitFailure", ExitFailure),
    ELEMENT("exitSuccess", ExitSuccess),
    ELEMENT("printParameter", PrintParameter),
    ELEMENT("test", TestCommand),
  };

  std::smatch result{};

  if (type == ":") {
    return;
  } else if (
    std::regex_match(type, result, FunctionCallExpression::pattern()) and
    commands.find(result[1]) != std::end(commands)) {
    base = commands.at(result[1])();
    base->start(FunctionCallExpression::splitParameters(result[3]), local());
  } else {
    fork_exec(type, content);
  }
}

}  // namespace syntax
}  // namespace openscenario_interpreter
