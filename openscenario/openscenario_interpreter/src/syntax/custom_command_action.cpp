// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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
#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/content.hpp>
#include <openscenario_interpreter/syntax/custom_command_action.hpp>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
CustomCommandAction::CustomCommandAction(const pugi::xml_node & node, const Scope & scope)
: Scope(scope),
  type(readAttribute<String>("type", node, local())),
  content(readContent<String>(node, local()))
{
}

auto CustomCommandAction::accomplished() noexcept -> bool { return true; }

auto CustomCommandAction::applyFaultInjectionAction(
  const std::vector<std::string> & events, const Scope &) -> int
{
  const auto now = node().now();

  auto makeFaultInjectionEvent = [](const auto & name) {
    autoware_simulation_msgs::msg::FaultInjectionEvent fault_injection_event;
    {
      fault_injection_event.level = autoware_simulation_msgs::msg::FaultInjectionEvent::ERROR;
      fault_injection_event.name = name;
    }

    return fault_injection_event;
  };

  auto makeFaultInjectionEvents = [&](const std::vector<std::string> & events) {
    autoware_simulation_msgs::msg::SimulationEvents simulation_events;
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

auto CustomCommandAction::applyWalkStraightAction(
  const std::vector<std::string> & actors, const Scope & scope) -> int
{
  for (const auto & actor : actors) {
    openscenario_interpreter::applyWalkStraightAction(actor);
  }

  for (const auto & actor : scope.actors) {
    openscenario_interpreter::applyWalkStraightAction(actor);
  }

  return scope.actors.size();
}

auto CustomCommandAction::debugError(const std::vector<std::string> &, const Scope &) -> int
{
  throw Error(__FILE__, ":", __LINE__);
}

auto CustomCommandAction::debugSegmentationFault(const std::vector<std::string> &, const Scope &)
  -> int
{
  return *reinterpret_cast<std::add_pointer<int>::type>(0);
}

auto CustomCommandAction::exitSuccess(const std::vector<std::string> &, const Scope &) -> int
{
  throw SpecialAction<EXIT_SUCCESS>();
}

auto CustomCommandAction::exitFailure(const std::vector<std::string> &, const Scope &) -> int
{
  throw SpecialAction<EXIT_FAILURE>();
}

auto CustomCommandAction::node() -> rclcpp::Node &
{
  static rclcpp::Node node{"custom_command_action", "simulation"};
  return node;
}

auto CustomCommandAction::publisher()
  -> rclcpp::Publisher<autoware_simulation_msgs::msg::SimulationEvents> &
{
  static auto publisher = node().create_publisher<autoware_simulation_msgs::msg::SimulationEvents>(
    "/simulation/events", rclcpp::QoS(1).reliable());
  return *publisher;
}

auto CustomCommandAction::run() noexcept -> void {}

auto CustomCommandAction::split(const std::string & s) -> std::vector<std::string>
{
  static const std::regex pattern{R"(([^\("\s,\)]+|\"[^"]*\"),?\s*)"};

  std::vector<std::string> args{};

  for (std::sregex_iterator iter{std::begin(s), std::end(s), pattern}, end; iter != end; ++iter) {
    args.emplace_back((*iter)[1]);
  }

  return args;
}

auto CustomCommandAction::start() const -> void
{
  static const std::unordered_map<
    std::string, std::function<int(const std::vector<std::string> &, const Scope &)>>
    commands{
      std::make_pair("FaultInjectionAction", applyFaultInjectionAction),
      std::make_pair("WalkStraightAction", applyWalkStraightAction),
      std::make_pair("debugError", debugError),
      std::make_pair("debugSegmentationFault", debugSegmentationFault),  // DEPRECATED
      std::make_pair("exitFailure", exitFailure),
      std::make_pair("exitSuccess", exitSuccess),
      std::make_pair("test", test),
    };

  /* ---- NOTE ---------------------------------------------------------------
   *
   *  <CustomCommandAction type="function(foo, &quot;hello, world!&quot;, 3.14)"/>
   *
   *    result[0] = function(foo, "hello, world!", 3.14)
   *    result[1] = function
   *    result[2] =         (foo, "hello, world!", 3.14)
   *    result[3] =          foo, "hello, world!", 3.14
   *
   * ---------------------------------------------------------------------- */
  static const std::regex pattern{R"(^(\w+)(\(((?:(?:[^\("\s,\)]+|\"[^"]*\"),?\s*)*)\))?$)"};

  std::smatch result{};

  if (type == ":") {
    return;
  } else if (
    std::regex_match(type, result, pattern) and commands.find(result[1]) != std::end(commands)) {
    commands.at(result[1])(split(result[3]), local());
  } else {
    fork_exec(type, content);
  }
}

auto CustomCommandAction::test(const std::vector<std::string> & args, const Scope &) -> int
{
  std::cout << "test" << std::endl;

  for (auto iter = std::cbegin(args); iter != std::cend(args); ++iter) {
    std::cout << "  args[" << std::distance(std::cbegin(args), iter) << "] = " << *iter << "\n";
  }

  return args.size();
}

}  // namespace syntax
}  // namespace openscenario_interpreter
