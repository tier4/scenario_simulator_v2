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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_

#include <openscenario_interpreter/scenario_failure.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <pugixml.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tier4_simulation_msgs/msg/simulation_events.hpp>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
template <int Value>
struct SpecialAction : public std::integral_constant<int, Value>
{
};

template <>
struct SpecialAction<EXIT_FAILURE> : public std::integral_constant<int, EXIT_FAILURE>,
                                     public ScenarioFailure
{
  using ScenarioFailure::ScenarioFailure;
  SpecialAction(const ScenarioFailure & failure) : ScenarioFailure{failure} {}
};

struct CustomCommand
{
  const std::vector<std::string> parameters;

  CustomCommand() = default;

  CustomCommand(const CustomCommand &) = default;

  CustomCommand(CustomCommand &&) = default;

  explicit CustomCommand(const std::vector<std::string> & parameters) : parameters(parameters) {}

  virtual ~CustomCommand() = default;

  virtual auto accomplished() noexcept -> bool { return true; }

  virtual auto endsImmediately() const -> bool { return true; }

  virtual auto run() noexcept -> void {}

  virtual auto start(const Scope &) -> void {}
};

/* ---- CustomCommandAction ----------------------------------------------------
 *
 *  <xsd:complexType name="CustomCommandAction">
 *    <xsd:simpleContent>
 *      <xsd:extension base="xsd:string">
 *        <xsd:attribute name="type" type="String" use="required"/>
 *      </xsd:extension>
 *    </xsd:simpleContent>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct CustomCommandAction : private Scope
{
  const String type;

  const String content;

private:
  const std::shared_ptr<CustomCommand> command;

public:
  explicit CustomCommandAction(const pugi::xml_node &, const Scope &);

  auto accomplished() noexcept -> bool { return command->accomplished(); }

  auto endsImmediately() const -> bool { return command->endsImmediately(); }

  auto run() noexcept -> void { return command->run(); }

  auto start() -> void { command->start(local()); }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_
