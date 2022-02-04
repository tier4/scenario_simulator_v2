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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_

#include <autoware_simulation_msgs/msg/simulation_events.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <pugixml.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
template <int Value>
struct SpecialAction : public std::integral_constant<int, Value>
{
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

  explicit CustomCommandAction(const pugi::xml_node &, const Scope &);

  static auto accomplished() noexcept -> bool;

  static auto applyFaultInjectionAction(const std::vector<std::string> &, const Scope &) -> int;

  static auto applyWalkStraightAction(const std::vector<std::string> &, const Scope &) -> int;

  static auto debugError(const std::vector<std::string> &, const Scope &) -> int;

  static auto debugSegmentationFault(const std::vector<std::string> &, const Scope &) -> int;

  static auto exitFailure(const std::vector<std::string> &, const Scope &) -> int;

  static auto exitSuccess(const std::vector<std::string> &, const Scope &) -> int;

  static auto node() -> rclcpp::Node &;

  static auto run() noexcept -> void;

  static auto publisher() -> rclcpp::Publisher<autoware_simulation_msgs::msg::SimulationEvents> &;

  static auto split(const std::string &) -> std::vector<std::string>;

  /*  */ auto start() const -> void;

  static auto test(const std::vector<std::string> &, const Scope &) -> int;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__CUSTOM_COMMAND_ACTION_HPP_
