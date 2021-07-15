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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__BY_VALUE_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__BY_VALUE_CONDITION_HPP_

#include <openscenario_interpreter/syntax/parameter_condition.hpp>
#include <openscenario_interpreter/syntax/simulation_time_condition.hpp>
#include <openscenario_interpreter/syntax/storyboard_element_state_condition.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_condition.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_controller_condition.hpp>
#include <openscenario_interpreter/syntax/user_defined_value_condition.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ByValueCondition -------------------------------------------------------
 *
 *  <xsd:complexType name="ByValueCondition">
 *    <xsd:choice>
 *      <xsd:element name="ParameterCondition" type="ParameterCondition"/>
 *      <xsd:element name="TimeOfDayCondition" type="TimeOfDayCondition"/>
 *      <xsd:element name="SimulationTimeCondition" type="SimulationTimeCondition"/>
 *      <xsd:element name="StoryboardElementStateCondition" type="StoryboardElementStateCondition"/>
 *      <xsd:element name="UserDefinedValueCondition" type="UserDefinedValueCondition"/>
 *      <xsd:element name="TrafficSignalCondition" type="TrafficSignalCondition"/>
 *      <xsd:element name="TrafficSignalControllerCondition" type="TrafficSignalControllerCondition"/>
 *    </xsd:choice>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ByValueCondition : public ComplexType
{
  template <typename Node, typename... Ts>
  explicit ByValueCondition(const Node & node, Ts &&... xs)
  // clang-format off
  : ComplexType(
      choice(node,
        std::make_pair(              "ParameterCondition", [&](auto && node) { return make<              ParameterCondition>(node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair(              "TimeOfDayCondition", [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
        std::make_pair(         "SimulationTimeCondition", [&](auto && node) { return make<         SimulationTimeCondition>(node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair( "StoryboardElementStateCondition", [&](auto && node) { return make< StoryboardElementStateCondition>(node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair(       "UserDefinedValueCondition", [&](auto && node) { return make<       UserDefinedValueCondition>(node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair(          "TrafficSignalCondition", [&](auto && node) { return make<          TrafficSignalCondition>(node, std::forward<decltype(xs)>(xs)...); }),
        std::make_pair("TrafficSignalControllerCondition", [&](auto && node) { return make<TrafficSignalControllerCondition>(node, std::forward<decltype(xs)>(xs)...); })))
  // clang-format on
  {
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__BY_VALUE_CONDITION_HPP_
