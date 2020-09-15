// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SCENARIO_RUNNER__SYNTAX__BY_VALUE_CONDITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__BY_VALUE_CONDITION_HPP_

#include <scenario_runner/syntax/simulation_time_condition.hpp>
#include <scenario_runner/syntax/storyboard_element_state_condition.hpp>
#include <scenario_runner/syntax/traffic_signal_condition.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== ByValueCondition =====================================================
 *
 * <xsd:complexType name="ByValueCondition">
 *   <xsd:choice>
 *     <xsd:element name="ParameterCondition" type="ParameterCondition"/>
 *     <xsd:element name="TimeOfDayCondition" type="TimeOfDayCondition"/>
 *     <xsd:element name="SimulationTimeCondition" type="SimulationTimeCondition"/>
 *     <xsd:element name="StoryboardElementStateCondition" type="StoryboardElementStateCondition"/>
 *     <xsd:element name="UserDefinedValueCondition" type="UserDefinedValueCondition"/>
 *     <xsd:element name="TrafficSignalCondition" type="TrafficSignalCondition"/>
 *     <xsd:element name="TrafficSignalControllerCondition" type="TrafficSignalControllerCondition"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct ByValueCondition
  : public Object
{
  template<typename Node, typename Scope>
  explicit ByValueCondition(const Node & node, Scope & scope)
  {
    callWithElements(node, "ParameterCondition", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "TimeOfDayCondition", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "SimulationTimeCondition", 0, 1, [&](auto && node)
      {
        return rebind<SimulationTimeCondition>(node, scope);
      });

    callWithElements(node, "StoryboardElementStateCondition", 0, 1, [&](auto && node)
      {
        return rebind<StoryboardElementStateCondition>(node, scope);
      });

    callWithElements(node, "UserDefinedValueCondition", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "TrafficSignalCondition", 0, 1, [&](auto && node)
      {
        return rebind<TrafficSignalCondition>(node, scope);
      });

    callWithElements(node, "TrafficSignalControllerCondition", 0, 1, THROW_UNSUPPORTED_ERROR(node));
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__BY_VALUE_CONDITION_HPP_
