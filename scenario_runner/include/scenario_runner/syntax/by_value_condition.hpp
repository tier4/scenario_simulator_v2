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
#include <scenario_runner/syntax/parameter_condition.hpp>

#include <utility>

namespace scenario_runner
{
inline namespace syntax
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
  : public Element
{
  template<typename Node, typename ... Ts>
  explicit ByValueCondition(const Node & node, Ts && ... xs)
  : Element(
      choice(
        node,

        std::make_pair("ParameterCondition", [&](auto && node)
        {
          return make<ParameterCondition>(node, std::forward<decltype(xs)>(xs)...);
        }),

        std::make_pair("TimeOfDayCondition", UNSUPPORTED()),

        std::make_pair("SimulationTimeCondition", [&](auto && node)
        {
          return make<SimulationTimeCondition>(node, std::forward<decltype(xs)>(xs)...);
        }),

        std::make_pair("StoryboardElementStateCondition", [&](auto && node)
        {
          return make<StoryboardElementStateCondition>(node, std::forward<decltype(xs)>(xs)...);
        }),

        std::make_pair("UserDefinedValueCondition", UNSUPPORTED()),

        std::make_pair("TrafficSignalCondition", [&](auto && node)
        {
          return make<TrafficSignalCondition>(node, std::forward<decltype(xs)>(xs)...);
        }),

        std::make_pair("TrafficSignalControllerCondition", UNSUPPORTED())))
  {}
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__BY_VALUE_CONDITION_HPP_
