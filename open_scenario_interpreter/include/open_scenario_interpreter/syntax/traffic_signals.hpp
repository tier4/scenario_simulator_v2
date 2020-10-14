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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNALS_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNALS_HPP_

#include <open_scenario_interpreter/reader/attribute.hpp>
#include <open_scenario_interpreter/reader/element.hpp>

#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== TrafficSignalState ===================================================
 *
 * <xsd:complexType name="TrafficSignalState">
 *   <xsd:attribute name="trafficSignalId" type="String" use="required"/>
 *   <xsd:attribute name="state" type="String" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct TrafficSignalState
{
  const String traffic_signal_id, state;

  template<typename Node, typename Scope>
  explicit TrafficSignalState(const Node & node, Scope & scope)
  : traffic_signal_id{readAttribute<String>("trafficSignalId", node, scope)},
    state{readAttribute<String>("state", node, scope)}
  {}
};

/* ==== Phase ================================================================
 *
 * <xsd:complexType name="Phase">
 *   <xsd:sequence>
 *     <xsd:element name="TrafficSignalState" minOccurs="0" maxOccurs="unbounded" type="TrafficSignalState"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="name" type="String" use="required"/>
 *   <xsd:attribute name="duration" type="Double" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct Phase
{
  const String name;

  const Double duration;

  const TrafficSignalState state;

  template<typename Node, typename Scope>
  explicit Phase(const Node & node, Scope & outer_scope)
  : name{readAttribute<String>("name", node, outer_scope)},
    duration{
      readAttribute<Double>(
        "duration", node, outer_scope, Double::infinity())},
    state{readElement<TrafficSignalState>("TrafficSignalState", node, outer_scope)}
  {}
};

/* ==== TrafficSignalController ================================================
 *
 * <xsd:complexType name="TrafficSignalController">
 *   <xsd:sequence>
 *     <xsd:element name="Phase" minOccurs="0" maxOccurs="unbounded" type="Phase"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="name" type="String" use="required"/>
 *   <xsd:attribute name="delay" type="Double" use="optional"/>
 *   <xsd:attribute name="reference" type="String" use="optional"/>
 * </xsd:complexType>
 *
 * ========================================================================== */
struct TrafficSignalController
{
  const String name;

  const Double delay;

  const String reference;

  const Phase phase;

  template<typename Node, typename Scope>
  explicit TrafficSignalController(const Node & node, Scope & outer_scope)
  : name{readAttribute<String>("name", node, outer_scope)},
    delay{readAttribute<Double>("delay", node, outer_scope)},
    reference{readAttribute<String>("reference", node, outer_scope)},
    phase{readElement<Phase>("Phase", node, outer_scope)}
  {}
};

/* ==== TrafficSignals =========================================================
 *
 * <xsd:complexType name="TrafficSignals">
 *   <xsd:sequence>
 *     <xsd:element name="TrafficSignalController" minOccurs="0" maxOccurs="unbounded" type="TrafficSignalController"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * ========================================================================== */
struct TrafficSignals
  : public std::vector<TrafficSignalController>
{
  TrafficSignals() = default;

  template<typename Node, typename Scope>
  explicit TrafficSignals(const Node & node, Scope & outer_scope)
  {
    callWithElements(
      node, "TrafficSignalController", 0, unbounded, [&](auto && node)
      {
        emplace_back(node, outer_scope);
      });
  }
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNALS_HPP_
