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

#ifndef SCENARIO_RUNNER__SYNTAX__TRAFFIC_SIGNALS_HPP_
#define SCENARIO_RUNNER__SYNTAX__TRAFFIC_SIGNALS_HPP_

#include <scenario_runner/validator/attribute.hpp>
#include <scenario_runner/validator/sequence.hpp>

namespace scenario_runner
{inline namespace syntax
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
  const std::string traffic_signal_id, state;

  template<typename ... Ts>
  explicit TrafficSignalState(const pugi::xml_node & node, Ts && ...)
  : traffic_signal_id{readRequiredAttribute<std::string>(node, "trafficSignalId")},
    state{readRequiredAttribute<std::string>(node, "state")}
  {}

  decltype(auto) evaluate() const noexcept
  {
    return unspecified;
  }
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
  : public Sequence
{
  const std::string name;
  const double duration;

  template<typename ... Ts>
  explicit Phase(const pugi::xml_node & node, Ts && ... xs)
  : name{readRequiredAttribute<std::string>(node, "name")},
    duration{readUnsupportedAttribute<double>(node, "duration",
        std::numeric_limits<double>::infinity())}
  {
    defineElement<TrafficSignalState>("TrafficSignalState", 0, unbounded);

    validate(node, std::forward<decltype(xs)>(xs)...);
  }

  decltype(auto) evaluate() const noexcept
  {
    return unspecified;
  }
};

/* ==== TrafficSignalController =======================================================
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
 * ======================================================================== */
struct TrafficSignalController
  : public Sequence
{
  const std::string name;
  const double delay;
  const std::string reference;

  template<typename ... Ts>
  explicit TrafficSignalController(const pugi::xml_node & node, Ts && ... xs)
  : name{readRequiredAttribute<std::string>(node, "name")},
    delay{readUnsupportedAttribute<double>(node, "delay")},
    reference{readUnsupportedAttribute<std::string>(node, "reference")}
  {
    defineElement<Phase>("Phase", 0, unbounded);

    validate(node, std::forward<decltype(xs)>(xs)...);
  }

  decltype(auto) evaluate() const noexcept
  {
    return unspecified;
  }
};

/* ==== TrafficSignals =======================================================
 *
 * <xsd:complexType name="TrafficSignals">
 *   <xsd:sequence>
 *     <xsd:element name="TrafficSignalController" minOccurs="0" maxOccurs="unbounded" type="TrafficSignalController"/>
 *   </xsd:sequence>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct TrafficSignals
  : public std::vector<TrafficSignalController>
{
  explicit TrafficSignals() = default;

  template<typename Node, typename Scope>
  explicit TrafficSignals(const Node & node, Scope & outer_scope)
  {
    callWithElements(node, "TrafficSignalController", 0, unbounded, [&](auto && node)
      {
        emplace_back(node, outer_scope);
      });
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__TRAFFIC_SIGNALS_HPP_
