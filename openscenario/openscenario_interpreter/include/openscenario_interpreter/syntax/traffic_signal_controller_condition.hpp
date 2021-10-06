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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONTROLLER_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONTROLLER_CONDITION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_controller.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- TrafficSignalControllerCondition ---------------------------------------
 *
 *  Condition becomes true if the referenced signal controller reaches the
 *  indicated state.
 *
 *  <xsd:complexType name="TrafficSignalControllerCondition">
 *    <xsd:attribute name="trafficSignalControllerRef" type="String" use="required"/>
 *    <xsd:attribute name="phase" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct TrafficSignalControllerCondition
{
  // Name of the phase of the signal controller to be reached for the condition to become true. The available phases are defined in type RoadNetwork under the property trafficSignalControllers.
  const String phase;

  // ID of the referenced signal controller in a road network.
  const String traffic_signal_controller_ref;

  String current_phase_name;

  Double current_phase_since;

  Scope scope;

  explicit TrafficSignalControllerCondition(const pugi::xml_node &, const Scope &);

  auto description() const -> String;

  auto evaluate() -> Element;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_CONTROLLER_CONDITION_HPP_
