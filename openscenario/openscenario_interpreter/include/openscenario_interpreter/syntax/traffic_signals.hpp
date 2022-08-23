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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNALS_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNALS_HPP_

#include <memory>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_controller.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- TrafficSignals ---------------------------------------------------------
 *
 *  <xsd:complexType name="TrafficSignals">
 *    <xsd:sequence>
 *      <xsd:element name="TrafficSignalController" minOccurs="0" maxOccurs="unbounded" type="TrafficSignalController"/>
 *    </xsd:sequence>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
class TrafficSignals
{
  std::list<std::shared_ptr<TrafficSignalController>> traffic_signal_controllers;

  auto resolve_reference(Scope &) -> void;

public:
  TrafficSignals() = default;

  explicit TrafficSignals(const pugi::xml_node &, Scope &);

  auto evaluate() -> Object;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNALS_HPP_
