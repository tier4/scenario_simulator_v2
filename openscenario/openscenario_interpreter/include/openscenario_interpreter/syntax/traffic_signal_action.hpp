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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_ACTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_controller_action.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_state_action.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- NOTE -------------------------------------------------------------------
 *
 *  <xsd:complexType name="TrafficSignalAction">
 *    <xsd:choice>
 *      <xsd:element name="TrafficSignalControllerAction" type="TrafficSignalControllerAction"/>
 *      <xsd:element name="TrafficSignalStateAction" type="TrafficSignalStateAction"/>
 *    </xsd:choice>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct TrafficSignalAction : public ComplexType
{
  explicit TrafficSignalAction(const pugi::xml_node & node, Scope & scope)
  // clang-format off
  : ComplexType(
      choice(node,
        std::make_pair("TrafficSignalControllerAction", [&](const auto & node) { return make<TrafficSignalControllerAction>(node, scope); }),
        std::make_pair("TrafficSignalStateAction",      [&](const auto & node) { return make<TrafficSignalStateAction     >(node, scope); })))
  // clang-format on
  {
  }

  auto endsImmediately() const -> bool;

  auto run() -> void;

  auto start() -> void;
};

DEFINE_LAZY_VISITOR(
  TrafficSignalAction,
  CASE(TrafficSignalStateAction),       //
  CASE(TrafficSignalControllerAction),  //
);

DEFINE_LAZY_VISITOR(
  const TrafficSignalAction,
  CASE(TrafficSignalStateAction),       //
  CASE(TrafficSignalControllerAction),  //
);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_ACTION_HPP_
