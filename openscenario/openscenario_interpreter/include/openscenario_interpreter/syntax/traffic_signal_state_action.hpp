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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_STATE_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_STATE_ACTION_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/string.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- NOTE -------------------------------------------------------------------
 *
 *  Controls the state of a traffic signal.
 *
 *  <xsd:complexType name="TrafficSignalStateAction">
 *    <xsd:attribute name="name" type="String" use="required"/>
 *    <xsd:attribute name="state" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct TrafficSignalStateAction : private Scope
{
  /* ---- NOTE -----------------------------------------------------------------
   *
   *  ID of a signal in a road network. The signal ID must be listed in the
   *  TrafficSignal list of the RoadNetwork.
   *
   * ------------------------------------------------------------------------ */
  const String name;

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  Targeted state of the signal. The available states are listed in the
   *  TrafficSignal list of the RoadNetwork.
   *
   * ------------------------------------------------------------------------ */
  const String state;

  template <typename Node>
  explicit TrafficSignalStateAction(const Node & node, Scope & current_scope)
  : Scope(current_scope),
    name(readAttribute<String>("name", node, static_cast<Scope &>(*this))),
    state(readAttribute<String>("state", node, static_cast<Scope &>(*this)))
  {
  }

  static auto accomplished() noexcept { return true; }

  auto start() const { return unspecified; }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_STATE_ACTION_HPP_
