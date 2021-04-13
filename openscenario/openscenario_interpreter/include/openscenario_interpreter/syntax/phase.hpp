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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__PHASE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__PHASE_HPP_

#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/traffic_signal_state.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- Phase ------------------------------------------------------------------
 *
 *  <xsd:complexType name="Phase">
 *    <xsd:sequence>
 *      <xsd:element name="TrafficSignalState" minOccurs="0" maxOccurs="unbounded" type="TrafficSignalState"/>
 *    </xsd:sequence>
 *    <xsd:attribute name="name" type="String" use="required"/>
 *    <xsd:attribute name="duration" type="Double" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct Phase
{
  /* ---- NOTE -----------------------------------------------------------------
   *
   *  Name of the phase.
   *
   * ------------------------------------------------------------------------ */
  const String name;

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  Duration of the phase. Unit: s; Range: [0..inf[.
   *
   * ------------------------------------------------------------------------ */
  const Double duration;

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  Each phase has multiple TrafficSignalStates. One for each TrafficSignal
   *  that is controlled.
   *
   *  E.g. phase1 (trafficSignal1: true; false; false,
   *               trafficSignal2: false; false; true).
   *
   * ------------------------------------------------------------------------ */
  const std::vector<TrafficSignalState> traffic_signal_states;

  template <typename Node, typename Scope>
  explicit Phase(const Node & node, Scope & outer_scope)
  : name(readAttribute<String>("name", node, outer_scope)),
    duration(readAttribute<Double>("duration", node, outer_scope, Double::infinity())),
    traffic_signal_states(
      readElements<TrafficSignalState, 0>("TrafficSignalState", node, outer_scope))
  {
  }

  auto evaluate() const
  {
    for (const auto & state : traffic_signal_states) {
      state.evaluate();
    }

    return unspecified;
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__PHASE_HPP_
