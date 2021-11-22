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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_STATE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_STATE_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- TrafficSignalState -----------------------------------------------------
 *
 *  <xsd:complexType name="TrafficSignalState">
 *    <xsd:attribute name="trafficSignalId" type="String" use="required"/>
 *    <xsd:attribute name="state" type="String" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct TrafficSignalState
{
  using LaneletId = std::int64_t;

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  ID of the referenced signal in a road network. The signal ID must be
   *  listed in TrafficSignal list of the RoadNetwork.
   *
   *  In the TierIV OpenSCENARIO implementation, it is the Lanelet ID (positive
   *  integer) of the traffic light.
   *
   * ------------------------------------------------------------------------ */
  const String traffic_signal_id;

  /* ---- NOTE -----------------------------------------------------------------
   *
   *  State of the signal. The available states are listed in the TrafficSignal
   *  list of the RoadNetwork.
   *
   * ------------------------------------------------------------------------ */
  const String state;

  explicit TrafficSignalState(const pugi::xml_node &, Scope &);

  auto evaluate() const -> Object;

  auto id() const -> LaneletId;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_STATE_HPP_
