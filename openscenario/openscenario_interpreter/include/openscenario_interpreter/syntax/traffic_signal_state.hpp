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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_STATE_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_STATE_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   TrafficSignalState (OpenSCENARIO XML 1.3.1)

   State of a traffic signal for this phase. One state per phase and traffic signal.

   <xsd:complexType name="TrafficSignalState">
     <xsd:attribute name="state" type="String" use="required"/>
     <xsd:attribute name="trafficSignalId" type="String" use="required"/>
   </xsd:complexType>
*/
struct TrafficSignalState : private SimulatorCore::NonStandardOperation
{
  /* ---- NOTE -----------------------------------------------------------------
   *
   *  ID of the referenced signal in a road network. The signal ID must be
   *  listed in TrafficSignal list of the RoadNetwork.
   *
   *  In the TIER IV OpenSCENARIO implementation, it is the Lanelet ID (positive
   *  integer) of the traffic light, optionally followed by a space and the
   *  signal type ("v2i"). For example: "34802" or "34802 v2i".
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

  auto id() const -> lanelet::Id;

  enum class TrafficSignalType { CONVENTIONAL, V2I };

  auto traffic_signal_type() const -> TrafficSignalType;

  struct ParsedTrafficSignalID
  {
    lanelet::Id lanelet_id;

    TrafficSignalType traffic_signal_type;

    explicit ParsedTrafficSignalID(const String & traffic_signal_id);
  };

  const ParsedTrafficSignalID parsed_traffic_signal_id;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_STATE_HPP_
