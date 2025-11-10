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

  // NOTE: If overwrite is false, it will keep the states already added
  //       and simply add the specified state.
  auto evaluate(bool overwrite = true) const -> Object;

  struct TrafficSignalType
  {
    enum value_type { conventional, v2i } value;

    explicit TrafficSignalType(value_type value) : value(value) {}

    explicit TrafficSignalType(const std::string &);

    constexpr operator value_type() const noexcept { return value; }
  };

  static auto parseTrafficSignalId(const std::string & traffic_signal_id)
    -> std::pair<lanelet::Id, TrafficSignalType>;

  auto id() const -> lanelet::Id { return parsed_traffic_signal_id.first; }

  auto trafficSignalType() const -> TrafficSignalType { return parsed_traffic_signal_id.second; }

private:
  const std::pair<lanelet::Id, TrafficSignalType> parsed_traffic_signal_id;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TRAFFIC_SIGNAL_STATE_HPP_
