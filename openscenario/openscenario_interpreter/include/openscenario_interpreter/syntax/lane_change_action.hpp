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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__LANE_CHANGE_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__LANE_CHANGE_ACTION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/boolean.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/entity.hpp>
#include <openscenario_interpreter/syntax/lane_change_target.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/transition_dynamics.hpp>
#include <pugixml.hpp>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- LaneChangeAction -------------------------------------------------------
 *
 *  <xsd:complexType name="LaneChangeAction">
 *    <xsd:all>
 *      <xsd:element name="LaneChangeActionDynamics" type="TransitionDynamics"/>
 *      <xsd:element name="LaneChangeTarget" type="LaneChangeTarget"/>
 *    </xsd:all>
 *    <xsd:attribute name="targetLaneOffset" type="Double" use="optional"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct LaneChangeAction : private Scope,
                          private SimulatorCore::ActionApplication,
                          private SimulatorCore::NonStandardOperation
{
  const Double target_lane_offset;

  const TransitionDynamics lane_change_action_dynamics;

  const LaneChangeTarget lane_change_target;

  explicit LaneChangeAction(const pugi::xml_node &, Scope &);

  std::unordered_map<Entity, Boolean> accomplishments;

  /*  */ auto accomplished() -> bool;

  static auto endsImmediately() noexcept -> bool;

  static auto run() noexcept -> void;

  /*  */ auto start() -> void;

  explicit operator traffic_simulator::lane_change::AbsoluteTarget() const;

  explicit operator traffic_simulator::lane_change::RelativeTarget() const;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__LANE_CHANGE_ACTION_HPP_
