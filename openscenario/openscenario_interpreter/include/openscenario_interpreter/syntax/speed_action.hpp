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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_ACTION_HPP_

#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/boolean.hpp>
#include <openscenario_interpreter/syntax/speed_action_target.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/transition_dynamics.hpp>
#include <pugixml.hpp>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- SpeedAction ------------------------------------------------------------
 *
 *  <xsd:complexType name="SpeedAction">
 *    <xsd:all>
 *      <xsd:element name="SpeedActionDynamics" type="TransitionDynamics"/>
 *      <xsd:element name="SpeedActionTarget" type="SpeedActionTarget"/>
 *    </xsd:all>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct SpeedAction : private Scope,  // NOTE: Required for access to actors
                     private SimulatorCore::ActionApplication,
                     private SimulatorCore::ConditionEvaluation
{
  const TransitionDynamics speed_action_dynamics;

  const SpeedActionTarget speed_action_target;

  explicit SpeedAction(const pugi::xml_node &, Scope &);

  std::unordered_map<Entity, Boolean> accomplishments;

  auto accomplished() -> bool;

  auto endsImmediately() const -> bool;

  auto run() -> void;

  auto start() -> void;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_ACTION_HPP_
