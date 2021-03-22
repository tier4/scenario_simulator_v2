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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_ACTION_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>
#include <openscenario_interpreter/syntax/speed_action_target.hpp>
#include <openscenario_interpreter/syntax/transition_dynamics.hpp>

#include <string>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- SpeedAction ------------------------------------------------------------
 *
 * <xsd:complexType name="SpeedAction">
 *   <xsd:all>
 *     <xsd:element name="SpeedActionDynamics" type="TransitionDynamics"/>
 *     <xsd:element name="SpeedActionTarget" type="SpeedActionTarget"/>
 *   </xsd:all>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct SpeedAction
{
  Scope inner_scope;

  const TransitionDynamics speed_action_dynamics;

  const SpeedActionTarget speed_action_target;

  template<typename Node>
  explicit SpeedAction(const Node & node, Scope & outer_scope)
  : inner_scope(outer_scope),
    speed_action_dynamics(
      readElement<TransitionDynamics>("SpeedActionDynamics", node, inner_scope)),
    speed_action_target(readElement<SpeedActionTarget>("SpeedActionTarget", node, inner_scope))
  {}

  std::unordered_map<String, Boolean> accomplishments;

  auto reset()
  {
    accomplishments.clear();
    for (const auto & actor : inner_scope.actors) {
      accomplishments.emplace(actor, false);
    }
  }

  template<typename T>
  decltype(auto) setLinearTransition(const String & actor, const T value) const
  {
    return setTargetSpeed(actor, value, true);
  }

  template<typename T>
  decltype(auto) setStepTransition(const String & actor, const T value) const
  {
    auto status = getEntityStatus(actor);
    status.action_status.twist.linear.x = value;
    setEntityStatus(actor, status);
    return setTargetSpeed(actor, status.action_status.twist.linear.x, true);
  }

  decltype(auto) request(const String & actor) const
  {
    if (speed_action_target.is<AbsoluteTargetSpeed>()) {
      switch (speed_action_dynamics.dynamics_shape) {
        case DynamicsShape::linear:
          return setLinearTransition(
            actor, speed_action_target.as<AbsoluteTargetSpeed>().value);
        case DynamicsShape::step:
          return setStepTransition(
            actor, speed_action_target.as<AbsoluteTargetSpeed>().value);
        default:
          THROW(ImplementationFault);
      }
    } else {
      THROW(ImplementationFault);
    }
  }

  auto start()
  {
    reset();

    for (const auto & actor : inner_scope.actors) {
      request(actor);
    }

    return unspecified;
  }

  auto check(const String & actor) try
  {
    const auto compare = Rule(Rule::equalTo);

    if (speed_action_target.is<AbsoluteTargetSpeed>()) {
      return compare(
        getEntityStatus(actor).action_status.twist.linear.x,
        speed_action_target.as<AbsoluteTargetSpeed>().value);
    } else {
      THROW(ImplementationFault);
    }
  } catch (const SemanticError &) {
    return false;  // NOTE: The actor is maybe lane-changing now.
  }

  auto update()
  {
    for (auto && each : accomplishments) {
      each.second = each.second || check(each.first);
    }
  }

  auto accomplished()
  {
    update();
    return std::all_of(std::begin(accomplishments), std::end(accomplishments), cdr);
  }
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_ACTION_HPP_
