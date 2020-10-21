// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__REACH_POSITION_CONDITION_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__REACH_POSITION_CONDITION_HPP_

#include <open_scenario_interpreter/accessor.hpp>
#include <open_scenario_interpreter/syntax/position.hpp>
#include <open_scenario_interpreter/syntax/triggering_entities.hpp>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ---- ReachPositionCondition -------------------------------------------------
 *
 * <xsd:complexType name="ReachPositionCondition">
 *   <xsd:all>
 *     <xsd:element name="Position" type="Position"/>
 *   </xsd:all>
 *   <xsd:attribute name="tolerance" type="Double" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ReachPositionCondition
  : private Accessor
{
  const Double tolerance;

  const Position position;

  const TriggeringEntities trigger;

  template<typename Node>
  explicit ReachPositionCondition(
    const Node & node, Scope & outer_scope, const TriggeringEntities & trigger)
  : tolerance(readAttribute<Double>("tolerance", node, outer_scope)),
    position(readElement<Position>("Position", node, outer_scope)),
    trigger(trigger)
  {}

  auto evaluate()
  {
    if (position.is<WorldPosition>()) {
      return asBoolean(
        trigger([&](auto && entity)
        {
          return isReachedPosition(entity, position.as<WorldPosition>(), tolerance);
        }));
    } else if (position.is<LanePosition>()) {
      return asBoolean(
        trigger([&](auto && entity)
        {
          return isReachedPosition(
            entity,
            static_cast<Integer>(position.as<LanePosition>().lane_id),
            position.as<LanePosition>().s,
            position.as<LanePosition>().offset,
            tolerance);
        }));
    } else {
      THROW(ImplementationFault);
    }
  }
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__REACH_POSITION_CONDITION_HPP_
