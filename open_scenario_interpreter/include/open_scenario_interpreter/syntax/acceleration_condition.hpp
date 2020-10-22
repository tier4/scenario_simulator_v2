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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__ACCELERATION_CONDITION_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__ACCELERATION_CONDITION_HPP_

#include <open_scenario_interpreter/procedure.hpp>
#include <open_scenario_interpreter/syntax/rule.hpp>
#include <open_scenario_interpreter/syntax/triggering_entities.hpp>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ---- AccelerationCondition --------------------------------------------------
 *
 * <xsd:complexType name="AccelerationCondition">
 *   <xsd:attribute name="value" type="Double" use="required"/>
 *   <xsd:attribute name="rule" type="Rule" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct AccelerationCondition
{
  const Double value;

  const Rule compare;

  const TriggeringEntities trigger;

  template<typename Node>
  explicit AccelerationCondition(
    const Node & node, Scope & outer_scope, const TriggeringEntities & trigger)
  : value(readAttribute<Double>("value", node, outer_scope)),
    compare(readAttribute<Rule>("rule", node, outer_scope)),
    trigger(trigger)
  {}

  auto evaluate() const
  {
    return asBoolean(
      trigger([&](auto && entity)
      {
        return compare(getEntityStatus(entity).accel.linear.x, value);
      }));
  }
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__ACCELERATION_CONDITION_HPP_
