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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__TIME_HEADWAY_CONDITION_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__TIME_HEADWAY_CONDITION_HPP_

#include <open_scenario_interpreter/accessor.hpp>
#include <open_scenario_interpreter/syntax/rule.hpp>
#include <open_scenario_interpreter/syntax/triggering_entities.hpp>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ==== TimeHeadwayCondition =================================================
 *
 * <xsd:complexType name="TimeHeadwayCondition">
 *   <xsd:attribute name="entityRef" type="String" use="required"/>
 *   <xsd:attribute name="value" type="Double" use="required"/>
 *   <xsd:attribute name="freespace" type="Boolean" use="required"/>
 *   <xsd:attribute name="alongRoute" type="Boolean" use="required"/>
 *   <xsd:attribute name="rule" type="Rule" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct TimeHeadwayCondition
{
  const String entity_ref;

  const Double value;

  const Boolean freespace;

  const Boolean along_route;

  const Rule compare;

  const TriggeringEntities trigger;

  template<typename Node>
  explicit TimeHeadwayCondition(
    const Node & node, Scope & outer_scope, const TriggeringEntities & trigger)
  : entity_ref(readAttribute<String>("entityRef", node, outer_scope)),
    value(readAttribute<Double>("value", node, outer_scope)),
    freespace(readAttribute<Boolean>("freespace", node, outer_scope)),
    along_route(readAttribute<Boolean>("alongRoute", node, outer_scope)),
    compare(readAttribute<Rule>("rule", node, outer_scope)),
    trigger(trigger)
  {}

  auto evaluate()
  {
    return asBoolean(
      trigger([&](auto && entity)
      {
        return compare(getTimeHeadway(entity, entity_ref), value);
      }));
  }
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__TIME_HEADWAY_CONDITION_HPP_
