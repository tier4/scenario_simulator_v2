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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__REACH_POSITION_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__REACH_POSITION_CONDITION_HPP_

#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/position.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ReachPositionCondition -------------------------------------------------
 *
 *  <xsd:complexType name="ReachPositionCondition">
 *    <xsd:all>
 *      <xsd:element name="Position" type="Position"/>
 *    </xsd:all>
 *    <xsd:attribute name="tolerance" type="Double" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ReachPositionCondition
{
  const Double tolerance;

  const Position position;

  const Rule compare;

  const TriggeringEntities triggering_entities;

  std::vector<Double> results;  // for description

  template <typename Node, typename Scope>
  explicit ReachPositionCondition(
    const Node & node, Scope & outer_scope, const TriggeringEntities & triggering_entities)
  : tolerance(readAttribute<Double>("tolerance", node, outer_scope)),
    position(readElement<Position>("Position", node, outer_scope)),
    compare(Rule::lessThan),
    triggering_entities(triggering_entities),
    results(triggering_entities.entity_refs.size(), Double::nan())
  {
  }

  auto description() const -> String;

  auto evaluate() -> Element;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__REACH_POSITION_CONDITION_HPP_
