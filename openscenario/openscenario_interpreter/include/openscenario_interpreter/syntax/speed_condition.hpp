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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_CONDITION_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- SpeedCondition ---------------------------------------------------------
 *
 *  Compares a triggering entity's/entities' speed to a target speed. The
 *  logical operator for the comparison is given by the rule attribute.
 *
 *  <xsd:complexType name="SpeedCondition">
 *    <xsd:attribute name="value" type="Double" use="required"/>
 *    <xsd:attribute name="rule" type="Rule" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct SpeedCondition
{
  const Double value;

  const Rule compare;

  const TriggeringEntities triggering_entities;

  std::vector<Double> last_checked_values;  // for description

  template <typename AST, typename Scope>
  explicit SpeedCondition(
    const AST & node, Scope & outer_scope, const TriggeringEntities & triggering_entities)
  : value(readAttribute<Double>("value", node, outer_scope)),
    compare(readAttribute<Rule>("rule", node, outer_scope)),
    triggering_entities(triggering_entities),
    last_checked_values(triggering_entities.entity_refs.size(), Double::nan())
  {
  }

  auto description() const
  {
    std::stringstream description;

    description << triggering_entities.description() << "'s speed = ";

    print_to(description, last_checked_values);

    description << " " << compare << " " << value << "?";

    return description.str();
  }

  auto evaluate()
  {
    last_checked_values.clear();

    return asBoolean(triggering_entities.apply([&](auto && triggering_entity) {
      last_checked_values.push_back(
        getEntityStatus(triggering_entity).action_status.twist.linear.x);
      return compare(last_checked_values.back(), value);
    }));
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__SPEED_CONDITION_HPP_
