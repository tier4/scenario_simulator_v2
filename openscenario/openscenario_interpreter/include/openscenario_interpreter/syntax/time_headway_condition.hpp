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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__TIME_HEADWAY_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__TIME_HEADWAY_CONDITION_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>
#include <openscenario_interpreter/utility/print.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- TimeHeadwayCondition ---------------------------------------------------
 *
 *  Condition based on the headway time between a triggering entity/entities
 *  and a reference entity. The logical operator used for comparison is defined
 *  by the rule attribute.
 *
 *  <xsd:complexType name="TimeHeadwayCondition">
 *    <xsd:attribute name="entityRef" type="String" use="required"/>
 *    <xsd:attribute name="value" type="Double" use="required"/>
 *    <xsd:attribute name="freespace" type="Boolean" use="required"/>
 *    <xsd:attribute name="alongRoute" type="Boolean" use="required"/>
 *    <xsd:attribute name="rule" type="Rule" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct TimeHeadwayCondition
{
  const String entity_ref;

  const Double value;

  const Boolean freespace;

  const Boolean along_route;

  const Rule compare;

  const TriggeringEntities triggering_entities;

  std::vector<Double> last_checked_values;  // for description

  template <typename Node, typename Scope>
  explicit TimeHeadwayCondition(
    const Node & node, Scope & outer_scope, const TriggeringEntities & triggering_entities)
  // clang-format off
  : entity_ref (readAttribute<String> ("entityRef",  node, outer_scope)),
    value      (readAttribute<Double> ("value",      node, outer_scope)),
    freespace  (readAttribute<Boolean>("freespace",  node, outer_scope)),
    along_route(readAttribute<Boolean>("alongRoute", node, outer_scope)),
    compare    (readAttribute<Rule>   ("rule",       node, outer_scope)),
    triggering_entities(triggering_entities),
    last_checked_values(triggering_entities.entity_refs.size(), Double::nan())
  // clang-format on
  {
  }

  auto description() const
  {
    std::stringstream description;

    description << triggering_entities.description()
                << "'s headway time between each and the referenced entity " << entity_ref << " = ";

    print_to(description, last_checked_values);

    description << " " << compare << " " << value << "?";

    return description.str();
  }

  auto evaluate()
  {
    last_checked_values.clear();

    return asBoolean(triggering_entities.apply([&](auto && triggering_entity) {
      last_checked_values.push_back(getTimeHeadway(triggering_entity, entity_ref));
      return compare(last_checked_values.back(), value);
    }));
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__TIME_HEADWAY_CONDITION_HPP_
