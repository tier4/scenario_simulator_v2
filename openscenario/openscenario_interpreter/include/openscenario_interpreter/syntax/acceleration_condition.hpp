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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ACCELERATION_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ACCELERATION_CONDITION_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>
#include <openscenario_interpreter/utility/print.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- AccelerationCondition --------------------------------------------------
 *
 *  <xsd:complexType name="AccelerationCondition">
 *    <xsd:attribute name="value" type="Double" use="required"/>
 *    <xsd:attribute name="rule" type="Rule" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct AccelerationCondition
{
  const Double value;

  const Rule compare;

  const TriggeringEntities triggering_entities;

  std::vector<double> last_checked_values;  // for description

  template <typename Node, typename Scope>
  explicit AccelerationCondition(
    const Node & node, Scope & outer_scope, const TriggeringEntities & triggering_entities)
  : value(readAttribute<Double>("value", node, outer_scope)),
    compare(readAttribute<Rule>("rule", node, outer_scope)),
    triggering_entities(triggering_entities),
    last_checked_values(triggering_entities.entity_refs.size(), Double::nan())
  {
  }

  auto description() const -> std::string;

  auto evaluate() -> Element;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ACCELERATION_CONDITION_HPP_
