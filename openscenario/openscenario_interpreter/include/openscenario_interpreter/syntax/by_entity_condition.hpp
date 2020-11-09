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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__BY_ENTITY_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__BY_ENTITY_CONDITION_HPP_

#include <openscenario_interpreter/syntax/entity_condition.hpp>

#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ByEntityCondition ------------------------------------------------------
 *
 * <xsd:complexType name="ByEntityCondition">
 *   <xsd:all>
 *     <xsd:element name="TriggeringEntities" type="TriggeringEntities"/>
 *     <xsd:element name="EntityCondition" type="EntityCondition"/>
 *   </xsd:all>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ByEntityCondition
{
  Scope inner_scope;

  const EntityCondition entity_condition;

  template<typename Node>
  explicit ByEntityCondition(const Node & node, Scope & outer_scope)
  : inner_scope(outer_scope),
    entity_condition(
      readElement<EntityCondition>(
        "EntityCondition", node, inner_scope,
        readElement<TriggeringEntities>("TriggeringEntities", node, inner_scope)))
  {}

  template<typename ... Ts>
  auto evaluate(Ts && ... xs) const
  {
    return entity_condition.evaluate(std::forward<decltype(xs)>(xs)...);
  }
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__BY_ENTITY_CONDITION_HPP_
