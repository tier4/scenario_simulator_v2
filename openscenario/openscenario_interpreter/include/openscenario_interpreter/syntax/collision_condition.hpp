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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__COLLISION_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__COLLISION_CONDITION_HPP_

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- CollisionCondition -----------------------------------------------------
 *
 *  Condition becomes true when the triggering entity/entities collide with
 *  another given entity or any entity of a specific type.
 *
 *  <xsd:complexType name="CollisionCondition">
 *    <xsd:choice>
 *      <xsd:element name="EntityRef" type="EntityRef"/>
 *      <xsd:element name="ByType" type="ByObjectType"/>
 *    </xsd:choice>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct CollisionCondition : private Scope
{
  const Object another_given_entity;

  const TriggeringEntities triggering_entities;

  explicit CollisionCondition(const pugi::xml_node &, Scope &, const TriggeringEntities &);

  auto description() const -> std::string;

  auto evaluate() const -> Object;
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__COLLISION_CONDITION_HPP_
