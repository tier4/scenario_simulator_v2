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

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>
#include <utility>

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
struct CollisionCondition
{
  const Element another_given_entity;

  const TriggeringEntities triggering_entities;

  template <typename Node, typename Scope>
  explicit CollisionCondition(
    const Node & node, Scope & scope, const TriggeringEntities & triggering_entities)
  // clang-format off
  : another_given_entity(
      choice(node,
        std::make_pair("EntityRef", [&](auto && node) { return make<EntityRef>(node, scope); }),
        std::make_pair("ByType",    [&](auto && node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }))),
    triggering_entities(triggering_entities)
  // clang-format on
  {
  }

  auto description() const
  {
    std::stringstream description;

    description << triggering_entities.description() << " colliding with another given entity "
                << another_given_entity << "?";

    // TODO (yamacir-kit): If another_given_entity.is<ByType>(), description
    // will be "Is any of [A, B, C] colliding with another T typed entities?"

    return description.str();
  }

  auto evaluate() const noexcept
  {
    if (another_given_entity.is<EntityRef>()) {
      return asBoolean(triggering_entities.apply([&](auto && triggering_entity) {
        return evaluateCollisionCondition(triggering_entity, another_given_entity.as<EntityRef>());
      }));
    } else {
      // TODO ByType
      return false_v;
    }
  }
};

std::ostream & operator<<(std::ostream &, const CollisionCondition &);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__COLLISION_CONDITION_HPP_
