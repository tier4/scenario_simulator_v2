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

#ifndef SCENARIO_RUNNER__SYNTAX__COLLISION_CONDITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__COLLISION_CONDITION_HPP_

#include <scenario_runner/syntax/entity_ref.hpp>
#include <scenario_runner/syntax/triggering_entities.hpp>

#include <utility>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== CollisionCondition ===================================================
 *
 * <xsd:complexType name="CollisionCondition">
 *   <xsd:choice>
 *     <xsd:element name="EntityRef" type="EntityRef"/>
 *     <xsd:element name="ByType" type="ByObjectType"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct CollisionCondition
  : public Element
{
  template<typename Node, typename Scope>
  explicit CollisionCondition(const Node & node, Scope & scope, const TriggeringEntities &)
  : Element(
      choice(
        node,
        std::make_pair("EntityRef", [&](auto && node) {
          return make<EntityRef>(node, scope);
        }),
        std::make_pair("ByType", UNSUPPORTED())))
  {}

  auto evaluate() const noexcept
  {
    return false_v;
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__COLLISION_CONDITION_HPP_
