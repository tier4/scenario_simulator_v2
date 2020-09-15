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

#ifndef SCENARIO_RUNNER__SYNTAX__TRIGGERING_ENTITIES_HPP_
#define SCENARIO_RUNNER__SYNTAX__TRIGGERING_ENTITIES_HPP_

#include <scenario_runner/syntax/entity_ref.hpp>
#include <scenario_runner/syntax/triggering_entities_rule.hpp>

namespace scenario_runner
{inline namespace syntax
{
/* ==== TriggeringEntities ===================================================
 *
 * <xsd:complexType name="TriggeringEntities">
 *   <xsd:sequence>
 *     <xsd:element name="EntityRef" maxOccurs="unbounded" type="EntityRef"/>
 *   </xsd:sequence>
 *   <xsd:attribute name="triggeringEntitiesRule" type="TriggeringEntitiesRule" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct TriggeringEntities
  : public std::vector<EntityRef>
{
  const TriggeringEntitiesRule verify;

  template<typename Node, typename Scope>
  explicit TriggeringEntities(const Node & node, Scope & scope)
  : verify{readAttribute<TriggeringEntitiesRule>(node, scope, "triggeringEntitiesRule")}
  {
    callWithElements(node, "EntityRef", 1, unbounded, [&](auto && node)
      {
        emplace_back(node, scope);
      });
  }

  template<typename ... Ts>
  constexpr decltype(auto) operator()(Ts && ... xs) const
  {
    return verify(std::begin(*this), std::end(*this), std::forward<decltype(xs)>(xs)...);
  }
};
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__TRIGGERING_ENTITIES_HPP_
