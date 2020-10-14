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

#ifndef SCENARIO_RUNNER__SYNTAX__RELATIVE_DISTANCE_CONDITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__RELATIVE_DISTANCE_CONDITION_HPP_

#include <scenario_runner/syntax/relative_distance_type.hpp>
#include <scenario_runner/syntax/triggering_entities.hpp>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== RelativeDistanceCondition ============================================
 *
 * <xsd:complexType name="RelativeDistanceCondition">
 *   <xsd:attribute name="entityRef" type="String" use="required"/>
 *   <xsd:attribute name="relativeDistanceType" type="RelativeDistanceType" use="required"/>
 *   <xsd:attribute name="value" type="Double" use="required"/>
 *   <xsd:attribute name="freespace" type="Boolean" use="required"/>
 *   <xsd:attribute name="rule" type="Rule" use="required"/>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct RelativeDistanceCondition
{
  const String entity_ref;

  const RelativeDistanceType relative_distance_type;

  const Double value;

  const Boolean freespace;

  const Rule compare;

  Scope inner_scope;

  const TriggeringEntities verify;

  template<typename Node, typename Scope>
  explicit RelativeDistanceCondition(
    const Node & node, Scope & outer_scope,
    const TriggeringEntities & triggering_entities)
  : entity_ref{readAttribute<String>("entityRef", node, outer_scope)},
    relative_distance_type{
      readAttribute<RelativeDistanceType>("relativeDistanceType", node, outer_scope)},
    value{readAttribute<Double>("value", node, outer_scope)},
    freespace{readAttribute<Boolean>("freespace", node, outer_scope)},
    compare{readAttribute<Rule>("rule", node, outer_scope)},
    inner_scope{outer_scope},
    verify{triggering_entities}
  {}

  auto evaluate()
  {
    // switch (relative_distance_type)
    // {
    // case RelativeDistanceType::longitudinal:
    //
    //   return
    //     asBoolean(
    //       verify([&](auto&& subject)
    //       {
    //         const auto distance { std::fabs(inner_scope.connection->entity->getRelativePose(
    //         subject, entity_ref).position.x) };
    //         std::cout << "DISTANCE: " << distance << std::endl;
    //         return compare(distance, value);
    //       }));
    //
    // case RelativeDistanceType::lateral:
    //
    //   return
    //     asBoolean(
    //       verify([&](auto&& subject)
    //       {
    //         const auto distance { std::fabs(inner_scope.connection->entity->getRelativePose(
    //         subject, entity_ref).position.y) };
    //         std::cout << "DISTANCE: " << distance << std::endl;
    //         return compare(distance, value);
    //       }));
    //
    // case RelativeDistanceType::cartesianDistance:
    //
    //   return
    //     asBoolean(
    //       verify([&](auto&& subject)
    //       {
    //         const auto distance {
    //           std::hypot(
    //             inner_scope.connection->entity->getRelativePose(subject, entity_ref).position.x,
    //             inner_scope.connection->entity->getRelativePose(subject, entity_ref).position.y)
    //         };
    //         std::cout << "DISTANCE: " << distance << std::endl;
    //         return compare(distance, value);
    //       }));
    //
    // default:
    //   THROW(ImplementationFault);
    // }

    return false_v;
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__RELATIVE_DISTANCE_CONDITION_HPP_
