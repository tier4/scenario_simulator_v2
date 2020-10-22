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

#ifndef OPEN_SCENARIO_INTERPRETER__SYNTAX__RELATIVE_DISTANCE_CONDITION_HPP_
#define OPEN_SCENARIO_INTERPRETER__SYNTAX__RELATIVE_DISTANCE_CONDITION_HPP_

#include <open_scenario_interpreter/procedure.hpp>
#include <open_scenario_interpreter/syntax/relative_distance_type.hpp>
#include <open_scenario_interpreter/syntax/triggering_entities.hpp>

namespace open_scenario_interpreter
{
inline namespace syntax
{
/* ---- RelativeDistanceCondition ----------------------------------------------
 *
 * <xsd:complexType name="RelativeDistanceCondition">
 *   <xsd:attribute name="entityRef" type="String" use="required"/>
 *   <xsd:attribute name="relativeDistanceType" type="RelativeDistanceType" use="required"/>
 *   <xsd:attribute name="value" type="Double" use="required"/>
 *   <xsd:attribute name="freespace" type="Boolean" use="required"/>
 *   <xsd:attribute name="rule" type="Rule" use="required"/>
 * </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct RelativeDistanceCondition
{
  const String entity_ref;

  const RelativeDistanceType relative_distance_type;

  const Double value;

  const Boolean freespace;

  const Rule compare;

  const TriggeringEntities verify;

  template<typename Node, typename Scope>
  explicit RelativeDistanceCondition(
    const Node & node, Scope & outer_scope, const TriggeringEntities & triggering_entities)
  : entity_ref(readAttribute<String>("entityRef", node, outer_scope)),
    relative_distance_type(
      readAttribute<RelativeDistanceType>("relativeDistanceType", node, outer_scope)),
    value(readAttribute<Double>("value", node, outer_scope)),
    freespace(readAttribute<Boolean>("freespace", node, outer_scope)),
    compare(readAttribute<Rule>("rule", node, outer_scope)),
    verify(triggering_entities)
  {}

  auto evaluate()
  {
    switch (relative_distance_type) {
      case RelativeDistanceType::longitudinal:

        return asBoolean(
          verify([&](auto && subject)
          {
            const auto distance {
              std::fabs(getRelativePose(subject, entity_ref).position.x)
            };
            #ifndef NDEBUG
            std::cout << "DISTANCE: " << distance << std::endl;
            #endif
            return compare(distance, value);
          }));

      case RelativeDistanceType::lateral:

        return asBoolean(
          verify([&](auto && subject)
          {
            const auto distance {
              std::fabs(getRelativePose(subject, entity_ref).position.y)
            };
            #ifndef NDEBUG
            std::cout << "DISTANCE: " << distance << std::endl;
            #endif
            return compare(distance, value);
          }));

      case RelativeDistanceType::cartesianDistance:

        return asBoolean(
          verify([&](auto && subject)
          {
            const auto distance {
              std::hypot(
                getRelativePose(subject, entity_ref).position.x,
                getRelativePose(subject, entity_ref).position.y)
            };
            #ifndef NDEBUG
            std::cout << "DISTANCE: " << distance << std::endl;
            #endif
            return compare(distance, value);
          }));

      default:
        THROW(ImplementationFault);
    }
  }
};
}
}  // namespace open_scenario_interpreter

#endif  // OPEN_SCENARIO_INTERPRETER__SYNTAX__RELATIVE_DISTANCE_CONDITION_HPP_
