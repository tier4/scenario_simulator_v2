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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_DISTANCE_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_DISTANCE_CONDITION_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/relative_distance_type.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>

#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- RelativeDistanceCondition ----------------------------------------------
 *
 *  <xsd:complexType name="RelativeDistanceCondition">
 *    <xsd:attribute name="entityRef" type="String" use="required"/>
 *    <xsd:attribute name="relativeDistanceType" type="RelativeDistanceType" use="required"/>
 *    <xsd:attribute name="value" type="Double" use="required"/>
 *    <xsd:attribute name="freespace" type="Boolean" use="required"/>
 *    <xsd:attribute name="rule" type="Rule" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct RelativeDistanceCondition
{
  const String entity_ref;

  const RelativeDistanceType relative_distance_type;

  const Double value;

  /* ---- freespace ------------------------------------------------------------
   *
   *  True: distance is measured between closest bounding box points.
   *  False: reference point distance is used.
   *
   * ------------------------------------------------------------------------ */
  const Boolean freespace;

  const Rule compare;

  template<typename Node, typename Scope>
  explicit RelativeDistanceCondition(
    const Node & node, Scope & outer_scope, const TriggeringEntities & triggering_entities)
  : entity_ref(readAttribute<String>("entityRef", node, outer_scope)),
    relative_distance_type(
      readAttribute<RelativeDistanceType>("relativeDistanceType", node, outer_scope)),
    value(readAttribute<Double>("value", node, outer_scope)),
    freespace(readAttribute<Boolean>("freespace", node, outer_scope)),
    compare(readAttribute<Rule>("rule", node, outer_scope)),
    for_each(triggering_entities)
  {}

  const TriggeringEntities for_each;

  using TriggeringEntity = TriggeringEntities::value_type;

  auto distance(const TriggeringEntity & triggering_entity)
  {
    if (freespace) {
      switch (relative_distance_type) {
        case RelativeDistanceType::cartesianDistance:
          return getBoundingBoxDistance(entity_ref, triggering_entity);
        default:
          THROW(ImplementationFault);
      }
    } else {
      switch (relative_distance_type) {
        case RelativeDistanceType::longitudinal:
          return std::abs(getRelativePose(triggering_entity, entity_ref).position.x);
        case RelativeDistanceType::lateral:
          return std::abs(getRelativePose(triggering_entity, entity_ref).position.y);
        case RelativeDistanceType::cartesianDistance:
          return std::hypot(
            getRelativePose(triggering_entity, entity_ref).position.x,
            getRelativePose(triggering_entity, entity_ref).position.y);
        default:
          THROW(ImplementationFault);
      }
    }
  }

  template<typename ... Ts>
  auto operator()(Ts && ... xs)
  {
    return compare(distance(std::forward<decltype(xs)>(xs)...), value);
  }

  auto evaluate()
  {
    #ifndef NDEBUG
    std::cout << (indent++) << "- BEC.RDC:\n";
    #endif

    const auto result = asBoolean(
      for_each(
        [&](auto && triggering_entity)
        {
          const auto result = (*this)(triggering_entity);
          #ifndef NDEBUG
          std::cout << indent << "  " << triggering_entity << ": ";
          std::cout << "distance = " << distance(triggering_entity);
          std::cout << " " << compare << " " << value << "? => " << result << std::endl;
          #endif
          return result;
        }));

    #ifndef NDEBUG
    --indent;
    #endif

    return result;
  }
};
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_DISTANCE_CONDITION_HPP_
