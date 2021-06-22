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
 *  The current relative distance of a triggering entity/entities to a
 *  reference entity is compared to a given value. The logical operator used
 *  for comparison is defined in the rule attribute.
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
  /* ---- entityRef ------------------------------------------------------------
   *
   *  Reference entity.
   *
   * ------------------------------------------------------------------------ */
  const String entity_ref;

  /* ---- relativeDistanceType -------------------------------------------------
   *
   *  The domain the distance is calculated in.
   *
   * ------------------------------------------------------------------------ */
  const RelativeDistanceType relative_distance_type;

  /* ---- value ----------------------------------------------------------------
   *
   *  The distance value. Unit: m; Range: [0..inf[.
   *
   * ------------------------------------------------------------------------ */
  const Double value;

  /* ---- freespace ------------------------------------------------------------
   *
   *  True: distance is measured between the closest bounding box points.
   *  False: reference point distance is used.
   *
   * ------------------------------------------------------------------------ */
  const Boolean freespace;

  /* ---- rule -----------------------------------------------------------------
   *
   *  The operator (less, greater, equal).
   *
   * ------------------------------------------------------------------------ */
  const Rule compare;

  const TriggeringEntities triggering_entities;

  std::vector<Double> last_checked_values;  // for description

  template <typename Node, typename Scope>
  explicit RelativeDistanceCondition(
    const Node & node, Scope & outer_scope, const TriggeringEntities & triggering_entities)
  // clang-format off
  : entity_ref            (readAttribute<String>              ("entityRef",            node, outer_scope)),
    relative_distance_type(readAttribute<RelativeDistanceType>("relativeDistanceType", node, outer_scope)),
    value                 (readAttribute<Double>              ("value",                node, outer_scope)),
    freespace             (readAttribute<Boolean>             ("freespace",            node, outer_scope)),
    compare               (readAttribute<Rule>                ("rule",                 node, outer_scope)),
    triggering_entities(triggering_entities),
    last_checked_values(triggering_entities.entity_refs.size(), Double::nan())
  // clang-format on
  {
  }

  auto description() const
  {
    std::stringstream description;

    description << triggering_entities.description() << "'s relative distance to given entity "
                << entity_ref << " = ";

    print_to(description, last_checked_values);

    description << " " << compare << " " << value << "?";

    return description.str();
  }

  auto distance(const EntityRef & triggering_entity)
  {
    if (freespace) {
      switch (relative_distance_type) {
        case RelativeDistanceType::cartesianDistance:
          return getBoundingBoxDistance(entity_ref, triggering_entity);

        default:
          throw UNSUPPORTED_SETTING_DETECTED(RelativeDistanceCondition, relative_distance_type);
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
          throw UNSUPPORTED_SETTING_DETECTED(RelativeDistanceCondition, relative_distance_type);
      }
    }
  }

  auto evaluate()
  {
    last_checked_values.clear();

    return asBoolean(triggering_entities.apply([&](auto && triggering_entity) {
      last_checked_values.push_back(distance(triggering_entity));
      return compare(last_checked_values.back(), value);
    }));
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_DISTANCE_CONDITION_HPP_
