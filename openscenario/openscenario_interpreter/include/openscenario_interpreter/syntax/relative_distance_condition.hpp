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
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/syntax/coordinate_system.hpp>
#include <openscenario_interpreter/syntax/relative_distance_type.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>
#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
enum class Between {
  reference_points,             // freespace = false
  closest_bounding_box_points,  // freespace = true
};

/* ---- RelativeDistanceCondition (OpenSCENARIO 1.1) ---------------------------
 *
 *  The current relative distance of a triggering entity/entities to a
 *  reference entity is compared to a given value. The logical operator used
 *  for comparison is defined in the rule attribute.
 *
 *  <xsd:complexType name="RelativeDistanceCondition">
 *    <xsd:attribute name="coordinateSystem" type="CoordinateSystem"/>
 *    <xsd:attribute name="entityRef" type="String" use="required"/>
 *    <xsd:attribute name="freespace" type="Boolean" use="required"/>
 *    <xsd:attribute name="relativeDistanceType" type="RelativeDistanceType" use="required"/>
 *    <xsd:attribute name="rule" type="Rule" use="required"/>
 *    <xsd:attribute name="value" type="Double" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct RelativeDistanceCondition : private Scope
{
  const CoordinateSystem coordinate_system;
  // Definition of the coordinate system to be used for calculations. If not provided the value is interpreted as "entity".

  const String entity_ref;  // Reference entity.

  const Boolean freespace;
  // True: distance is measured between closest bounding box points. False: reference point distance is used.

  const RelativeDistanceType relative_distance_type;
  // Definition of the coordinate system dimension(s) to be used for calculating distances.

  const Rule rule;  // The operator (less, greater, equal).

  const Double value;  // The distance value. Unit: m; Range: [0..inf[.

  const TriggeringEntities triggering_entities;

  std::vector<Double> last_checked_values;  // for description

  template <typename Node, typename Scope>
  explicit RelativeDistanceCondition(
    const Node & node, Scope & scope, const TriggeringEntities & triggering_entities)
  // clang-format off
  : Scope(scope),
    coordinate_system     (readAttribute<CoordinateSystem    >("coordinateSystem",     node, scope, CoordinateSystem::entity)),
    entity_ref            (readAttribute<String              >("entityRef",            node, scope)),
    freespace             (readAttribute<Boolean             >("freespace",            node, scope)),
    relative_distance_type(readAttribute<RelativeDistanceType>("relativeDistanceType", node, scope)),
    rule                  (readAttribute<Rule                >("rule",                 node, scope)),
    value                 (readAttribute<Double              >("value",                node, scope)),
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

    description << " " << rule << " " << value << "?";

    return description.str();
  }

  template <CoordinateSystem::value_type, RelativeDistanceType::value_type, Between>
  auto distance(const EntityRef &) -> double
  {
    throw SyntaxError(__FILE__, ":", __LINE__);
  }

  auto distance(const EntityRef &) -> double;

  auto evaluate()
  {
    last_checked_values.clear();

    return asBoolean(triggering_entities.apply([&](const auto & triggering_entity) {
      last_checked_values.push_back(distance(triggering_entity));
      return rule(last_checked_values.back(), value);
    }));
  }
};

// clang-format off
template <> auto RelativeDistanceCondition::distance< CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, Between::closest_bounding_box_points>(const EntityRef &) -> double;
template <> auto RelativeDistanceCondition::distance< CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, Between::reference_points           >(const EntityRef &) -> double;
template <> auto RelativeDistanceCondition::distance< CoordinateSystem::entity, RelativeDistanceType::lateral,           Between::reference_points           >(const EntityRef &) -> double;
template <> auto RelativeDistanceCondition::distance< CoordinateSystem::entity, RelativeDistanceType::longitudinal,      Between::reference_points           >(const EntityRef &) -> double;
template <> auto RelativeDistanceCondition::distance< CoordinateSystem::lane,   RelativeDistanceType::longitudinal,      Between::reference_points           >(const EntityRef &) -> double;
// clang-format on
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_DISTANCE_CONDITION_HPP_
