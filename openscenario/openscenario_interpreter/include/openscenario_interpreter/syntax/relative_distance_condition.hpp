// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/coordinate_system.hpp>
#include <openscenario_interpreter/syntax/relative_distance_type.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>
#include <pugixml.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
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
struct RelativeDistanceCondition : private Scope, private SimulatorCore::ConditionEvaluation
{
  /*
     Definition of the coordinate system to be used for calculations. If not
     provided the value is interpreted as "entity".
  */
  const CoordinateSystem coordinate_system;

  /*
     Reference entity.
  */
  const String entity_ref;

  /*
     True: distance is measured between closest bounding box points.
     False: reference point distance is used.
  */
  const Boolean freespace;

  /*
     Definition of the coordinate system dimension(s) to be used for
     calculating distances.
  */
  const RelativeDistanceType relative_distance_type;

  /*
     The operator (less, greater, equal).
  */
  const Rule rule;

  /*
     The distance value. Unit: m; Range: [0..inf[.
  */
  const Double value;

  const TriggeringEntities triggering_entities;

  std::vector<Double> results;  // for description

  const bool consider_z;

  explicit RelativeDistanceCondition(const pugi::xml_node &, Scope &, const TriggeringEntities &);

  auto description() const -> String;

  template <CoordinateSystem::value_type, RelativeDistanceType::value_type, Boolean::value_type>
  auto distance(const EntityRef &) -> double
  {
    throw SyntaxError(__FILE__, ":", __LINE__);
  }

  auto distance(const EntityRef &) -> double;

  auto evaluate() -> Object;
};

// NOTE: Ignore spell miss due to OpenSCENARIO standard.
// cspell: ignore euclidian

// clang-format off
template <> auto RelativeDistanceCondition::distance<CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, false>(const EntityRef &) -> double;
template <> auto RelativeDistanceCondition::distance<CoordinateSystem::entity, RelativeDistanceType::euclidianDistance, true >(const EntityRef &) -> double;
template <> auto RelativeDistanceCondition::distance<CoordinateSystem::entity, RelativeDistanceType::lateral,           false>(const EntityRef &) -> double;
template <> auto RelativeDistanceCondition::distance<CoordinateSystem::entity, RelativeDistanceType::lateral,           true >(const EntityRef &) -> double;
template <> auto RelativeDistanceCondition::distance<CoordinateSystem::entity, RelativeDistanceType::longitudinal,      false>(const EntityRef &) -> double;
template <> auto RelativeDistanceCondition::distance<CoordinateSystem::entity, RelativeDistanceType::longitudinal,      true >(const EntityRef &) -> double;
template <> auto RelativeDistanceCondition::distance<CoordinateSystem::lane,   RelativeDistanceType::lateral,           false>(const EntityRef &) -> double;
template <> auto RelativeDistanceCondition::distance<CoordinateSystem::lane,   RelativeDistanceType::lateral,           true >(const EntityRef &) -> double;
template <> auto RelativeDistanceCondition::distance<CoordinateSystem::lane,   RelativeDistanceType::longitudinal,      false>(const EntityRef &) -> double;
template <> auto RelativeDistanceCondition::distance<CoordinateSystem::lane,   RelativeDistanceType::longitudinal,      true >(const EntityRef &) -> double;
// clang-format on
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_DISTANCE_CONDITION_HPP_
