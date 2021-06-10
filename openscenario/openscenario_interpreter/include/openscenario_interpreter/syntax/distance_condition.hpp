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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__DISTANCE_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__DISTANCE_CONDITION_HPP_

#include <cmath>
#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/boolean.hpp>
#include <openscenario_interpreter/syntax/double.hpp>
#include <openscenario_interpreter/syntax/position.hpp>
#include <openscenario_interpreter/syntax/rule.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- DistanceCondition ------------------------------------------------------
 *
 *  The current distance between an entity and a reference entity is compared to
 *  a given distance (less, greater, equal). Several additional parameters like
 *  free space etc. can be defined.
 *
 *  <xsd:complexType name="DistanceCondition">
 *    <xsd:all>
 *      <xsd:element name="Position" type="Position"/>
 *    </xsd:all>
 *    <xsd:attribute name="value" type="Double" use="required"/>
 *    <xsd:attribute name="freespace" type="Boolean" use="required"/>
 *    <xsd:attribute name="alongRoute" type="Boolean" use="required"/>
 *    <xsd:attribute name="rule" type="Rule" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct DistanceCondition
{
  /* ---- value ----------------------------------------------------------------
   *
   *  The distance value. Unit: s; Range: [0..inf[.
   *
   * ------------------------------------------------------------------------ */
  const Double value;

  /* ---- freespace ------------------------------------------------------------
   *
   *  True: distance is measured between the closest bounding box points.
   *
   *  False: reference point distance is used.
   *
   * ------------------------------------------------------------------------ */
  const Boolean freespace;

  /* ---- alongRoute -----------------------------------------------------------
   *
   *  True: routing is taken into account, e.g. turns will increase distance.
   *
   *  False: straight line distance is used.
   *
   * ------------------------------------------------------------------------ */
  const Boolean along_route;

  /* ---- rule -----------------------------------------------------------------
   *
   *  The operator (less, greater, equal).
   *
   * ------------------------------------------------------------------------ */
  const Rule compare;

  /* ---- Position -------------------------------------------------------------
   *
   *  The given position the distance is related to.
   *
   * ------------------------------------------------------------------------ */
  const Position position;

  template <typename Node>
  explicit DistanceCondition(
    const Node & node, Scope & outer_scope, const TriggeringEntities & triggering_entities)
  : value(readAttribute<Double>("value", node, outer_scope)),
    freespace(readAttribute<Boolean>("freespace", node, outer_scope)),
    along_route(readAttribute<Boolean>("alongRoute", node, outer_scope)),
    compare(readAttribute<Rule>("rule", node, outer_scope)),
    position(readElement<Position>("Position", node, outer_scope)),
    for_each(triggering_entities)
  {
  }

  const TriggeringEntities for_each;

  auto evaluate()
  {
    return asBoolean(for_each([&](auto && triggering_entity) {
      const auto pose =
        getRelativePose(triggering_entity, static_cast<geometry_msgs::msg::Pose>(position));
      return compare(std::hypot(pose.position.x, pose.position.y), value);
    }));
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__DISTANCE_CONDITION_HPP_
