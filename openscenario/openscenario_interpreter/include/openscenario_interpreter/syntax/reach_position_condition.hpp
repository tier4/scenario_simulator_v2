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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__REACH_POSITION_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__REACH_POSITION_CONDITION_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/position.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>

#include <traffic_simulator/helper/helper.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ReachPositionCondition -------------------------------------------------
 *
 *  <xsd:complexType name="ReachPositionCondition">
 *    <xsd:all>
 *      <xsd:element name="Position" type="Position"/>
 *    </xsd:all>
 *    <xsd:attribute name="tolerance" type="Double" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ReachPositionCondition
{
  const Double tolerance;

  const Position position;

  template<typename Node>
  explicit ReachPositionCondition(
    const Node & node, Scope & outer_scope, const TriggeringEntities & triggering_entities)
  : tolerance(readAttribute<Double>("tolerance", node, outer_scope)),
    position(readElement<Position>("Position", node, outer_scope)),
    for_each(triggering_entities)
  {}

  const TriggeringEntities for_each;

  using TriggeringEntity = TriggeringEntities::value_type;

  decltype(auto) operator()(
    const WorldPosition & world_position,
    const TriggeringEntity & triggering_entity) const
  {
    return isReachedPosition(
      triggering_entity,
      static_cast<geometry_msgs::msg::Pose>(world_position),
      tolerance);
  }

  bool operator()(
    const RelativeWorldPosition &,
    const TriggeringEntity &) const
  {
    THROW(ImplementationFault);
  }

  decltype(auto) operator()(
    const LanePosition & lane_position,
    const TriggeringEntity & triggering_entity) const
  {
    return isReachedPosition(
      triggering_entity,
      static_cast<openscenario_msgs::msg::LaneletPose>(lane_position),
      tolerance);
  }

  #ifndef NDEBUG
  auto distance(const TriggeringEntity & name)
  {
    const auto pose = getRelativePose(
      name, static_cast<geometry_msgs::msg::Pose>(position));
    return std::hypot(pose.position.x, pose.position.y);
  }
  #endif

  auto evaluate()
  {
    #ifndef NDEBUG
    std::cout << (indent++) << "- BEC.RPC:\n";
    #endif

    const auto result = asBoolean(
      for_each(
        [&](const auto & triggering_entity)
        {
          const bool result = apply<bool>(*this, position, triggering_entity);
          #ifndef NDEBUG
          std::cout << indent << "  " << triggering_entity << ": ";
          std::cout << std::boolalpha << result;
          std::cout << " (distance = " << distance(triggering_entity);
          std::cout << " < " << tolerance << ")" << std::endl;
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

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__REACH_POSITION_CONDITION_HPP_
