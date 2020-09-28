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

#ifndef SCENARIO_RUNNER__SYNTAX__ENTITY_CONDITION_HPP_
#define SCENARIO_RUNNER__SYNTAX__ENTITY_CONDITION_HPP_

#include <scenario_runner/syntax/acceleration_condition.hpp>
#include <scenario_runner/syntax/collision_condition.hpp>
#include <scenario_runner/syntax/reach_position_condition.hpp>
#include <scenario_runner/syntax/relative_distance_condition.hpp>
#include <scenario_runner/syntax/speed_condition.hpp>
#include <scenario_runner/syntax/time_headway_condition.hpp>

#include <utility>

namespace scenario_runner
{
inline namespace syntax
{
/* ==== EntityCondition ======================================================
 *
 * <xsd:complexType name="EntityCondition">
 *   <xsd:choice>
 *     <xsd:element name="EndOfRoadCondition" type="EndOfRoadCondition"/>
 *     <xsd:element name="CollisionCondition" type="CollisionCondition"/>
 *     <xsd:element name="OffroadCondition" type="OffroadCondition"/>
 *     <xsd:element name="TimeHeadwayCondition" type="TimeHeadwayCondition"/>
 *     <xsd:element name="TimeToCollisionCondition" type="TimeToCollisionCondition"/>
 *     <xsd:element name="AccelerationCondition" type="AccelerationCondition"/>
 *     <xsd:element name="StandStillCondition" type="StandStillCondition"/>
 *     <xsd:element name="SpeedCondition" type="SpeedCondition"/>
 *     <xsd:element name="RelativeSpeedCondition" type="RelativeSpeedCondition"/>
 *     <xsd:element name="TraveledDistanceCondition" type="TraveledDistanceCondition"/>
 *     <xsd:element name="ReachPositionCondition" type="ReachPositionCondition"/>
 *     <xsd:element name="DistanceCondition" type="DistanceCondition"/>
 *     <xsd:element name="RelativeDistanceCondition" type="RelativeDistanceCondition"/>
 *   </xsd:choice>
 * </xsd:complexType>
 *
 * ======================================================================== */
struct EntityCondition
  : public Object
{
  template<typename Node, typename ... Ts>
  explicit EntityCondition(const Node & node, Ts && ... xs)
  {
    callWithElements(node, "EndOfRoadCondition", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "CollisionCondition", 0, 1, [&](auto && node)
      {
        return rebind<CollisionCondition>(node, std::forward<decltype(xs)>(xs)...);
      });

    callWithElements(node, "OffroadCondition", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "TimeHeadwayCondition", 0, 1, [&](auto && node)
      {
        return rebind<TimeHeadwayCondition>(node, std::forward<decltype(xs)>(xs)...);
      });

    callWithElements(node, "TimeToCollisionCondition", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "AccelerationCondition", 0, 1, [&](auto && node)
      {
        return rebind<AccelerationCondition>(node, std::forward<decltype(xs)>(xs)...);
      });

    callWithElements(node, "StandStillCondition", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "SpeedCondition", 0, 1, [&](auto && node)
      {
        return rebind<SpeedCondition>(node, std::forward<decltype(xs)>(xs)...);
      });

    callWithElements(node, "RelativeSpeedCondition", 0, 1, THROW_UNSUPPORTED_ERROR(node));
    callWithElements(node, "TraveledDistanceCondition", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "ReachPositionCondition", 0, 1, [&](auto && node)
      {
        return rebind<ReachPositionCondition>(node, std::forward<decltype(xs)>(xs)...);
      });

    callWithElements(node, "DistanceCondition", 0, 1, THROW_UNSUPPORTED_ERROR(node));

    callWithElements(node, "RelativeDistanceCondition", 0, 1, [&](auto && node)
      {
        return rebind<RelativeDistanceCondition>(node, std::forward<decltype(xs)>(xs)...);
      });
  }
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__ENTITY_CONDITION_HPP_
