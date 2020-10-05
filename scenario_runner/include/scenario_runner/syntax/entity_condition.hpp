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
  : public Element
{
  template<typename Node, typename ... Ts>
  explicit EntityCondition(const Node & node, Ts && ... xs)
  : Element(
      choice(
        node,
        std::make_pair("EndOfRoadCondition", UNSUPPORTED()),
        std::make_pair("CollisionCondition", [&](auto && node) {
          return make<CollisionCondition>(node, std::forward<decltype(xs)>(xs)...);
        }),
        std::make_pair("OffroadCondition", UNSUPPORTED()),
        std::make_pair("TimeHeadwayCondition", [&](auto && node) {
          return make<TimeHeadwayCondition>(node, std::forward<decltype(xs)>(xs)...);
        }),
        std::make_pair("TimeToCollisionCondition", UNSUPPORTED()),
        std::make_pair("AccelerationCondition", [&](auto && node) {
          return make<AccelerationCondition>(node, std::forward<decltype(xs)>(xs)...);
        }),
        std::make_pair("StandStillCondition", UNSUPPORTED()),
        std::make_pair("SpeedCondition", [&](auto && node) {
          return make<SpeedCondition>(node, std::forward<decltype(xs)>(xs)...);
        }),
        std::make_pair("RelativeSpeedCondition", UNSUPPORTED()),
        std::make_pair("TraveledDistanceCondition", UNSUPPORTED()),
        std::make_pair("ReachPositionCondition", [&](auto && node) {
          return make<ReachPositionCondition>(node, std::forward<decltype(xs)>(xs)...);
        }),
        std::make_pair("DistanceCondition", UNSUPPORTED()),
        std::make_pair("RelativeDistanceCondition", [&](auto && node) {
          return make<RelativeDistanceCondition>(node, std::forward<decltype(xs)>(xs)...);
        })))
  {}
};
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__SYNTAX__ENTITY_CONDITION_HPP_
