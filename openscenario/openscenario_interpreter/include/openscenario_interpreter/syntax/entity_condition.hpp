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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_CONDITION_HPP_

#include <openscenario_interpreter/syntax/acceleration_condition.hpp>
#include <openscenario_interpreter/syntax/collision_condition.hpp>
#include <openscenario_interpreter/syntax/distance_condition.hpp>
#include <openscenario_interpreter/syntax/reach_position_condition.hpp>
#include <openscenario_interpreter/syntax/relative_distance_condition.hpp>
#include <openscenario_interpreter/syntax/speed_condition.hpp>
#include <openscenario_interpreter/syntax/stand_still_condition.hpp>
#include <openscenario_interpreter/syntax/time_headway_condition.hpp>

#include <utility>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- EntityCondition --------------------------------------------------------
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
 * -------------------------------------------------------------------------- */
#define ELEMENT(TYPENAME) \
  std::make_pair( \
    #TYPENAME, \
    [&](auto && node) { \
      return make<TYPENAME>(node, std::forward<decltype(xs)>(xs)...); \
    })

struct EntityCondition
  : public Element
{
  template
  <
    typename Node,
    typename ... Ts
  >
  explicit EntityCondition(const Node & node, Ts && ... xs)
  : Element(
      choice(
        node,
        std::make_pair("EndOfRoadCondition", UNSUPPORTED()),
        ELEMENT(CollisionCondition),
        std::make_pair("OffroadCondition", UNSUPPORTED()),
        ELEMENT(TimeHeadwayCondition),
        std::make_pair("TimeToCollisionCondition", UNSUPPORTED()),
        ELEMENT(AccelerationCondition),
        ELEMENT(StandStillCondition),
        ELEMENT(SpeedCondition),
        std::make_pair("RelativeSpeedCondition", UNSUPPORTED()),
        std::make_pair("TraveledDistanceCondition", UNSUPPORTED()),
        ELEMENT(ReachPositionCondition),
        ELEMENT(DistanceCondition),
        ELEMENT(RelativeDistanceCondition)))
  {}
};

#undef ELEMENT
}
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ENTITY_CONDITION_HPP_
