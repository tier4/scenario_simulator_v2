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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/acceleration_condition.hpp>
#include <openscenario_interpreter/syntax/collision_condition.hpp>
#include <openscenario_interpreter/syntax/distance_condition.hpp>
#include <openscenario_interpreter/syntax/entity_condition.hpp>
#include <openscenario_interpreter/syntax/reach_position_condition.hpp>
#include <openscenario_interpreter/syntax/relative_clearance_condition.hpp>
#include <openscenario_interpreter/syntax/relative_distance_condition.hpp>
#include <openscenario_interpreter/syntax/speed_condition.hpp>
#include <openscenario_interpreter/syntax/stand_still_condition.hpp>
#include <openscenario_interpreter/syntax/time_headway_condition.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
EntityCondition::EntityCondition(
  const pugi::xml_node & node, Scope & scope, const TriggeringEntities & triggering_entities)
// clang-format off
: ComplexType(
    choice(node,
      std::make_pair(        "EndOfRoadCondition", [&](const auto & node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
      std::make_pair(        "CollisionCondition", [&](const auto & node) { return make<        CollisionCondition>(node, scope, triggering_entities); }),
      std::make_pair(          "OffroadCondition", [&](const auto & node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
      std::make_pair(      "TimeHeadwayCondition", [&](const auto & node) { return make<      TimeHeadwayCondition>(node, scope, triggering_entities); }),
      std::make_pair(  "TimeToCollisionCondition", [&](const auto & node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
      std::make_pair(     "AccelerationCondition", [&](const auto & node) { return make<     AccelerationCondition>(node, scope, triggering_entities); }),
      std::make_pair(       "StandStillCondition", [&](const auto & node) { return make<       StandStillCondition>(node, scope, triggering_entities); }),
      std::make_pair(            "SpeedCondition", [&](const auto & node) { return make<            SpeedCondition>(node, scope, triggering_entities); }),
      std::make_pair(    "RelativeSpeedCondition", [&](const auto & node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
      std::make_pair( "TraveledDistanceCondition", [&](const auto & node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
      std::make_pair(    "ReachPositionCondition", [&](const auto & node) { return make<    ReachPositionCondition>(node, scope, triggering_entities); }),
      std::make_pair(         "DistanceCondition", [&](const auto & node) { return make<         DistanceCondition>(node, scope, triggering_entities); }),
      std::make_pair( "RelativeDistanceCondition", [&](const auto & node) { return make< RelativeDistanceCondition>(node, scope, triggering_entities); }),
      std::make_pair("RelativeClearanceCondition", [&](const auto & node) { return make<RelativeClearanceCondition>(node, scope, triggering_entities); }),
      std::make_pair(            "AngleCondition", [&](const auto & node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; }),
      std::make_pair(    "RelativeAngleCondition", [&](const auto & node) { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified; })
      ))
// clang-format on
{
}
}  // namespace syntax
}  // namespace openscenario_interpreter
