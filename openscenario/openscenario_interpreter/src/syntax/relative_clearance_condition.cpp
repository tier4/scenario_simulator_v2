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

#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/relative_clearance_condition.hpp>
#include <openscenario_interpreter/utility/print.hpp>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
RelativeClearanceCondition::RelativeClearanceCondition(
  const pugi::xml_node & node, Scope & scope, const TriggeringEntities & triggering_entities)
: Scope(scope),
  distance_backward(readAttribute<Double>("distanceBackward", node, scope, 0.0)),
  distance_forward(readAttribute<Double>("distanceForward", node, scope, 0.0)),
  free_space(readAttribute<Boolean>("freeSpace", node, scope)),
  opposite_lanes(readAttribute<Boolean>("oppositeLanes", node, scope)),
  relative_lane_range(readElements<RelativeLaneRange, 0>("RelativeLaneRange", node, scope)),
  entity_refs(readElements<EntityRef, 0>("EntityRef", node, scope)),
  triggering_entities(triggering_entities)
{
}

auto RelativeClearanceCondition::description() const -> String
{
  std::stringstream description;

  description << triggering_entities.description()
              << "'s have relative clearance to given entities [";

  print_to(description, entity_refs);

  description << "]?";

  return description.str();
}

auto RelativeClearanceCondition::evaluate() -> Object
{
  // TODO(HansRobo): implement
  return unspecified;
}

auto RelativeClearanceCondition::getRelativeLanePosition(
  const EntityRef & triggering_entity, const EntityRef & entity, bool use_bounding_box) const
  -> traffic_simulator::LaneletPose
{
  if (use_bounding_box) {
    return static_cast<traffic_simulator::LaneletPose>(makeNativeBoundingBoxRelativeLanePosition(
      triggering_entity, entity, RoutingAlgorithm::shortest));
  } else {
    return static_cast<traffic_simulator::LaneletPose>(
      makeNativeRelativeLanePosition(triggering_entity, entity, RoutingAlgorithm::shortest));
  }
}
}  // namespace syntax
}  // namespace openscenario_interpreter
