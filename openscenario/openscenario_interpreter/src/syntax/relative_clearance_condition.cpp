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
#include <openscenario_interpreter/syntax/entities.hpp>
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
  entity_refs([&]() {
    auto entities = readElements<EntityRef, 0>("EntityRef", node, scope);
    if (entities.empty()) {
      for (const auto & [name, entity] : *global().entities) {
        entities.emplace_back(name);
      }
    }
    return entities;
  }()),
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
  return asBoolean(triggering_entities.apply([&](const auto & triggering_entity) {
    for (const auto & entity : entity_refs) {
      auto is_in_lateral_range = [&]() {
        if (relative_lane_range.empty()) {
          return true;
        } else {
          auto relative_lateral_lane =
            evaluateLateralRelativeLanes(triggering_entity, entity, RoutingAlgorithm::shortest);
          return std::any_of(
            relative_lane_range.begin(), relative_lane_range.end(), [&](const auto & range) {
              return range.from <= relative_lateral_lane && range.to >= relative_lateral_lane;
            });
        }
      };

      auto is_in_longitudinal_range = [&]() {
        auto relative_lane_position =
          getRelativeLanePosition(triggering_entity, entity, free_space);
        if (relative_lane_position.s < 0) {
          return std::abs(relative_lane_position.s) <= distance_backward;
        } else {
          return relative_lane_position.s <= distance_forward;
        }
      };

      return is_in_lateral_range() && is_in_longitudinal_range();
    }
  }));
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
