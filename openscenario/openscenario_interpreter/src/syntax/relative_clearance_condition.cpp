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

#include <boost/math/constants/constants.hpp>
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
      for (const auto & entity : *global().entities) {
        if (
          std::find_if(
            triggering_entities.entity_refs.begin(), triggering_entities.entity_refs.end(),
            [&](const auto & triggering_entity) {
              return triggering_entity.name() == entity.first;
            }) == triggering_entities.entity_refs.end()) {
          entities.emplace_back(entity.first);
        }
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

  description << triggering_entities.description() << " have clearances to given entities [";

  print_to(description, entity_refs);

  description << "] with relative longitudinal ranges ( forward: " << distance_forward
              << ", backward: " << distance_backward << " ) and relative lateral ranges [";

  if (relative_lane_range.empty()) {
    description << "all lanes";
  } else {
    for (auto range = relative_lane_range.begin(); range != relative_lane_range.end(); range++) {
      description << ((range != relative_lane_range.begin()) ? std::string(", ") : std::string(""))
                  << range->from << " to " << range->to;
    }
  }

  description << "] ";

  if (opposite_lanes) {
    description << " including opposite lanes?";
  } else {
    description << " excluding opposite lanes?";
  }

  return description.str();
}

auto RelativeClearanceCondition::evaluate() -> Object
{
  return asBoolean(triggering_entities.apply([&](const auto & triggering_entity) {
    return std::all_of(
      entity_refs.begin(), entity_refs.end(),
      [&, is_back =
            (std::abs(evaluateRelativeHeading(triggering_entity)) >
             0.5 * boost::math::constants::pi<double>())](const auto & entity) {
        auto is_in_lateral_range = [&]() {
          // The lanes to be checked to left and right of the triggering entity (positive to the y-axis). If omitted: all lanes are checked.
          if (relative_lane_range.empty()) {
            return true;
          } else {
            if (auto relative_lateral_lane = evaluateLateralRelativeLanes(
                  triggering_entity, entity, RoutingAlgorithm::shortest);
                relative_lateral_lane.has_value()) {
              if (is_back) {
                relative_lateral_lane.value() = -relative_lateral_lane.value();
              }
              return std::any_of(
                relative_lane_range.begin(), relative_lane_range.end(), [&](const auto & range) {
                  return range.from <= relative_lateral_lane.value() &&
                         range.to >= relative_lateral_lane.value();
                });
            } else {
              throw common::Error("Relative lateral lane is not available");
            }
          }
        };

        auto is_in_longitudinal_range = [&]() {
          auto relative_longitudinal =
            getRelativeLanePosition(triggering_entity, entity, free_space).s;
          if (is_back) {
            relative_longitudinal = -relative_longitudinal;
          }
          if (relative_longitudinal < 0) {
            return std::abs(relative_longitudinal) <= distance_backward;
          } else {
            return relative_longitudinal <= distance_forward;
          }
        };

        auto lat_ok = is_in_lateral_range();
        auto lon_ok = is_in_longitudinal_range();
        return not(lat_ok && lon_ok);
      });
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
