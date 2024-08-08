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
#include <openscenario_interpreter/syntax/entity_selection.hpp>
#include <openscenario_interpreter/syntax/relative_clearance_condition.hpp>
#include <openscenario_interpreter/syntax/scenario_object.hpp>
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
  entity_refs(readElements<Entity, 0>("EntityRef", node, scope)),
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
      description << ((range != relative_lane_range.begin()) ? std::string(", ") : std::string(""));
      if (range->from) {
        if (range->to) {
          description << range->from.value() << " to " << range->to.value();
        } else {
          description << range->from.value() << "or more";
        }
      } else {
        if (range->to) {
          description << range->to.value() << " or less";
        }
      }
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
  auto is_relative_clearance_exist = [&](
                                       const auto & triggering_entity, const auto & target_entity) {
    auto is_back =
      (std::abs(evaluateRelativeHeading(triggering_entity)) >
       boost::math::constants::half_pi<double>());
    auto is_in_lateral_range = [&]() {
      // The lanes to be checked to left and right of the triggering entity (positive to the y-axis). If omitted: all lanes are checked.
      if (relative_lane_range.empty()) {
        return true;
      } else {
        int relative_lateral_lane;
        try {
          relative_lateral_lane = evaluateLateralRelativeLanes(
            triggering_entity, target_entity, RoutingAlgorithm::shortest);
        } catch (const std::exception &) {
          // occurring errors means that the target entity is not in the specified range,
          // under the assumption that relative lane range is defined in routable range .
          return false;
        }
        if (is_back) {
          relative_lateral_lane = -relative_lateral_lane;
        }
        return std::any_of(
          relative_lane_range.begin(), relative_lane_range.end(),
          [&](const auto & range) { return range.evaluate(relative_lateral_lane); });
      }
    };

    auto is_in_longitudinal_range = [&]() {
      auto relative_longitudinal = [&]() {
        if (free_space) {
          return static_cast<traffic_simulator::LaneletPose>(
                   makeNativeBoundingBoxRelativeLanePosition(
                     triggering_entity, target_entity, RoutingAlgorithm::shortest))
            .s;
        } else {
          return static_cast<traffic_simulator::LaneletPose>(
                   makeNativeRelativeLanePosition(
                     triggering_entity, target_entity, RoutingAlgorithm::shortest))
            .s;
        }
      }();

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
  };

  return asBoolean(triggering_entities.apply([&](const auto & triggering_scenario_object) {
    assert(triggering_scenario_object.is<ScenarioObject>());
    if (not entity_refs.empty()) {
      return std::all_of(
        entity_refs.begin(), entity_refs.end(), [&](const auto & target_entity_ref) {
          const auto & target_entity = global().entities->ref(target_entity_ref);
          if (target_entity.template is<ScenarioObject>()) {
            return is_relative_clearance_exist(
              triggering_scenario_object, target_entity.template as<ScenarioObject>().name);
            ;
          } else if (target_entity.template is<EntitySelection>()) {  //
            auto target_scenario_objects = target_entity.template as<EntitySelection>().objects();
            return std::all_of(
              target_scenario_objects.begin(), target_scenario_objects.end(),
              [&](const auto & target_scenario_object) {
                return is_relative_clearance_exist(
                  triggering_scenario_object, target_scenario_object.name());
              });
          } else {
            throw common::Error(
              "Unexpected entity interface is detected. RelativeClearanceCondition expects "
              "ScenarioObject or EntitySelection.");
          }
        });
    } else {
      return std::all_of(
        global().entities->begin(), global().entities->end(),
        [&](const auto & target_candidate_entity) {
          if (not target_candidate_entity.second.template is<ScenarioObject>()) {
            return true;
          } else if (not target_candidate_entity.second.template as<ScenarioObject>().is_added) {
            return true;
          } else {
            const ScenarioObject & target_candidate_scenario_object =
              target_candidate_entity.second.template as<ScenarioObject>();
            // exclude entities included in TriggeringEntities
            auto is_target_candidate_scenario_object_in_triggering_entities =
              [&](const ScenarioObject & target_candidate) -> bool {
              bool is_included = false;
              triggering_entities.apply(
                [target_candidate, &is_included](const auto & triggering_scenario_object) {
                  assert(triggering_scenario_object.is<ScenarioObject>());
                  if (triggering_scenario_object.name() == target_candidate.name) {
                    is_included = true;
                  }
                  return true;
                });
              return is_included;
            };
            if (is_target_candidate_scenario_object_in_triggering_entities(
                  target_candidate_scenario_object)) {
              return true;
            } else {
              const auto & target_scenario_object = target_candidate_scenario_object;
              return is_relative_clearance_exist(
                triggering_scenario_object, target_scenario_object.name);
            }
          }
        });
    }
  }));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
