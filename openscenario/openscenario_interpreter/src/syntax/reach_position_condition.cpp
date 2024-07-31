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
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/reach_position_condition.hpp>
#include <openscenario_interpreter/utility/overload.hpp>
#include <openscenario_interpreter/utility/print.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/helper/helper.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
ReachPositionCondition::ReachPositionCondition(
  const pugi::xml_node & node, Scope & scope, const TriggeringEntities & triggering_entities)
: tolerance(readAttribute<Double>("tolerance", node, scope)),
  position(readElement<Position>("Position", node, scope)),
  compare(Rule::lessThan),
  triggering_entities(triggering_entities),
  results(triggering_entities.entity_refs.size(), {Double::nan()}),
  consider_z([]() {
    rclcpp::Node node{"get_parameter", "simulation"};
    node.declare_parameter("consider_pose_by_road_slope", false);
    return node.get_parameter("consider_pose_by_road_slope").as_bool();
  }())
{
}

auto ReachPositionCondition::description() const -> String
{
  std::stringstream description;

  description << triggering_entities.description() << "s distance to given position = ";

  print_to(description, results);

  description << " " << compare << " " << tolerance << "?";

  return description.str();
}

// @todo: after checking all the scenario work well with consider_z = true, remove this function and use std::hypot(x,y,z)
static double hypot(const double x, const double y, const double z, const bool consider_z)
{
  return consider_z ? std::hypot(x, y, z) : std::hypot(x, y);
}

auto ReachPositionCondition::evaluate() -> Object
{
  // TODO USE DistanceCondition::distance
  const auto distance = overload(
    [&](const WorldPosition & position, auto && triggering_entity) {
      const auto pose = makeNativeRelativeWorldPosition(
        triggering_entity, static_cast<geometry_msgs::msg::Pose>(position));
      return hypot(pose.position.x, pose.position.y, pose.position.z, consider_z);
    },
    [&](const RelativeWorldPosition & position, auto && triggering_entity) {
      const auto pose = makeNativeRelativeWorldPosition(
        triggering_entity, static_cast<geometry_msgs::msg::Pose>(position));
      return hypot(pose.position.x, pose.position.y, pose.position.z, consider_z);
    },
    [&](const RelativeObjectPosition & position, auto && triggering_entity) {
      const auto pose = makeNativeRelativeWorldPosition(
        triggering_entity, static_cast<geometry_msgs::msg::Pose>(position));
      return hypot(pose.position.x, pose.position.y, pose.position.z, consider_z);
    },
    [&](const LanePosition & position, auto && triggering_entity) {
      const auto pose = makeNativeRelativeWorldPosition(
        triggering_entity, static_cast<geometry_msgs::msg::Pose>(position));
      return hypot(pose.position.x, pose.position.y, pose.position.z, consider_z);
    });

  results.clear();

  return asBoolean(triggering_entities.apply([&](const auto & triggering_entity) {
    results.push_back(triggering_entity.apply(
      [&](const auto & object) { return apply<double>(distance, position, object); }));
    return not results.back().size() or compare(results.back(), tolerance).min();
  }));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
