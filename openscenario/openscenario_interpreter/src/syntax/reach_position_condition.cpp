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

#include <geometry/transform.hpp>
#include <openscenario_interpreter/cmath/hypot.hpp>
#include <openscenario_interpreter/reader/attribute.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/reach_position_condition.hpp>
#include <openscenario_interpreter/utility/overload.hpp>
#include <openscenario_interpreter/utility/print.hpp>
#include <openscenario_interpreter/visualization_buffer.hpp>
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
  results(triggering_entities.entity_refs.size(), {Double::nan()})
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

auto ReachPositionCondition::visualize() const -> void
{
  auto center = static_cast<geometry_msgs::msg::Pose>(position);
  center.orientation.w = 0;
  const auto radius = tolerance;

  const auto make_radius_marker = [&](auto && triggering_entity) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "reach_position_condition/" + triggering_entity.name();
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = center;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;
    marker.color.a = 0.3f;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    return marker;
  };

  const auto make_label_marker = [&](auto && triggering_entity) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "reach_position_condition/" + triggering_entity.name();
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = center;
    marker.pose.position.z += radius;
    marker.text = "ReachPositionCondition\n" + triggering_entity.name() +
                  "\ntolerance = " + std::to_string(tolerance);
    marker.scale.z = 0.3;
    marker.color.a = 0.8f;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    return marker;
  };

  const auto make_distance_marker = [&](auto && triggering_entity) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "reach_position_condition/" + triggering_entity.name();
    marker.id = 3;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    auto relative_pose = makeNativeRelativeWorldPosition(center, triggering_entity.name());
    marker.points.push_back(center.position);
    auto entity_position = center.position;
    entity_position.x -= relative_pose.position.x;
    entity_position.y -= relative_pose.position.y;
    entity_position.z -= relative_pose.position.z;
    marker.points.push_back(entity_position);
    marker.scale.x = 0.1;
    marker.color.a = 0.8f;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    return marker;
  };

  const auto make_distance_label_marker = [&](auto && triggering_entity) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "reach_position_condition/" + triggering_entity.name();
    marker.id = 4;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    auto relative_pose = makeNativeRelativeWorldPosition(center, triggering_entity.name());
    marker.pose.position.x = center.position.x - relative_pose.position.x / 2;
    marker.pose.position.y = center.position.y - relative_pose.position.y / 2;
    marker.pose.position.z = center.position.z - relative_pose.position.z / 2;
    marker.text = std::to_string(hypot(
      relative_pose.position.x, relative_pose.position.y, relative_pose.position.z));
    marker.scale.z = 0.3;
    marker.color.a = 0.8f;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    return marker;
  };

  std::for_each(
    triggering_entities.entity_refs.begin(), triggering_entities.entity_refs.end(),
    [&](auto && triggering_entity) {
      triggering_entity.apply([&](auto && object) {
        add(make_radius_marker(object));
        add(make_label_marker(object));
        add(make_distance_marker(object));
        add(make_distance_label_marker(object));
      });
    });
}

auto ReachPositionCondition::evaluate() -> Object
{
  // TODO USE DistanceCondition::distance
  const auto distance = overload(
    [&](const WorldPosition & position, auto && triggering_entity) {
      const auto pose = makeNativeRelativeWorldPosition(
        triggering_entity, static_cast<geometry_msgs::msg::Pose>(position));
      return hypot(pose.position.x, pose.position.y, pose.position.z);
    },
    [&](const RelativeWorldPosition & position, auto && triggering_entity) {
      const auto pose = makeNativeRelativeWorldPosition(
        triggering_entity, static_cast<geometry_msgs::msg::Pose>(position));
      return hypot(pose.position.x, pose.position.y, pose.position.z);
    },
    [&](const RelativeObjectPosition & position, auto && triggering_entity) {
      const auto pose = makeNativeRelativeWorldPosition(
        triggering_entity, static_cast<geometry_msgs::msg::Pose>(position));
      return hypot(pose.position.x, pose.position.y, pose.position.z);
    },
    [&](const LanePosition & position, auto && triggering_entity) {
      const auto pose = makeNativeRelativeWorldPosition(
        triggering_entity, static_cast<geometry_msgs::msg::Pose>(position));
      return hypot(pose.position.x, pose.position.y, pose.position.z);
    });

  call_visualize([this]() { visualize(); });

  results.clear();

  return asBoolean(triggering_entities.apply([&](const auto & triggering_entity) {
    results.push_back(triggering_entity.apply(
      [&](const auto & object) { return apply<double>(distance, position, object); }));
    return not results.back().size() or compare(results.back(), tolerance).min();
  }));
}
}  // namespace syntax
}  // namespace openscenario_interpreter
