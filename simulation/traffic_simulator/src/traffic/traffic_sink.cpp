/**
 * @file traffic_sink.cpp
 * @author Masaya Kataoka (masaya.kataoka@tier4.jp)
 * @brief implementation of the TrafficSink class
 * @version 0.1
 * @date 2021-04-01
 *
 * @copyright Copyright(c) TIER IV.Inc {2015}
 *
 */

// Copyright 2015 TIER IV.inc. All rights reserved.
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

#include <color_names/color_names.hpp>
#include <functional>
#include <geometry/distance.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <traffic_simulator/traffic/traffic_sink.hpp>
#include <vector>

namespace traffic_simulator
{
namespace traffic
{
TrafficSink::TrafficSink(
  const std::shared_ptr<entity::EntityManager> entity_manager_ptr, const double radius,
  const geometry_msgs::msg::Point & position,
  const std::unordered_set<std::uint8_t> & sinkable_entity_type,
  const std::optional<lanelet::Id> lanelet_id_opt /*= std::nullopt*/)
: TrafficModuleBase(),
  description([](std::optional<lanelet::Id> lanelet_id_opt) -> std::string {
    static long unique_id = 0L;
    if (lanelet_id_opt.has_value()) {
      return std::string("auto_") + std::to_string(lanelet_id_opt.value());
    } else {
      return std::string("custom_") + std::to_string(unique_id++);
    }
  }(lanelet_id_opt)),
  radius(radius),
  position(position),
  entity_manager_ptr(entity_manager_ptr),
  sinkable_entity_type(sinkable_entity_type)
{
}

void TrafficSink::execute(
  [[maybe_unused]] const double current_time, [[maybe_unused]] const double step_time)
{
  const auto names = getEntityNames();
  for (const auto & name : names) {
    const auto is_sinkable_entity = [this](const auto & entity_name) {
      return sinkable_entity_type.empty()
               ? true
               : sinkable_entity_type.find(getEntityType(entity_name).type) !=
                   sinkable_entity_type.end();
    };
    const auto pose = getEntityPose(name);
    if (is_sinkable_entity(name) and math::geometry::getDistance(position, pose) <= radius) {
      despawn(name);
    }
  }
}

auto TrafficSink::appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array) const
  -> void
{
  visualization_msgs::msg::Marker traffic_sink_marker;
  traffic_sink_marker.header.frame_id = "map";
  traffic_sink_marker.ns = "traffic_controller/traffic_sink/" + description;
  traffic_sink_marker.id = 0;
  traffic_sink_marker.action = traffic_sink_marker.ADD;
  traffic_sink_marker.type = 3;  // cylinder
  traffic_sink_marker.pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>().position(position).orientation(
      geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1));
  traffic_sink_marker.color = color_names::makeColorMsg("firebrick", 0.99);
  traffic_sink_marker.scale.x = radius * 2;
  traffic_sink_marker.scale.y = radius * 2;
  traffic_sink_marker.scale.z = 1.0;
  marker_array.markers.emplace_back(traffic_sink_marker);

  visualization_msgs::msg::Marker text_marker;
  text_marker = traffic_sink_marker;
  text_marker.id = 1;
  text_marker.type = 9;  //text
  text_marker.text = description;
  text_marker.color = color_names::makeColorMsg("white", 0.99);
  text_marker.scale.z = 0.6;
  marker_array.markers.emplace_back(text_marker);
}

auto TrafficSink::getEntityNames() const -> std::vector<std::string>
{
  return entity_manager_ptr->getEntityNames();
}

auto TrafficSink::getEntityType(const std::string & entity_name) const noexcept(false) -> EntityType
{
  if (const auto entity = entity_manager_ptr->getEntity(entity_name)) {
    return entity->getEntityType();
  } else {
    THROW_SEMANTIC_ERROR("Entity ", std::quoted(entity_name), " does not exists.");
  }
}
auto TrafficSink::getEntityPose(const std::string & entity_name) const noexcept(false)
  -> geometry_msgs::msg::Pose
{
  if (const auto entity = entity_manager_ptr->getEntity(entity_name)) {
    return entity->getMapPose();
  } else {
    THROW_SEMANTIC_ERROR("Entity ", std::quoted(entity_name), " does not exists.");
  }
}
auto TrafficSink::despawn(const std::string & entity_name) const -> void
{
  const auto entity_position = getEntityPose(entity_name).position;
  const bool in_despawn_proximity = math::geometry::hypot(entity_position, position) <= radius;

  const std::uint8_t entity_type = getEntityType(entity_name).type;
  const bool is_despawn_candidate =
    sinkable_entity_type.empty() or
    sinkable_entity_type.find(entity_type) != sinkable_entity_type.cend();
  if (is_despawn_candidate and in_despawn_proximity) {
    entity_manager_ptr->despawnEntity(entity_name);
  }
}
}  // namespace traffic
}  // namespace traffic_simulator
