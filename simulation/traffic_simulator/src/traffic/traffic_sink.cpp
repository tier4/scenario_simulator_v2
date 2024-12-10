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
  const std::shared_ptr<entity::EntityManager> entity_manager_ptr, const TrafficSinkConfig & config)
: TrafficModuleBase(), config(config), entity_manager_ptr(entity_manager_ptr)
{
}

void TrafficSink::execute(
  [[maybe_unused]] const double current_time, [[maybe_unused]] const double step_time)
{
  const auto entity_names = getEntityNames();
  for (const auto & entity_name : entity_names) {
    const bool is_in_sinkable_radius =
      math::geometry::getDistance(config.position, getEntityPose(entity_name)) <= config.radius;
    const bool has_sinkable_entity_type =
      config.sinkable_entity_types.find(getEntityType(entity_name).type) !=
      config.sinkable_entity_types.end();
    if (has_sinkable_entity_type and is_in_sinkable_radius) {
      entity_manager_ptr->despawnEntity(entity_name);
    }
  }
}

auto TrafficSink::appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array) const
  -> void
{
  visualization_msgs::msg::Marker traffic_sink_marker;
  traffic_sink_marker.header.frame_id = "map";
  traffic_sink_marker.ns = "traffic_controller/traffic_sink/" + config.description;
  traffic_sink_marker.id = 0;
  traffic_sink_marker.action = traffic_sink_marker.ADD;
  traffic_sink_marker.type = 3;  // cylinder
  traffic_sink_marker.pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(config.position)
      .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1));
  traffic_sink_marker.color = color_names::makeColorMsg("firebrick", 0.99);
  traffic_sink_marker.scale.x = config.radius * 2.0;
  traffic_sink_marker.scale.y = config.radius * 2.0;
  traffic_sink_marker.scale.z = 1.0;
  marker_array.markers.emplace_back(traffic_sink_marker);

  visualization_msgs::msg::Marker text_marker;
  text_marker = traffic_sink_marker;
  text_marker.id = 1;
  text_marker.type = 9;  //text
  text_marker.text = config.description;
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
}  // namespace traffic
}  // namespace traffic_simulator
