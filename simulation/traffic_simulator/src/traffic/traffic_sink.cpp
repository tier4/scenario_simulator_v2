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
  const std::function<void(const std::string &)> & despawn,
  const std::shared_ptr<entity::EntityManager> entity_manager_ptr, const TrafficSinkConfig & config)
: TrafficModuleBase(), despawn_(despawn), entity_manager_ptr_(entity_manager_ptr), config_(config)
{
}

auto TrafficSink::execute(
  [[maybe_unused]] const double current_time, [[maybe_unused]] const double step_time) -> void
{
  for (const auto & entity_name : entity_manager_ptr_->getEntityNames()) {
    if (isEntitySinkable(entity_name)) {
      despawn_(entity_name);
    }
  }
}

auto TrafficSink::appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array) const
  -> void
{
  visualization_msgs::msg::Marker traffic_sink_marker;
  traffic_sink_marker.header.frame_id = "map";
  traffic_sink_marker.ns = "traffic_controller/traffic_sink/" + config_.description;
  traffic_sink_marker.id = 0;
  traffic_sink_marker.action = traffic_sink_marker.ADD;
  traffic_sink_marker.type = 3;  // cylinder
  traffic_sink_marker.pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(config_.position)
      .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1));
  traffic_sink_marker.color = color_names::makeColorMsg("firebrick", 0.99);
  traffic_sink_marker.scale.x = config_.radius * 2.0;
  traffic_sink_marker.scale.y = config_.radius * 2.0;
  traffic_sink_marker.scale.z = 1.0;
  marker_array.markers.emplace_back(traffic_sink_marker);

  visualization_msgs::msg::Marker text_marker;
  text_marker = traffic_sink_marker;
  text_marker.id = 1;
  text_marker.type = 9;  // text
  text_marker.text = config_.description;
  text_marker.color = color_names::makeColorMsg("white", 0.99);
  text_marker.scale.z = 0.6;
  marker_array.markers.emplace_back(text_marker);
}

auto TrafficSink::isEntitySinkable(const std::string & entity_name) const noexcept(false) -> bool
{
  const auto & entity = entity_manager_ptr_->getEntity(entity_name);
  if (
    config_.sinkable_entity_types.find(entity.getEntityType().type) ==
    config_.sinkable_entity_types.end()) {
    return false;
  } else if (math::geometry::getDistance(config_.position, entity.getMapPose()) > config_.radius) {
    return false;
  } else {
    return true;
  }
}
}  // namespace traffic
}  // namespace traffic_simulator
