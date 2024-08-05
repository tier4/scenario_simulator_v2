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
#include <geometry_msgs/msg/quaternion.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <traffic_simulator/traffic/traffic_sink.hpp>
#include <vector>

namespace traffic_simulator
{
namespace traffic
{
TrafficSink::TrafficSink(
  lanelet::Id lanelet_id, double radius, const geometry_msgs::msg::Point & position,
  const std::function<std::vector<std::string>(void)> & get_entity_names_function,
  const std::function<geometry_msgs::msg::Pose(const std::string &)> & get_entity_pose_function,
  const std::function<void(std::string)> & despawn_function)
: TrafficModuleBase(),
  lanelet_id(lanelet_id),
  radius(radius),
  position(position),
  get_entity_names_function(get_entity_names_function),
  get_entity_pose_function(get_entity_pose_function),
  despawn_function(despawn_function)
{
}

void TrafficSink::execute(
  [[maybe_unused]] const double current_time, [[maybe_unused]] const double step_time)
{
  const auto names = get_entity_names_function();
  for (const auto & name : names) {
    const auto pose = get_entity_pose_function(name);
    if (math::geometry::getDistance(position, pose) <= radius) {
      despawn_function(name);
    }
  }
}

auto TrafficSink::appendDebugMarker(visualization_msgs::msg::MarkerArray & marker_array) const
  -> void
{
  const auto lanelet_text = std::to_string(lanelet_id);
  visualization_msgs::msg::Marker traffic_sink_marker;
  traffic_sink_marker.header.frame_id = "map";
  traffic_sink_marker.ns = "traffic_controller/traffic_sink/" + lanelet_text;
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
  text_marker.text = lanelet_text;
  text_marker.color = color_names::makeColorMsg("white", 0.99);
  text_marker.scale.z = 0.6;
  marker_array.markers.emplace_back(text_marker);
}
}  // namespace traffic
}  // namespace traffic_simulator
