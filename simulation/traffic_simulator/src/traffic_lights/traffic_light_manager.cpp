// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <iterator>
#include <memory>
#include <string>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <type_traits>
#include <utility>
#include <vector>

namespace traffic_simulator
{
std::vector<std::int64_t> TrafficLightManager::getIds() const
{
  std::vector<std::int64_t> result;

  std::transform(
    std::begin(traffic_lights_), std::end(traffic_lights_), std::back_inserter(result),
    [](const auto & each) { return std::get<0>(each); });

  return result;
}

TrafficLight TrafficLightManager::getInstance(const std::int64_t lanelet_id)
{
  return traffic_lights_.at(lanelet_id);
}

void TrafficLightManager::deleteAllMarkers() const
{
  visualization_msgs::msg::MarkerArray msg;
  {
    visualization_msgs::msg::Marker marker;
    marker.action = marker.DELETEALL;
    msg.markers.push_back(marker);
  }

  marker_pub_->publish(msg);
}

void TrafficLightManager::drawMarkers() const
{
  visualization_msgs::msg::MarkerArray msg;

  const auto now = (*clock_ptr_).now();

  for (const auto & light : traffic_lights_) {
    const auto color = std::get<1>(light).getColor();

    if (color != TrafficLightColor::NONE) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = now;
      marker.header.frame_id = map_frame_;
      marker.action = marker.ADD;
      marker.ns = "bulb";
      marker.id = light.first;
      marker.type = marker.SPHERE;
      marker.pose.position = std::get<1>(light).getPosition(color);
      marker.pose.orientation = geometry_msgs::msg::Quaternion();
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      marker.color = color_utils::makeColorMsg(boost::lexical_cast<std::string>(color));
      msg.markers.push_back(marker);
    }
  }

  marker_pub_->publish(msg);
}

bool TrafficLightManager::hasAnyLightChanged()
{
  return std::any_of(
    std::begin(traffic_lights_), std::end(traffic_lights_), [](const auto & id_and_traffic_light) {
      return (
        std::get<1>(id_and_traffic_light).colorChanged() ||
        std::get<1>(id_and_traffic_light).arrowChanged());
    });
}

void TrafficLightManager::update(const double step_time)
{
  publishTrafficLightStateArray();

  for (auto & light : traffic_lights_) {
    light.second.update(step_time);
  }

  if (hasAnyLightChanged()) {
    deleteAllMarkers();
  }

  drawMarkers();
}
}  // namespace traffic_simulator
