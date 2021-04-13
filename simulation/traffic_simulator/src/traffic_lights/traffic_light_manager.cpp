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

#include <memory>
#include <string>
#include <traffic_simulator/entity/exception.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <utility>
#include <vector>

namespace traffic_simulator
{
TrafficLightManager::TrafficLightManager(
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
  const rclcpp::Publisher<autoware_perception_msgs::msg::TrafficLightStateArray>::SharedPtr &
    traffic_light_state_array_publisher,
  const std::shared_ptr<rclcpp::Clock> & clock_ptr, const std::string & map_frame)
: traffic_light_state_array_publisher_(traffic_light_state_array_publisher),
  traffic_lights_(),
  marker_pub_(publisher),
  clock_ptr_(clock_ptr),
  map_frame_(map_frame)
{
  for (const auto id : (*hdmap_utils_ptr).getTrafficLightIds()) {
    std::shared_ptr<TrafficLight> light_ptr = std::make_shared<TrafficLight>(id);
    auto red_position = hdmap_utils_ptr->getTrafficLightBulbPosition(id, TrafficLightColor::RED);
    if (red_position) {
      light_ptr->setPosition(TrafficLightColor::RED, red_position.get());
    }
    auto yellow_position =
      hdmap_utils_ptr->getTrafficLightBulbPosition(id, TrafficLightColor::YELLOW);
    if (yellow_position) {
      light_ptr->setPosition(TrafficLightColor::YELLOW, yellow_position.get());
    }
    auto green_position =
      hdmap_utils_ptr->getTrafficLightBulbPosition(id, TrafficLightColor::GREEN);
    if (green_position) {
      light_ptr->setPosition(TrafficLightColor::GREEN, green_position.get());
    }
    traffic_lights_.insert({id, light_ptr});
  }
}

std::vector<std::int64_t> TrafficLightManager::getIds() const
{
  std::vector<std::int64_t> ret;
  for (const auto & traffic_light : traffic_lights_) {
    ret.push_back(traffic_light.first);
  }
  return ret;
}

void TrafficLightManager::deleteAllMarkers() const
{
  visualization_msgs::msg::MarkerArray msg;
  visualization_msgs::msg::Marker marker;
  marker.action = marker.DELETEALL;
  msg.markers.push_back(marker);
  marker_pub_->publish(msg);
}

void TrafficLightManager::drawMarkers() const
{
  visualization_msgs::msg::MarkerArray msg;
  const auto now = (*clock_ptr_).now();
  for (const auto & light : traffic_lights_) {
    const auto color = light.second->getColor();
    if (color != TrafficLightColor::NONE) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = now;
      marker.header.frame_id = map_frame_;
      marker.action = marker.ADD;
      marker.ns = "bulb";
      marker.id = light.first;
      marker.type = marker.SPHERE;
      marker.pose.position = light.second->getPosition(light.second->getColor());
      marker.pose.orientation = geometry_msgs::msg::Quaternion();
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      if (color == TrafficLightColor::GREEN) {
        marker.color = color_utils::makeColorMsg("green");
      }
      if (color == TrafficLightColor::YELLOW) {
        marker.color = color_utils::makeColorMsg("yellow");
      }
      if (color == TrafficLightColor::RED) {
        marker.color = color_utils::makeColorMsg("red");
      }
      msg.markers.push_back(marker);
    }
  }
  marker_pub_->publish(msg);
}

void TrafficLightManager::update(const double step_time)
{
  auto colorChanged = [](const auto & traffic_light) {
    return (*traffic_light.second).colorChanged();
  };

  if (std::any_of(std::begin(traffic_lights_), std::end(traffic_lights_), colorChanged)) {
    deleteAllMarkers();
  }
  drawMarkers();
}
}  // namespace traffic_simulator
