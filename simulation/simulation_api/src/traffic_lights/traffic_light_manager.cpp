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

#include <simulation_api/traffic_lights/traffic_light_manager.hpp>
#include <simulation_api/entity/exception.hpp>

#include <memory>
#include <vector>
#include <utility>

namespace simulation_api
{
TrafficLightManager::TrafficLightManager(
  std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr,
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher,
  const std::shared_ptr<rclcpp::Clock> & clock_ptr,
  const std::string & map_frame)
: marker_pub_(publisher), clock_ptr_(clock_ptr), map_frame_(map_frame)
{
  traffic_lights_ = {};
  const auto ids = hdmap_utils_ptr->getTrafficLightIds();
  for (const auto id : ids) {
    std::shared_ptr<TrafficLight> light_ptr = std::make_shared<TrafficLight>(id);
    auto red_position = hdmap_utils_ptr->getTrafficLightBulbPosition(
      id,
      TrafficLightColor::RED);
    if (red_position) {
      light_ptr->setPosition(TrafficLightColor::RED, red_position.get());
    }
    auto yellow_position = hdmap_utils_ptr->getTrafficLightBulbPosition(
      id,
      TrafficLightColor::YELLOW);
    if (yellow_position) {
      light_ptr->setPosition(TrafficLightColor::YELLOW, yellow_position.get());
    }
    auto green_position =
      hdmap_utils_ptr->getTrafficLightBulbPosition(
      id,
      TrafficLightColor::GREEN);
    if (green_position) {
      light_ptr->setPosition(TrafficLightColor::GREEN, green_position.get());
    }
    traffic_lights_.insert({id, light_ptr});
  }
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
  const auto now = clock_ptr_->now();
  for (const auto light : traffic_lights_) {
    const auto color = light.second->getColor();
    if (color == TrafficLightColor::NONE) {
      continue;
    } else {
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

void TrafficLightManager::update(double step_time)
{
  bool color_changed = false;
  for (const auto light : traffic_lights_) {
    light.second->update(step_time);
    if (light.second->colorChanged()) {
      color_changed = true;
    }
  }
  if (color_changed) {
    deleteAllMarkers();
  }
}

TrafficLightArrow TrafficLightManager::getArrow(std::int64_t lanelet_id) const
{
  if (traffic_lights_.count(lanelet_id) == 0) {
    throw SimulationRuntimeError("lanelet id does not match");
  }
  return traffic_lights_.at(lanelet_id)->getArrow();
}

TrafficLightColor TrafficLightManager::getColor(std::int64_t lanelet_id) const
{
  if (traffic_lights_.count(lanelet_id) == 0) {
    throw SimulationRuntimeError("lanelet id does not match");
  }
  return traffic_lights_.at(lanelet_id)->getColor();
}
}  // namespace simulation_api
