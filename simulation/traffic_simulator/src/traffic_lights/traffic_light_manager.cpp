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
auto TrafficLightManagerBase::getIds() const -> std::vector<LaneletID>
{
  std::vector<LaneletID> result;

  std::transform(
    std::begin(traffic_lights_), std::end(traffic_lights_), std::back_inserter(result),
    [](const auto & each) { return each.first; });

  return result;
}

auto TrafficLightManagerBase::getInstance(const LaneletID lanelet_id) const -> TrafficLight
{
  return traffic_lights_.at(lanelet_id);
}

auto TrafficLightManagerBase::deleteAllMarkers() const -> void
{
  visualization_msgs::msg::MarkerArray message;
  {
    visualization_msgs::msg::Marker marker;
    marker.action = marker.DELETEALL;
    message.markers.push_back(marker);
  }

  marker_pub_->publish(message);
}

auto TrafficLightManagerBase::drawMarkers() const -> void
{
  visualization_msgs::msg::MarkerArray marker_array;

  const auto now = clock_ptr_->now();

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
      marker_array.markers.push_back(marker);
    }
  }

  marker_pub_->publish(marker_array);
}

auto TrafficLightManagerBase::hasAnyLightChanged() -> bool
{
  return std::any_of(
    std::begin(traffic_lights_), std::end(traffic_lights_), [](const auto & id_and_traffic_light) {
      return (
        std::get<1>(id_and_traffic_light).colorChanged() ||
        std::get<1>(id_and_traffic_light).arrowChanged());
    });
}

auto TrafficLightManagerBase::update(const double step_time) -> void
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

template <>
auto TrafficLightManager<
  autoware_perception_msgs::msg::TrafficLightStateArray>::publishTrafficLightStateArray() const
  -> void
{
  autoware_perception_msgs::msg::TrafficLightStateArray traffic_light_state_array;
  {
    traffic_light_state_array.header.frame_id = "camera_link";  // DIRTY HACK!!!
    traffic_light_state_array.header.stamp = clock_ptr_->now();
    for (const auto & each : traffic_lights_) {
      if (each.second.getColor() != TrafficLightColor::NONE) {
        traffic_light_state_array.states.push_back(
          static_cast<autoware_perception_msgs::msg::TrafficLightState>(each.second));
      }
    }
  }
  traffic_light_state_array_publisher_->publish(traffic_light_state_array);
}

#ifndef SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO
template <>
auto TrafficLightManager<
  autoware_auto_perception_msgs::msg::TrafficSignalArray>::publishTrafficLightStateArray() const
  -> void
{
  autoware_auto_perception_msgs::msg::TrafficSignalArray traffic_light_state_array;
  {
    traffic_light_state_array.header.frame_id = "camera_link";  // DIRTY HACK!!!
    traffic_light_state_array.header.stamp = clock_ptr_->now();
    for (const auto & each : traffic_lights_) {
      if (each.second.getColor() != TrafficLightColor::NONE) {
        traffic_light_state_array.signals.push_back(
          static_cast<autoware_auto_perception_msgs::msg::TrafficSignal>(each.second));
      }
    }
  }
  traffic_light_state_array_publisher_->publish(traffic_light_state_array);
}
#endif
}  // namespace traffic_simulator
