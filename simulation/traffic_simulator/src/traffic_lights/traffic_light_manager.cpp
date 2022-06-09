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

#include <color_names/color_names.hpp>
#include <iterator>
#include <memory>
#include <string>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <type_traits>
#include <utility>
#include <vector>

namespace traffic_simulator
{
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

  for (const auto & [id, traffic_light] : getTrafficLights()) {
    traffic_light.draw(marker_array.markers, now, map_frame_);
  }

  marker_pub_->publish(marker_array);
}

auto TrafficLightManagerBase::hasAnyLightChanged() -> bool
{
  return true;
  // return std::any_of(
  //   std::begin(getTrafficLights()), std::end(getTrafficLights()), [](auto && id_and_traffic_light) {
  //     return id_and_traffic_light.second.colorChanged() or
  //            id_and_traffic_light.second.arrowChanged();
  //   });
}

auto TrafficLightManagerBase::update(const double) -> void
{
  publishTrafficLightStateArray();

  if (hasAnyLightChanged()) {
    deleteAllMarkers();
  }

  drawMarkers();
}

template <>
auto TrafficLightManager<
  autoware_auto_perception_msgs::msg::TrafficSignalArray>::publishTrafficLightStateArray() const
  -> void
{
  autoware_auto_perception_msgs::msg::TrafficSignalArray traffic_light_state_array;
  {
    traffic_light_state_array.header.frame_id = "camera_link";  // DIRTY HACK!!!
    traffic_light_state_array.header.stamp = clock_ptr_->now();
    for (const auto & [id, traffic_light] : getTrafficLights()) {
      traffic_light_state_array.signals.push_back(
        static_cast<autoware_auto_perception_msgs::msg::TrafficSignal>(traffic_light));
    }
  }
  traffic_light_state_array_publisher_->publish(traffic_light_state_array);
}

template <>
auto TrafficLightManager<autoware_auto_perception_msgs::msg::TrafficSignalArray>::name() -> const
  char *
{
  return "/perception/traffic_light_recognition/traffic_signals";
}
}  // namespace traffic_simulator
