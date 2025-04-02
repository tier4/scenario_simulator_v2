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

#include <traffic_simulator/traffic_lights/traffic_light_marker_publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace traffic_simulator
{
auto TrafficLightMarkerPublisher::deleteMarkers() const -> void
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(marker);
  publisher_->publish(marker_array);
}

auto TrafficLightMarkerPublisher::drawMarkers(
  const std::unordered_map<lanelet::Id, TrafficLight> & traffic_lights_map) const -> void
{
  visualization_msgs::msg::MarkerArray marker_array;
  for (const auto & [id, traffic_light] : traffic_lights_map) {
    traffic_light.draw(marker_array.markers, clock_ptr_->now(), frame_);
  }
  publisher_->publish(marker_array);
}
}  // namespace traffic_simulator
