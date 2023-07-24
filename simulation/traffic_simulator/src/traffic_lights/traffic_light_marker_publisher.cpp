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
auto TrafficLightMarkerPublisher::deleteAllMarkers() const -> void
{
  visualization_msgs::msg::MarkerArray message;
  {
    visualization_msgs::msg::Marker marker;
    marker.action = marker.DELETEALL;
    message.markers.push_back(marker);
  }

  marker_pub_->publish(message);
}

auto TrafficLightMarkerPublisher::drawMarkers() const -> void
{
  visualization_msgs::msg::MarkerArray marker_array;

  const auto now = clock_ptr_->now();

  for (const auto & [id, traffic_light] : traffic_light_manager_->getTrafficLights()) {
    traffic_light.draw(marker_array.markers, now, map_frame_);
  }

  marker_pub_->publish(marker_array);
}

auto TrafficLightMarkerPublisher::update() -> void
{
  if (traffic_light_manager_->hasAnyLightChanged()) {
    deleteAllMarkers();
  }

  drawMarkers();
}

}  // namespace traffic_simulator
