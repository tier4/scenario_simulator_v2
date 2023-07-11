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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MARKER_PUBLISHER_HPP
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MARKER_PUBLISHER_HPP

#include <traffic_simulator/traffic_lights/configurable_rate_updater.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>

namespace traffic_simulator
{
class TrafficLightMarkerPublisher : public ConfigurableRateUpdater
{
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  const std::string map_frame_;
  const std::shared_ptr<TrafficLightManager> traffic_light_manager_;

  auto deleteAllMarkers() const -> void;
  auto drawMarkers() const -> void;

public:
  template <typename NodePointer>
  explicit TrafficLightMarkerPublisher(
    const std::shared_ptr<TrafficLightManager> & traffic_light_manager, const NodePointer & node,
    const std::string & map_frame = "map")
  : ConfigurableRateUpdater(node),
    marker_pub_(rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      node, "traffic_light/marker", rclcpp::QoS(1).transient_local())),
    map_frame_(map_frame),
    traffic_light_manager_(traffic_light_manager)
  {
  }

  virtual auto update() -> void override;
};

}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MARKER_PUBLISHER_HPP
