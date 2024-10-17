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

#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>

namespace traffic_simulator
{
class TrafficLightMarkerPublisher
{
public:
  template <typename NodeTypePointer>
  explicit TrafficLightMarkerPublisher(
    const NodeTypePointer & node_ptr, const std::string & frame = "map")
  : frame_(frame),
    clock_ptr_(node_ptr->get_clock()),
    publisher_(rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      node_ptr, "traffic_light/marker", rclcpp::QoS(1).transient_local()))
  {
  }

  auto deleteMarkers() const -> void;

  auto drawMarkers(const std::unordered_map<lanelet::Id, TrafficLight> & traffic_lights_map) const
    -> void;

private:
  const std::string frame_;
  const rclcpp::Clock::SharedPtr clock_ptr_;
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MARKER_PUBLISHER_HPP
