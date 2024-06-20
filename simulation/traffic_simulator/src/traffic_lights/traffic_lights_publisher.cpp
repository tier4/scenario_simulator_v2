// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights_publisher.hpp>

namespace traffic_simulator
{
template <>
auto TrafficLightsPublisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>::publish(
  const std::unordered_map<lanelet::Id, TrafficLight> & traffic_lights_map) const -> void
{
  autoware_auto_perception_msgs::msg::TrafficSignalArray traffic_lights_message;
  traffic_lights_message.header.frame_id = frame_;
  traffic_lights_message.header.stamp = clock_ptr_->now();
  for (const auto & [id, traffic_light] : traffic_lights_map) {
    traffic_lights_message.signals.push_back(
      static_cast<autoware_auto_perception_msgs::msg::TrafficSignal>(traffic_light));
  }
  traffic_light_state_array_publisher_->publish(traffic_lights_message);
}

template <>
auto TrafficLightsPublisher<autoware_perception_msgs::msg::TrafficSignalArray>::publish(
  const std::unordered_map<lanelet::Id, TrafficLight> & traffic_lights_map) const -> void
{
  autoware_perception_msgs::msg::TrafficSignalArray traffic_lights_message;
  traffic_lights_message.stamp = clock_ptr_->now();

  size_t total_number_of_signals{0};
  for (const auto & [id, traffic_light] : traffic_lights_map) {
    total_number_of_signals += traffic_light.regulatory_elements_ids.size();
  }
  traffic_lights_message.signals.reserve(total_number_of_signals);

  for (const auto & [id, traffic_light] : traffic_lights_map) {
    const auto traffic_signals =
      static_cast<std::vector<autoware_perception_msgs::msg::TrafficSignal>>(traffic_light);
    traffic_lights_message.signals.insert(
      traffic_lights_message.signals.end(), traffic_signals.begin(), traffic_signals.end());
  }
  traffic_light_state_array_publisher_->publish(traffic_lights_message);
}
}  // namespace traffic_simulator
