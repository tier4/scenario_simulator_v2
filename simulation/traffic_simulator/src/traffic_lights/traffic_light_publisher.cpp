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

// This message will be deleted in the future
#if __has_include(<autoware_perception_msgs/msg/traffic_signal_array.hpp>)
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#endif

#if __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#endif

#include <traffic_simulator/traffic_lights/traffic_light_publisher.hpp>
#include <traffic_simulator_msgs/msg/traffic_light_array_v1.hpp>

namespace traffic_simulator
{
#if __has_include(<autoware_perception_msgs/msg/traffic_signal_array.hpp>)
template <>
auto TrafficLightPublisher<autoware_perception_msgs::msg::TrafficSignalArray>::publish(
  const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request) -> void
{
  assert(hdmap_utils_ != nullptr);

  autoware_perception_msgs::msg::TrafficSignalArray message;
  message.stamp = current_ros_time;
  for (const auto & traffic_light : request.states()) {
    auto relation_ids =
      hdmap_utils_->getTrafficLightRegulatoryElementIDsFromTrafficLight(traffic_light.id());

    for (const auto & relation_id : relation_ids) {
      // skip if the traffic light has no bulbs
      if (not traffic_light.traffic_light_status().empty()) {
        using TrafficLightType = autoware_perception_msgs::msg::TrafficSignal;
        TrafficLightType traffic_light_message;
        traffic_light_message.traffic_signal_id = relation_id;

        for (const auto & bulb_status : traffic_light.traffic_light_status()) {
          using TrafficLightBulbType =
            autoware_perception_msgs::msg::TrafficSignal::_elements_type::value_type;
          TrafficLightBulbType light_bulb_message;
          simulation_interface::toMsg<TrafficLightBulbType>(bulb_status, light_bulb_message);
          traffic_light_message.elements.push_back(light_bulb_message);
        }
        message.signals.push_back(traffic_light_message);
      }
    }
  }
  traffic_light_state_array_publisher_->publish(message);
}
#endif

#if __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)
template <>
auto TrafficLightPublisher<autoware_perception_msgs::msg::TrafficLightGroupArray>::publish(
  const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request) -> void
{
  assert(hdmap_utils_ != nullptr);

  autoware_perception_msgs::msg::TrafficLightGroupArray message;
  message.stamp = current_ros_time;
  for (const auto & traffic_light : request.states()) {
    auto relation_ids =
      hdmap_utils_->getTrafficLightRegulatoryElementIDsFromTrafficLight(traffic_light.id());

    for (auto relation_id : relation_ids) {
      // skip if the traffic light has no bulbs
      if (not traffic_light.traffic_light_status().empty()) {
        using TrafficLightGroupType = autoware_perception_msgs::msg::TrafficLightGroup;
        TrafficLightGroupType traffic_light_group_message;
        traffic_light_group_message.traffic_light_group_id = relation_id;

        for (auto bulb_status : traffic_light.traffic_light_status()) {
          using TrafficLightBulbType = autoware_perception_msgs::msg::TrafficLightElement;
          TrafficLightBulbType light_bulb_message;
          simulation_interface::toMsg<TrafficLightBulbType>(bulb_status, light_bulb_message);
          traffic_light_group_message.elements.push_back(light_bulb_message);
        }
        message.traffic_light_groups.push_back(traffic_light_group_message);
      }
    }
  }
  traffic_light_state_array_publisher_->publish(message);
}
#endif

template <>
auto TrafficLightPublisher<traffic_simulator_msgs::msg::TrafficLightArrayV1>::publish(
  [[maybe_unused]] const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request) -> void
{
  traffic_simulator_msgs::msg::TrafficLightArrayV1 message;
  using TrafficLightType = traffic_simulator_msgs::msg::TrafficLightV1;
  for (const auto & traffic_light : request.states()) {
    TrafficLightType traffic_light_message;
    traffic_light_message.lanelet_way_id = traffic_light.id();
    for (const auto & bulb_status : traffic_light.traffic_light_status()) {
      using TrafficLightBulbType = traffic_simulator_msgs::msg::TrafficLightBulbV1;
      TrafficLightBulbType light_bulb_message;
      simulation_interface::toMsg<TrafficLightBulbType>(bulb_status, light_bulb_message);
      traffic_light_message.traffic_light_bulbs.push_back(light_bulb_message);
    }
    message.traffic_lights.push_back(traffic_light_message);
  }
  traffic_light_state_array_publisher_->publish(message);
}
}  // namespace traffic_simulator
