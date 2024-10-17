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

#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <simple_sensor_simulator/sensor_simulation/traffic_lights/traffic_lights_publisher.hpp>

namespace simple_sensor_simulator
{
namespace traffic_lights
{
template <>
auto TrafficLightsPublisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>::publish(
  const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request) const -> void
{
  using TrafficLightType = autoware_auto_perception_msgs::msg::TrafficSignal;
  using TrafficLightBulbType = TrafficLightType::_lights_type::value_type;

  autoware_auto_perception_msgs::msg::TrafficSignalArray traffic_lights_message;
  traffic_lights_message.header.frame_id = "camera_link";  // DIRTY HACK!!!
  traffic_lights_message.header.stamp = current_ros_time;
  for (const auto & traffic_light : request.states()) {
    // NOT skip if the traffic light has no bulbs
    TrafficLightType single_traffic_light_message;
    single_traffic_light_message.map_primitive_id = traffic_light.id();
    for (const auto & bulb_status : traffic_light.traffic_light_status()) {
      TrafficLightBulbType light_bulb_message;
      simulation_interface::toMsg<TrafficLightBulbType>(bulb_status, light_bulb_message);
      single_traffic_light_message.lights.push_back(light_bulb_message);
    }
    traffic_lights_message.signals.push_back(single_traffic_light_message);
  }
  traffic_light_state_array_publisher_->publish(traffic_lights_message);
}

template <>
auto TrafficLightsPublisher<autoware_perception_msgs::msg::TrafficSignalArray>::publish(
  const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request) const -> void
{
  using TrafficLightType = autoware_perception_msgs::msg::TrafficSignal;
  using TrafficLightBulbType = TrafficLightType::_elements_type::value_type;

  autoware_perception_msgs::msg::TrafficSignalArray traffic_lights_message;
  traffic_lights_message.stamp = current_ros_time;
  for (const auto & traffic_light : request.states()) {
    for (const auto & relation_id : traffic_light.relation_ids()) {
      // skip if the traffic light has no bulbs
      if (not traffic_light.traffic_light_status().empty()) {
        TrafficLightType single_traffic_light_message;
        single_traffic_light_message.traffic_signal_id = relation_id;
        for (const auto & bulb_status : traffic_light.traffic_light_status()) {
          TrafficLightBulbType light_bulb_message;
          simulation_interface::toMsg<TrafficLightBulbType>(bulb_status, light_bulb_message);
          single_traffic_light_message.elements.push_back(light_bulb_message);
        }
        traffic_lights_message.signals.push_back(single_traffic_light_message);
      }
    }
  }
  traffic_light_state_array_publisher_->publish(traffic_lights_message);
}
}  // namespace traffic_lights
}  // namespace simple_sensor_simulator
