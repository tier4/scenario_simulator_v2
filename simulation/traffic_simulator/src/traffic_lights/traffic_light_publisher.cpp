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

#include <simulation_interface/conversions.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_publisher.hpp>

namespace traffic_simulator
{
auto TrafficLightPublisherBase::generateAutowareAutoPerceptionMsg(
  const rclcpp::Time & current_ros_time, const TrafficLightsBase & traffic_lights)
  -> autoware_auto_perception_msgs::msg::TrafficSignalArray
{
  const auto states_as_proto_request = traffic_lights.generateUpdateTrafficLightsRequest();
  autoware_auto_perception_msgs::msg::TrafficSignalArray message;

  message.header.frame_id = "camera_link";  // DIRTY HACK!!!
  message.header.stamp = current_ros_time;

  using TrafficLightType = autoware_auto_perception_msgs::msg::TrafficSignal;
  using TrafficLightBulbType = TrafficLightType::_lights_type::value_type;
  for (const auto & traffic_light : states_as_proto_request.states()) {
    TrafficLightType traffic_light_message;
    traffic_light_message.map_primitive_id = traffic_light.id();
    for (const auto & bulb_status : traffic_light.traffic_light_status()) {
      TrafficLightBulbType light_bulb_message;
      simulation_interface::toMsg<TrafficLightBulbType>(bulb_status, light_bulb_message);
      traffic_light_message.lights.push_back(light_bulb_message);
    }
    message.signals.push_back(traffic_light_message);
  }
  return message;
}

auto TrafficLightPublisherBase::generateAutowarePerceptionMsg(
  const rclcpp::Time & current_ros_time, const TrafficLightsBase & traffic_lights)
  -> autoware_perception_msgs::msg::TrafficSignalArray
{
  const auto states_as_proto_request = traffic_lights.generateUpdateTrafficLightsRequest();
  autoware_perception_msgs::msg::TrafficSignalArray message;

  message.stamp = current_ros_time;

  using TrafficLightType = autoware_perception_msgs::msg::TrafficSignal;
  using TrafficLightBulbType =
    autoware_perception_msgs::msg::TrafficSignal::_elements_type::value_type;
  for (const auto & traffic_light : states_as_proto_request.states()) {
    for (const auto & relation_id : traffic_light.relation_ids()) {
      // skip if the traffic light has no bulbs
      if (not traffic_light.traffic_light_status().empty()) {
        TrafficLightType traffic_light_message;
        traffic_light_message.traffic_signal_id = relation_id;
        for (const auto & bulb_status : traffic_light.traffic_light_status()) {
          TrafficLightBulbType light_bulb_message;
          simulation_interface::toMsg<TrafficLightBulbType>(bulb_status, light_bulb_message);
          traffic_light_message.elements.push_back(light_bulb_message);
        }
        message.signals.push_back(traffic_light_message);
      }
    }
  }
  return message;
}

template <>
auto TrafficLightPublisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>::publish(
  const TrafficLightsBase & traffic_lights) const -> void
{
  traffic_light_state_array_publisher_->publish(
    generateAutowareAutoPerceptionMsg(clock_ptr_->now(), traffic_lights));
}

template <>
auto TrafficLightPublisher<autoware_perception_msgs::msg::TrafficSignalArray>::publish(
  const TrafficLightsBase & traffic_lights) const -> void
{
  traffic_light_state_array_publisher_->publish(
    generateAutowarePerceptionMsg(clock_ptr_->now(), traffic_lights));
}

template <>
auto TrafficLightPublisher<traffic_simulator_msgs::msg::TrafficLightArrayV1>::publish(
  const TrafficLightsBase & traffic_lights) const -> void
{
  traffic_light_state_array_publisher_->publish(traffic_lights.generateTrafficSimulatorV1Msg());
}
}  // namespace traffic_simulator
