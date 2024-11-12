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

auto TrafficLightPublisherBase::generateTrafficSimulatorV1Msg(
  [[maybe_unused]] const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request)
  -> traffic_simulator_msgs::msg::TrafficLightArrayV1::UniquePtr
{
  auto message_ptr = std::make_unique<traffic_simulator_msgs::msg::TrafficLightArrayV1>();

  using TrafficLightType = traffic_simulator_msgs::msg::TrafficLightV1;
  using TrafficLightBulbType = traffic_simulator_msgs::msg::TrafficLightBulbV1;

  for (const auto & traffic_light : request.states()) {
    TrafficLightType traffic_light_message;
    traffic_light_message.lanelet_way_id = traffic_light.id();
    for (const auto & bulb_status : traffic_light.traffic_light_status()) {
      TrafficLightBulbType light_bulb_message;
      simulation_interface::toMsg<TrafficLightBulbType>(bulb_status, light_bulb_message);
      traffic_light_message.traffic_light_bulbs.push_back(light_bulb_message);
    }
    message_ptr->traffic_lights.push_back(traffic_light_message);
  }
  return message_ptr;
}

auto TrafficLightPublisherBase::generateAutowareAutoPerceptionMsg(
  const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request, const std::string & frame)
  -> autoware_auto_perception_msgs::msg::TrafficSignalArray::UniquePtr
{
  auto message_ptr = std::make_unique<autoware_auto_perception_msgs::msg::TrafficSignalArray>();

  message_ptr->header.frame_id = frame;
  message_ptr->header.stamp = current_ros_time;

  using TrafficLightType = autoware_auto_perception_msgs::msg::TrafficSignal;
  using TrafficLightBulbType = TrafficLightType::_lights_type::value_type;

  for (const auto & traffic_light : request.states()) {
    TrafficLightType traffic_light_message;
    traffic_light_message.map_primitive_id = traffic_light.id();
    for (const auto & bulb_status : traffic_light.traffic_light_status()) {
      TrafficLightBulbType light_bulb_message;
      simulation_interface::toMsg<TrafficLightBulbType>(bulb_status, light_bulb_message);
      traffic_light_message.lights.push_back(light_bulb_message);
    }
    message_ptr->signals.push_back(traffic_light_message);
  }
  return message_ptr;
}

auto TrafficLightPublisherBase::generateAutowarePerceptionTrafficSignalMsg(
  const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request)
  -> autoware_perception_msgs::msg::TrafficSignalArray::UniquePtr
{
  auto message_ptr = std::make_unique<autoware_perception_msgs::msg::TrafficSignalArray>();

  message_ptr->stamp = current_ros_time;

  using TrafficLightType = autoware_perception_msgs::msg::TrafficSignal;
  using TrafficLightBulbType = TrafficLightType::_elements_type::value_type;

  for (const auto & traffic_light : request.states()) {
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
        message_ptr->signals.push_back(traffic_light_message);
      }
    }
  }
  return message_ptr;
}

#if __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)

auto TrafficLightPublisherBase::generateAutowarePerceptionTrafficLightGroupMsg(
  const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request)
  -> autoware_perception_msgs::msg::TrafficLightGroupArray::UniquePtr
{
  auto message_ptr = std::make_unique<autoware_perception_msgs::msg::TrafficLightGroupArray>();

  message_ptr->stamp = current_ros_time;

  using TrafficLightGroupType = autoware_perception_msgs::msg::TrafficLightGroup;
  using TrafficLightBulbType = autoware_perception_msgs::msg::TrafficLightElement;

  for (const auto & traffic_light : request.states()) {
    for (const auto & relation_id : traffic_light.relation_ids()) {
      // skip if the traffic light has no bulbs
      if (not traffic_light.traffic_light_status().empty()) {
        TrafficLightGroupType traffic_light_group_message;
        traffic_light_group_message.traffic_light_group_id = relation_id;
        for (auto bulb_status : traffic_light.traffic_light_status()) {
          TrafficLightBulbType light_bulb_message;
          simulation_interface::toMsg<TrafficLightBulbType>(bulb_status, light_bulb_message);
          traffic_light_group_message.elements.push_back(light_bulb_message);
        }
        message_ptr->traffic_light_groups.push_back(traffic_light_group_message);
      }
    }
  }
  return message_ptr;
}

#endif  // __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)

template <>
auto TrafficLightPublisher<traffic_simulator_msgs::msg::TrafficLightArrayV1>::publish(
  [[maybe_unused]] const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request) const -> void
{
  traffic_light_state_array_publisher_->publish(
    generateTrafficSimulatorV1Msg(current_ros_time, request));
}

template <>
auto TrafficLightPublisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>::publish(
  const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request) const -> void
{
  traffic_light_state_array_publisher_->publish(
    generateAutowareAutoPerceptionMsg(current_ros_time, request, frame_));
}

template <>
auto TrafficLightPublisher<autoware_perception_msgs::msg::TrafficSignalArray>::publish(
  const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request) const -> void
{
  traffic_light_state_array_publisher_->publish(
    generateAutowarePerceptionTrafficSignalMsg(current_ros_time, request));
}

#if __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)

template <>
auto TrafficLightPublisher<autoware_perception_msgs::msg::TrafficLightGroupArray>::publish(
  const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request) const -> void
{
  traffic_light_state_array_publisher_->publish(
    generateAutowarePerceptionTrafficLightGroupMsg(current_ros_time, request));
}

#endif  // __has_include(<autoware_perception_msgs/msg/traffic_light_group_array.hpp>)

}  // namespace traffic_simulator
