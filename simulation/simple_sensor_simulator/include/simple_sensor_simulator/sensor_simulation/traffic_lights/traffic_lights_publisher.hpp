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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_PUBLISHER_HPP_
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_PUBLISHER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>

namespace simple_sensor_simulator
{
namespace traffic_lights
{
class TrafficLightsPublisherBase
{
public:
  virtual auto publish(
    const rclcpp::Time & current_ros_time,
    const simulation_api_schema::UpdateTrafficLightsRequest & request) const -> void = 0;
};

template <typename MessageType>
class TrafficLightsPublisher : public TrafficLightsPublisherBase
{
public:
  template <typename NodeTypePointer>
  explicit TrafficLightsPublisher(const NodeTypePointer & node, const std::string & topic_name)
  : TrafficLightsPublisherBase(),
    traffic_light_state_array_publisher_(
      rclcpp::create_publisher<MessageType>(node, topic_name, rclcpp::QoS(10).transient_local()))
  {
  }

  auto publish(
    const rclcpp::Time & current_ros_time,
    const simulation_api_schema::UpdateTrafficLightsRequest & request) const -> void override;

private:
  const typename rclcpp::Publisher<MessageType>::SharedPtr traffic_light_state_array_publisher_;
};
}  // namespace traffic_lights
}  // namespace simple_sensor_simulator
#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_PUBLISHER_HPP_
