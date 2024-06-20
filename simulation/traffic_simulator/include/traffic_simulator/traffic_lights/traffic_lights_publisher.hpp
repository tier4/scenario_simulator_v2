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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_PUBLISHER_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_PUBLISHER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>

namespace traffic_simulator
{
class TrafficLightsPublisherBase
{
public:
  virtual auto publish(
    const std::unordered_map<lanelet::Id, TrafficLight> & traffic_lights_map) const -> void = 0;
};

template <typename MessageType>
class TrafficLightsPublisher : public TrafficLightsPublisherBase
{
public:
  template <typename NodeTypePointer>
  explicit TrafficLightsPublisher(
    const NodeTypePointer & node_ptr, const std::string & topic_name,
    const std::string & frame = "camera_link")
  : TrafficLightsPublisherBase(),
    frame_(frame),
    clock_ptr_(node_ptr->get_clock()),
    traffic_light_state_array_publisher_(rclcpp::create_publisher<MessageType>(
      node_ptr, topic_name, rclcpp::QoS(10).transient_local()))
  {
  }

  auto publish(const std::unordered_map<lanelet::Id, TrafficLight> & traffic_lights_map) const
    -> void override;

private:
  const std::string frame_;
  const rclcpp::Clock::SharedPtr clock_ptr_;
  const typename rclcpp::Publisher<MessageType>::SharedPtr traffic_light_state_array_publisher_;
};
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_PUBLISHER_HPP_
