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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_PUBLISHER_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_PUBLISHER_HPP_

#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights_base.hpp>

namespace traffic_simulator
{
class TrafficLightPublisherBase
{
public:
  virtual auto publish(const TrafficLightsBase & traffic_lights) const -> void = 0;
  virtual ~TrafficLightPublisherBase() = default;

  static auto generateAutowareAutoPerceptionMsg(
    const rclcpp::Time & current_ros_time, const TrafficLightsBase & traffic_lights)
    -> autoware_auto_perception_msgs::msg::TrafficSignalArray;

  static auto generateAutowarePerceptionMsg(
    const rclcpp::Time & current_ros_time, const TrafficLightsBase & traffic_lights)
    -> autoware_perception_msgs::msg::TrafficSignalArray;
};

template <typename MessageType>
class TrafficLightPublisher : public TrafficLightPublisherBase
{
public:
  template <typename NodeTypePointer>
  explicit TrafficLightPublisher(
    const NodeTypePointer & node_ptr, const std::string & topic_name,
    const std::string & frame = "camera_link")
  : TrafficLightPublisherBase(),
    frame_(frame),
    clock_ptr_(node_ptr->get_clock()),
    traffic_light_state_array_publisher_(rclcpp::create_publisher<MessageType>(
      node_ptr, topic_name, rclcpp::QoS(10).transient_local()))
  {
  }

  ~TrafficLightPublisher() override = default;

  auto publish(const TrafficLightsBase & traffic_lights) const -> void override;

private:
  const std::string frame_;
  const rclcpp::Clock::SharedPtr clock_ptr_;
  const typename rclcpp::Publisher<MessageType>::SharedPtr traffic_light_state_array_publisher_;
};
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_PUBLISHER_HPP_
