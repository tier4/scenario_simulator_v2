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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>

namespace traffic_simulator
{
class TrafficLightPublisherBase
{
public:
  virtual auto publish(
    const rclcpp::Time & current_ros_time,
    const simulation_api_schema::UpdateTrafficLightsRequest & request) -> void = 0;
};

template <typename Message>
class TrafficLightPublisher : public TrafficLightPublisherBase
{
  const typename rclcpp::Publisher<Message>::SharedPtr traffic_light_state_array_publisher_;

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_;

public:
  template <typename NodePointer>
  explicit TrafficLightPublisher(
    const std::string & topic_name, const NodePointer & node,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils = nullptr)
  : TrafficLightPublisherBase(),
    traffic_light_state_array_publisher_(
      rclcpp::create_publisher<Message>(node, topic_name, rclcpp::QoS(10).transient_local())),
    hdmap_utils_(hdmap_utils)
  {
  }

  auto publish(
    const rclcpp::Time & current_ros_time,
    const simulation_api_schema::UpdateTrafficLightsRequest & request) -> void override;
};
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_PUBLISHER_HPP_
