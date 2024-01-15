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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__V2I_TRAFFIC_LIGHT_INFO_PUBLISHER_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__V2I_TRAFFIC_LIGHT_INFO_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <simulation_interface/conversions.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>

#include <jpn_signal_v2i_msgs/msg/external_traffic_signal.hpp>

#include <memory>
#include <string>

namespace traffic_simulator
{
using Message = jpn_signal_v2i_msgs::msg::ExternalTrafficSignal;
class V2ITrafficLightInfoPublisher
{
  const typename rclcpp::Publisher<Message>::SharedPtr publisher_;

public:
  template <typename NodePointer>
  explicit V2ITrafficLightInfoPublisher(const std::string & topic_name, const NodePointer & node)
  : publisher_(
      rclcpp::create_publisher<Message>(node, topic_name, rclcpp::QoS(10).transient_local()))
  {
  }

  auto publish() -> void {}
};
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__V2I_TRAFFIC_LIGHT_INFO_PUBLISHER_HPP_
