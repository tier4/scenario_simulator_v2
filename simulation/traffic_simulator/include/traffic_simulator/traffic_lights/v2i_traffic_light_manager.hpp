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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__V2I_TRAFFIC_LIGHT_MANAGER_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__V2I_TRAFFIC_LIGHT_MANAGER_HPP_

#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager_base.hpp>

namespace traffic_simulator
{
template <typename Message>
class V2ITrafficLightManager : public TrafficLightManagerBase
{
//  const typename rclcpp::Publisher<Message>::SharedPtr traffic_light_state_array_publisher_;

public:
  template <typename Node>
  explicit V2ITrafficLightManager(
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap, const Node & node,
    const std::string & map_frame = "map")
  : TrafficLightManagerBase(node, hdmap, map_frame)
//    traffic_light_state_array_publisher_(
//      rclcpp::create_publisher<Message>(node, name(), rclcpp::QoS(10).transient_local()))
  {
  }

private:
  static auto name() -> const char *;

  auto publishTrafficLightStateArray() const -> void override;
};

template <>
auto V2ITrafficLightManager<
  autoware_auto_perception_msgs::msg::TrafficSignalArray>::publishTrafficLightStateArray() const
  -> void;

template <>
auto V2ITrafficLightManager<autoware_auto_perception_msgs::msg::TrafficSignalArray>::name() -> const
  char *;
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__V2I_TRAFFIC_LIGHT_MANAGER_HPP_
