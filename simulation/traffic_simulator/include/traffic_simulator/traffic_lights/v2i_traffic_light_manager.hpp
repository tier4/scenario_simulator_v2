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

#if __has_include(<autoware_perception_msgs/msg/traffic_signal_array.hpp>)

#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager_base.hpp>

namespace traffic_simulator
{
class V2ITrafficLightManager : public TrafficLightManagerBase
{
  using MessageType = autoware_perception_msgs::msg::TrafficSignalArray;

  const rclcpp::Publisher<MessageType>::SharedPtr traffic_light_state_array_publisher_;

public:
  template <typename Node>
  explicit V2ITrafficLightManager(
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap, const Node & node,
    const std::string & map_frame = "map")
  : TrafficLightManagerBase(node, hdmap, map_frame),
    traffic_light_state_array_publisher_(rclcpp::create_publisher<MessageType>(
      node, "/v2x/traffic_signals", rclcpp::QoS(10).transient_local()))
  {
  }

private:
  static auto name() -> const char *;

  auto publishTrafficLightStateArray() const -> void override
  {
    MessageType traffic_light_state_array;
    {
      traffic_light_state_array.stamp = clock_ptr_->now();
      for (const auto & [id, traffic_light] : getTrafficLights()) {
        traffic_light_state_array.signals.push_back(
          static_cast<autoware_perception_msgs::msg::TrafficSignal>(traffic_light));
      }
    }
    traffic_light_state_array_publisher_->publish(traffic_light_state_array);
  }
};

}  // namespace traffic_simulator

#endif

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__V2I_TRAFFIC_LIGHT_MANAGER_HPP_
