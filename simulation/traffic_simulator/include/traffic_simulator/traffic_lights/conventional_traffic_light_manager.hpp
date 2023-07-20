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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__CONVENTIONAL_TRAFFIC_LIGHT_MANAGER_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__CONVENTIONAL_TRAFFIC_LIGHT_MANAGER_HPP_

#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>

#if __has_include(<autoware_perception_msgs/msg/traffic_signal_array.hpp>)
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#endif

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager_base.hpp>

namespace traffic_simulator
{
template <typename TrafficLightArrayMessageType>
class ConventionalTrafficLightManager : public TrafficLightManagerBase
{
  const typename rclcpp::Publisher<TrafficLightArrayMessageType>::SharedPtr publisher_;

#if __has_include(<autoware_perception_msgs/msg/traffic_signal_array.hpp>)

#endif

public:
  template <typename Node>
  explicit ConventionalTrafficLightManager(
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap, const Node & node,
    const std::string & map_frame = "map")
  : TrafficLightManagerBase(node, hdmap, map_frame), publisher_([node]() {
      if constexpr (std::is_same_v<
                      TrafficLightArrayMessageType,
                      autoware_perception_msgs::msg::TrafficSignalArray>) {
        return node->template create_publisher<TrafficLightArrayMessageType>(
          "/perception/traffic_light_recognition/traffic_signals",
          rclcpp::QoS(10).transient_local());
      } else if constexpr (std::is_same_v<
                             TrafficLightArrayMessageType,
                             autoware_auto_perception_msgs::msg::TrafficSignalArray>) {
        return node->template create_publisher<TrafficLightArrayMessageType>(
          "/perception/traffic_light_recognition/traffic_signals",
          rclcpp::QoS(10).transient_local());
      } else {
        static_assert(true, "Unsupported message type");
      }
    }())
  {
  }

private:
  auto publishTrafficLightStateArray() const -> void override
  {
    TrafficLightArrayMessageType traffic_light_array_message;
    if constexpr (std::is_same_v<
                    TrafficLightArrayMessageType,
                    autoware_perception_msgs::msg::TrafficSignalArray>) {
      traffic_light_array_message.stamp = clock_ptr_->now();
      for (const auto & [id, traffic_light] : getTrafficLights()) {
        traffic_light_array_message.signals.push_back(
          static_cast<typename TrafficLightArrayMessageType::_signals_type::value_type>(
            traffic_light));
      }
    } else if constexpr (std::is_same_v<
                           TrafficLightArrayMessageType,
                           autoware_auto_perception_msgs::msg::TrafficSignalArray>) {
      traffic_light_array_message.header.frame_id = "camera_link";  // DIRTY HACK!!!
      traffic_light_array_message.header.stamp = clock_ptr_->now();
      for (const auto & [id, traffic_light] : getTrafficLights()) {
        traffic_light_array_message.signals.push_back(
          static_cast<typename TrafficLightArrayMessageType::_signals_type::value_type>(
            traffic_light));
      }
    } else {
      // not reachable, because of static_assert in constructor
    }
    publisher_->publish(traffic_light_array_message);
  }
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__CONVENTIONAL_TRAFFIC_LIGHT_MANAGER_HPP_
