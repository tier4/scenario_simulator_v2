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
#include <traffic_simulator/traffic_lights/configurable_rate_updater.hpp>

namespace traffic_simulator
{
class TrafficLightManager;

template <typename Message>
class V2ITrafficLightPublisher : public ConfigurableRateUpdater
{
  const typename rclcpp::Publisher<Message>::SharedPtr traffic_light_state_array_publisher_;
  const std::shared_ptr<TrafficLightManager> traffic_light_manager_;
  const std::string sensor_frame_;

public:
  template <typename NodePointer>
  explicit V2ITrafficLightPublisher(
    const std::shared_ptr<TrafficLightManager> & traffic_light_manager,
    const std::string & topic_name, const NodePointer & node,
    const std::string & sensor_frame = "camera_link")
  : ConfigurableRateUpdater(node),
    traffic_light_state_array_publisher_(
      rclcpp::create_publisher<Message>(node, topic_name, rclcpp::QoS(10).transient_local())),
    traffic_light_manager_(traffic_light_manager),
    sensor_frame_(sensor_frame)
  {
  }

private:
  virtual auto update() -> void override;
};

template <>
auto V2ITrafficLightPublisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>::update()
  -> void;
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__V2I_TRAFFIC_LIGHT_MANAGER_HPP_
