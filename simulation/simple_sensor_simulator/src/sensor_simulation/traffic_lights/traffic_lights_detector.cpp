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

#include <simple_sensor_simulator/sensor_simulation/traffic_lights/traffic_lights_detector.hpp>

namespace simple_sensor_simulator
{
namespace traffic_lights
{
TrafficLightsDetector::TrafficLightsDetector(const std::string & topic_name, rclcpp::Node & node)
: traffic_light_state_array_publisher_(
    rclcpp::create_publisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>(
      node, topic_name, rclcpp::QoS(10).transient_local()))
{
}

auto TrafficLightsDetector::updateFrame(
  const rclcpp::Time & current_ros_time,
  const std::vector<autoware_auto_perception_msgs::msg::TrafficSignal> & traffic_light_state)
  -> void
{
  autoware_auto_perception_msgs::msg::TrafficSignalArray msg;
  msg.header.frame_id = "camera_link";  // DIRTY HACK!!!
  msg.header.stamp = current_ros_time;
  msg.signals = std::move(traffic_light_state);
  traffic_light_state_array_publisher_->publish(msg);
}
}  // namespace traffic_lights
}  // namespace simple_sensor_simulator
