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

#ifndef SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_PUBLISHER_HPP
#define SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_PUBLISHER_HPP

#include <autoware_auto_perception_msgs/msg/traffic_signal.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace simple_sensor_simulator
{
namespace traffic_lights
{
/** @brief Implements traffic lights detector mechanism simulation
 * Currently it only allows to set traffic lights state and publish them on predefined topic
 * Future implementations might, for example, publish only traffic lights that are in a specific FoV of a camera sensor
 * Further refactoring would be required, however, to achieve this.
 */
class TrafficLightsDetector
{
public:
  TrafficLightsDetector(const std::string & topic_name, rclcpp::Node & node);

  auto updateFrame(
    const rclcpp::Time & current_ros_time,
    const std::vector<autoware_auto_perception_msgs::msg::TrafficSignal> & new_traffic_light_state)
    -> void;

private:
  const rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>::SharedPtr
    traffic_light_state_array_publisher_;
};
}  // namespace traffic_lights
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__SENSOR_SIMULATION__TRAFFIC_LIGHTS__TRAFFIC_LIGHTS_PUBLISHER_HPP
