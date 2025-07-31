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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__PREDICTED_TRAFFIC_LIGHT_STATE_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__PREDICTED_TRAFFIC_LIGHT_STATE_HPP_

#include <autoware_perception_msgs/msg/predicted_traffic_light_state.hpp>
#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace traffic_simulator
{
struct PredictedTrafficLightState
{
  // Information source constants matching the message definition
  static constexpr const char* INFORMATION_SOURCE_V2N = "V2N";
  static constexpr const char* INFORMATION_SOURCE_V2I = "V2I";
  static constexpr const char* INFORMATION_SOURCE_V2V = "V2V";
  static constexpr const char* INFORMATION_SOURCE_LOCAL_PERCEPTION = "LOCAL_PERCEPTION";
  static constexpr const char* INFORMATION_SOURCE_MANUAL_OVERRIDE = "MANUAL_OVERRIDE";
  static constexpr const char* INFORMATION_SOURCE_SIMULATION = "SIMULATION";
  static constexpr const char* INFORMATION_SOURCE_INTERNAL_ESTIMATION = "INTERNAL_ESTIMATION";

  // Absolute time this state is expected
  builtin_interfaces::msg::Time predicted_stamp;

  // Valid signals at this time
  std::vector<autoware_perception_msgs::msg::TrafficLightElement> simultaneous_elements;

  // Confidence level [0.0–1.0]
  float reliability;

  // Source of information, using predefined string constants
  std::string information_source;

  // Default constructor
  PredictedTrafficLightState() = default;

  // Constructor from ROS message
  explicit PredictedTrafficLightState(const autoware_perception_msgs::msg::PredictedTrafficLightState & msg)
    : predicted_stamp(msg.predicted_stamp),
      simultaneous_elements(msg.simultaneous_elements),
      reliability(msg.reliability),
      information_source(msg.information_source)
  {
  }

  // Conversion operator to ROS message
  operator autoware_perception_msgs::msg::PredictedTrafficLightState() const
  {
    autoware_perception_msgs::msg::PredictedTrafficLightState msg;
    msg.predicted_stamp = predicted_stamp;
    msg.simultaneous_elements = simultaneous_elements;
    msg.reliability = reliability;
    msg.information_source = information_source;
    return msg;
  }

  // Method to convert to ROS message (alternative to operator)
  autoware_perception_msgs::msg::PredictedTrafficLightState toMsg() const
  {
    return static_cast<autoware_perception_msgs::msg::PredictedTrafficLightState>(*this);
  }
};

}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__PREDICTED_TRAFFIC_LIGHT_STATE_HPP_