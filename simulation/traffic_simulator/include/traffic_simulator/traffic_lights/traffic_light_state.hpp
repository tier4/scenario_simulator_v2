// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_STATE_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_STATE_HPP_

#ifndef SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO
#include <autoware_auto_perception_msgs/msg/traffic_light.hpp>
#endif

#include <autoware_perception_msgs/msg/lamp_state.hpp>
#include <iostream>
#include <stdexcept>

namespace traffic_simulator
{
enum class TrafficLightColor { NONE, RED, GREEN, YELLOW };

std::istream & operator>>(std::istream &, TrafficLightColor &);

std::ostream & operator<<(std::ostream &, const TrafficLightColor &);

enum class TrafficLightArrow { NONE, STRAIGHT, LEFT, RIGHT };

std::istream & operator>>(std::istream &, TrafficLightArrow &);

std::ostream & operator<<(std::ostream &, const TrafficLightArrow &);

template <typename T>
auto convert(const TrafficLightArrow &) -> T;

template <>
auto convert<autoware_perception_msgs::msg::LampState>(const TrafficLightArrow &)
  -> autoware_perception_msgs::msg::LampState;

#ifndef SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO
template <>
auto convert<autoware_auto_perception_msgs::msg::TrafficLight>(const TrafficLightArrow &)
  -> autoware_auto_perception_msgs::msg::TrafficLight;
#endif

template <typename T>
auto convert(const TrafficLightColor &) -> T;

template <>
auto convert<autoware_perception_msgs::msg::LampState>(const TrafficLightColor &)
  -> autoware_perception_msgs::msg::LampState;

#ifndef SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO
template <>
auto convert<autoware_auto_perception_msgs::msg::TrafficLight>(const TrafficLightColor &)
  -> autoware_auto_perception_msgs::msg::TrafficLight;
#endif
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_STATE_HPP_
