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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_HPP_

#include <iostream>
#include <limits>
#include <traffic_simulator/color_utils/color_utils.hpp>
#include <traffic_simulator/entity/exception.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_phase.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_state.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace traffic_simulator
{
class TrafficLight
{
  using Duration = double;

public:
  explicit TrafficLight(
    std::int64_t id,
    const std::unordered_map<TrafficLightColor, geometry_msgs::msg::Point> & color_positions = {},
    const std::unordered_map<TrafficLightArrow, geometry_msgs::msg::Point> & arrow_positions = {});
  void setColorPhase(
    const std::vector<std::pair<Duration, TrafficLightColor>> & phase, double time_offset = 0);
  void setArrowPhase(
    const std::vector<std::pair<Duration, TrafficLightArrow>> & phase, double time_offset = 0);
  void setColor(TrafficLightColor color);
  void setArrow(TrafficLightArrow arrow);
  double getColorPhaseLength() const;
  double getArrowPhaseLength() const;
  void update(double step_time);
  TrafficLightArrow getArrow() const;
  TrafficLightColor getColor() const;
  const std::int64_t id;
  const geometry_msgs::msg::Point getPosition(const TrafficLightColor & color);
  void setPosition(const TrafficLightColor & color, const geometry_msgs::msg::Point & position);
  const geometry_msgs::msg::Point getPosition(const TrafficLightArrow & arrow);
  bool colorChanged() const;
  bool arrowChanged() const;

private:
  std::unordered_map<TrafficLightColor, geometry_msgs::msg::Point> color_positions_;
  std::unordered_map<TrafficLightArrow, geometry_msgs::msg::Point> arrow_positions_;
  TrafficLightPhase<TrafficLightColor> color_phase_;
  TrafficLightPhase<TrafficLightArrow> arrow_phase_;
  bool color_changed_;
  bool arrow_changed_;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_HPP_
