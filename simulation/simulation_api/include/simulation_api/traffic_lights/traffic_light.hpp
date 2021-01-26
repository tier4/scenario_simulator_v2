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

#ifndef SIMULATION_API__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_HPP_
#define SIMULATION_API__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_HPP_

#include <simulation_api/traffic_lights/traffic_light_phase.hpp>

#include <simulation_api/entity/exception.hpp>

#include <vector>
#include <limits>
#include <utility>

namespace simulation_api
{
enum class TrafficLightColor
{
  NONE,
  RED,
  GREEN,
  YELLOW
};

enum class TrafficLightArrow
{
  NONE,
  STRAIGHT,
  LEFT,
  RIGHT
};

class TrafficLight
{
public:
  explicit TrafficLight(std::int64_t id);
  void setColorPhase(
    const std::vector<std::pair<double, TrafficLightColor>> & phase,
    double time_offset = 0);
  void setArrowPhase(
    const std::vector<std::pair<double, TrafficLightArrow>> & phase,
    double time_offset = 0);
  void setColor(TrafficLightColor color);
  void setArrow(TrafficLightArrow arrow);
  double getColorPhaseLength() const;
  double getArrowPhaseLength() const;
  void update(double step_time);
  const std::int64_t id;
  TrafficLightArrow getArrow() const;
  TrafficLightColor getColor() const;

private:
  TrafficLightPhase<TrafficLightColor> color_phase_;
  TrafficLightPhase<TrafficLightArrow> arrow_phase_;
};
}  // namespace simulation_api

#endif  // SIMULATION_API__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_HPP_
