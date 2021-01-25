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

#include <simulation_api/hdmap_utils/hdmap_utils.hpp>

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
  explicit TrafficLight(std::int64_t id, bool contain_arrow = false);
  explicit TrafficLight(
    std::int64_t id, std::vector<std::pair<double,
    TrafficLightColor>> color_sequence, bool contain_arrow = false);
  explicit TrafficLight(
    std::int64_t id,
    std::vector<std::pair<double, TrafficLightColor>> color_sequence,
    std::vector<std::pair<double, TrafficLightArrow>> arrow_sequence);
  void setColorSequence(
    std::vector<std::pair<double, TrafficLightColor>> color_sequence,
    double time_offset = 0);
  void setArrowSequence(
    std::vector<std::pair<double, TrafficLightArrow>> arrow_sequence,
    double time_offset = 0);
  void setColor(TrafficLightColor color);
  void setArrow(TrafficLightArrow arrow);
  double getColorSequenceLength() const;
  double getArrowSequenceLength() const;
  void update(double step_time);
  const std::int64_t id;
  const bool contain_arrow;
  TrafficLightArrow getArrow() const;
  TrafficLightColor getColor() const;

private:
  std::vector<std::pair<double, TrafficLightColor>> color_sequence_;
  std::vector<std::pair<double, TrafficLightArrow>> arrow_sequence_;
  double elapsed_time_in_color_sequence_;
  double elapsed_time_in_arrow_sequence_;
};
}  // namespace simulation_api

#endif  // SIMULATION_API__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_HPP_
