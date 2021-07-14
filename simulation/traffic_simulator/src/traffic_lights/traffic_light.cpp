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

#include <limits>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/color_utils/color_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace traffic_simulator
{
TrafficLight::TrafficLight(
  const std::int64_t id,
  const std::unordered_map<TrafficLightColor, geometry_msgs::msg::Point> & color_positions,
  const std::unordered_map<TrafficLightArrow, geometry_msgs::msg::Point> & arrow_positions)
: id(id),
  color_positions_(color_positions),
  arrow_positions_(arrow_positions),
  color_changed_(true),
  arrow_changed_(true)
{
  color_phase_.setState(TrafficLightColor::NONE);
  arrow_phase_.setState(TrafficLightArrow::NONE);
}

void TrafficLight::setColor(const TrafficLightColor color) { color_phase_.setState(color); }
void TrafficLight::setArrow(const TrafficLightArrow arrow) { arrow_phase_.setState(arrow); }

double TrafficLight::getColorPhaseDuration() const { return color_phase_.getPhaseDuration(); }
double TrafficLight::getArrowPhaseDuration() const { return arrow_phase_.getPhaseDuration(); }

TrafficLightArrow TrafficLight::getArrow() const { return arrow_phase_.getState(); }
TrafficLightColor TrafficLight::getColor() const { return color_phase_.getState(); }

void TrafficLight::update(const double step_time)
{
  const auto previous_arrow = getArrow();
  arrow_phase_.update(step_time);
  arrow_changed_ = (previous_arrow != getArrow());

  const auto previous_color = getColor();
  color_phase_.update(step_time);
  color_changed_ = (previous_color != getColor());
}

const geometry_msgs::msg::Point & TrafficLight::getPosition(const TrafficLightColor & color) const
{
  if (color_positions_.count(color) == 0) {
    THROW_SEMANTIC_ERROR("target color does not exist");
  }
  return color_positions_.at(color);
}

const geometry_msgs::msg::Point & TrafficLight::getPosition(const TrafficLightArrow & arrow) const
{
  if (arrow_positions_.count(arrow) == 0) {
    THROW_SEMANTIC_ERROR("target arrow does not exist");
  }
  return arrow_positions_.at(arrow);
}
}  // namespace traffic_simulator
