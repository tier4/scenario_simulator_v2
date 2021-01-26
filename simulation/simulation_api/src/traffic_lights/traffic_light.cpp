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

#include <simulation_api/traffic_lights/traffic_light.hpp>

#include <vector>
#include <limits>
#include <utility>

namespace simulation_api
{
TrafficLight::TrafficLight(std::int64_t id)
: id(id)
{
  color_phase_.setState(TrafficLightColor::NONE);
  arrow_phase_.setState(TrafficLightArrow::NONE);
}

void TrafficLight::setColorPhase(
  const std::vector<std::pair<double, TrafficLightColor>> & phase,
  double time_offset)
{
  color_phase_.setPhase(phase, time_offset);
}

void TrafficLight::setArrowPhase(
  const std::vector<std::pair<double, TrafficLightArrow>> & phase,
  double time_offset)
{
  arrow_phase_.setPhase(phase, time_offset);
}

void TrafficLight::setColor(TrafficLightColor color)
{
  color_phase_.setState(color);
}

void TrafficLight::setArrow(TrafficLightArrow arrow)
{
  arrow_phase_.setState(arrow);
}

double TrafficLight::getColorPhaseLength() const
{
  return color_phase_.getPhaseLength();
}

double TrafficLight::getArrowPhaseLength() const
{
  return arrow_phase_.getPhaseLength();
}

TrafficLightArrow TrafficLight::getArrow() const
{
  return arrow_phase_.getState();
}

TrafficLightColor TrafficLight::getColor() const
{
  return color_phase_.getState();
}

void TrafficLight::update(double step_time)
{
  arrow_phase_.update(step_time);
  color_phase_.update(step_time);
}
}  // namespace simulation_api
