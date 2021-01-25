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

#include <simulation_api/entity/exception.hpp>

namespace simulation_api
{
TrafficLight::TrafficLight(std::int64_t id, bool contain_arrow)
: id(id), contain_arrow(contain_arrow)
{
  color_phase_.emplace_back(
    std::make_pair(std::numeric_limits<double>::infinity(), TrafficLightColor::NONE) );
  arrow_phase_ = {};
  elapsed_time_in_color_phase_ = 0;
}

TrafficLight::TrafficLight(
  std::int64_t id, std::vector<std::pair<double,
  TrafficLightColor>> color_phase, bool contain_arrow)
: id(id), contain_arrow(contain_arrow)
{
  color_phase_ = color_phase;
  arrow_phase_ = {};
  elapsed_time_in_color_phase_ = 0;
}

TrafficLight::TrafficLight(
  std::int64_t id,
  std::vector<std::pair<double, TrafficLightColor>> color_phase,
  std::vector<std::pair<double, TrafficLightArrow>> arrow_phase)
: id(id), contain_arrow(true)
{
  color_phase_ = color_phase;
  arrow_phase_ = arrow_phase;
  elapsed_time_in_color_phase_ = 0;
}

void TrafficLight::setColorPhase(
  std::vector<std::pair<double,
  TrafficLightColor>> color_phase, double time_offset)
{
  color_phase_ = color_phase;
  elapsed_time_in_color_phase_ = time_offset;
}

void TrafficLight::setArrowPhase(
  std::vector<std::pair<double,
  TrafficLightArrow>> arrow_phase, double time_offset)
{
  if (!contain_arrow) {
    throw SimulationRuntimeError("traffic light " + std::to_string(id) + " does not contain arrow");
  }
  arrow_phase_ = arrow_phase;
  elapsed_time_in_arrow_phase_ = time_offset;
}

void TrafficLight::setColor(TrafficLightColor color)
{
  color_phase_ = {};
  color_phase_.emplace_back(std::make_pair(std::numeric_limits<double>::infinity(), color) );
  elapsed_time_in_color_phase_ = 0;
}

void TrafficLight::setArrow(TrafficLightArrow arrow)
{
  if (!contain_arrow) {
    throw SimulationRuntimeError("traffic light " + std::to_string(id) + " does not contain arrow");
  }
  arrow_phase_ = {};
  arrow_phase_.emplace_back(std::make_pair(std::numeric_limits<double>::infinity(), arrow) );
  elapsed_time_in_arrow_phase_ = 0;
}

double TrafficLight::getColorPhaseLength() const
{
  double ret = 0;
  for (const auto seq : color_phase_) {
    ret = ret + seq.first;
  }
  return ret;
}

double TrafficLight::getArrowPhaseLength() const
{
  if (!contain_arrow) {
    throw SimulationRuntimeError("traffic light " + std::to_string(id) + " does not contain arrow");
  }
  double ret = 0;
  for (const auto seq : arrow_phase_) {
    ret = ret + seq.first;
  }
  return ret;
}

TrafficLightArrow TrafficLight::getArrow() const
{
  if (!contain_arrow) {
    throw SimulationRuntimeError("traffic light " + std::to_string(id) + " does not contain arrow");
  }
  double t = 0;
  for (const auto seq : arrow_phase_) {
    t = t + seq.first;
    if (t > elapsed_time_in_arrow_phase_) {
      return seq.second;
    }
  }
  throw SimulationRuntimeError("failed to get arrow in phase");
}

TrafficLightColor TrafficLight::getColor() const
{
  double t = 0;
  for (const auto seq : color_phase_) {
    t = t + seq.first;
    if (t > elapsed_time_in_color_phase_) {
      return seq.second;
    }
  }
  throw SimulationRuntimeError("failed to get color in phase");
}

void TrafficLight::update(double step_time)
{
  if (contain_arrow) {
    elapsed_time_in_arrow_phase_ = elapsed_time_in_arrow_phase_ + step_time;
    if (elapsed_time_in_color_phase_ > getArrowPhaseLength()) {
      elapsed_time_in_color_phase_ = elapsed_time_in_color_phase_ - getArrowPhaseLength();
    }
  }
  elapsed_time_in_color_phase_ = elapsed_time_in_color_phase_ + step_time;
  if (elapsed_time_in_color_phase_ > getArrowPhaseLength()) {
    elapsed_time_in_color_phase_ = elapsed_time_in_color_phase_ - getArrowPhaseLength();
  }
}
}
