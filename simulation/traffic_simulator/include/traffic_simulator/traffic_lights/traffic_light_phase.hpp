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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_PHASE_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_PHASE_HPP_

#include <limits>
#include <traffic_simulator/entity/exception.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <utility>
#include <vector>

namespace traffic_simulator
{
template <typename T>
class TrafficLightPhase
{
public:
  TrafficLightPhase() { phase_ = {}; }
  explicit TrafficLightPhase(const std::vector<std::pair<double, T>> & phase) : phase_(phase) {}
  double getPhaseLength() const
  {
    if (phase_.empty()) {
      throw SimulationRuntimeError("phase is empty");
    }
    double l = 0;
    for (const auto p : phase_) {
      l = l + p.first;
    }
    return l;
  }
  const T getState() const
  {
    if (phase_.empty()) {
      throw SimulationRuntimeError("phase is empty");
    }
    double t = 0;
    for (const auto p : phase_) {
      t = t + p.first;
      if (t > elapsed_time_) {
        return p.second;
      }
    }
    throw SimulationRuntimeError("failed to get state in phase");
  }
  void update(double step_time)
  {
    if (phase_.empty()) {
      elapsed_time_ = 0;
      return;
    }
    elapsed_time_ = elapsed_time_ + step_time;
    if (elapsed_time_ > getPhaseLength()) {
      elapsed_time_ = elapsed_time_ - getPhaseLength();
    }
  }
  void setState(const T & state)
  {
    phase_ = {};
    phase_.emplace_back(std::make_pair(std::numeric_limits<double>::infinity(), state));
    elapsed_time_ = 0;
  }
  void setPhase(const std::vector<std::pair<double, T>> & phase, double time_offset = 0)
  {
    phase_ = phase;
    elapsed_time_ = time_offset;
  }
  const std::vector<std::pair<double, T>> getPhase() const { return phase_; }

private:
  std::vector<std::pair<double, T>> phase_;
  double elapsed_time_;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_PHASE_HPP_
