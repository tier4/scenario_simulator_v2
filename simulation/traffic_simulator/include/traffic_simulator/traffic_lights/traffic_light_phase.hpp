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
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <utility>
#include <vector>

namespace traffic_simulator
{
template <typename T>
class TrafficLightPhase
{
public:
  TrafficLightPhase() = default;

  explicit TrafficLightPhase(const std::vector<std::pair<double, T>> & phase) : phase_(phase) {}

  double getPhaseDuration() const
  {
    if (phase_.empty()) {
      THROW_SEMANTIC_ERROR("phase is empty");
    } else {
      return std::accumulate(
        std::begin(phase_), std::end(phase_), 0,
        [](const auto & lhs, const auto & rhs) { return lhs + rhs.first; });
    }
  }

  const T getState() const
  {
    if (phase_.empty()) {
      THROW_SEMANTIC_ERROR("phase is empty");
    }
    double t = 0;
    for (const auto p : phase_) {
      t = t + p.first;
      if (t > elapsed_time_) {
        return p.second;
      }
    }
    THROW_SIMULATION_ERROR("failed to get state of the traffic light, time does not match");
  }

  void update(double step_time)
  {
    if (phase_.empty()) {
      elapsed_time_ = 0;
      return;
    }
    elapsed_time_ = elapsed_time_ + step_time;
    if (elapsed_time_ > getPhaseDuration()) {
      elapsed_time_ = elapsed_time_ - getPhaseDuration();
    }
  }

  void setState(const T & state)
  {
    phase_.clear();
    phase_.emplace_back(std::numeric_limits<double>::infinity(), state);
    elapsed_time_ = 0;
  }

  void setPhase(const std::vector<std::pair<double, T>> & phase, double time_offset = 0)
  {
    phase_ = phase;
    elapsed_time_ = time_offset;
  }

  const auto & getPhase() const noexcept { return phase_; }

private:
  std::vector<std::pair<double, T>> phase_;
  double elapsed_time_;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_PHASE_HPP_
