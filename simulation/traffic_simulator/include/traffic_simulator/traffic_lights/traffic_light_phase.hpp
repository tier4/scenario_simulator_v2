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

  explicit TrafficLightPhase(const std::vector<std::pair<double, T>> & phase) : state(phase.front())
  {
  }

  T state;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_PHASE_HPP_
