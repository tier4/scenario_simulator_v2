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

#ifndef SIMULATION_API__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_HPP_
#define SIMULATION_API__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_HPP_

#include <simulation_api/hdmap_utils/hdmap_utils.hpp>
#include <simulation_api/traffic_lights/traffic_light.hpp>

#include <memory>
#include <unordered_map>
#include <vector>
#include <utility>

namespace simulation_api
{
class TrafficLightManager
{
public:
  explicit TrafficLightManager(std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr);
  void setColorPhase(
    std::int64_t lanelet_id,
    const std::vector<std::pair<double, TrafficLightColor>> & phase,
    double time_offset = 0);
  void setArrowPhase(
    std::int64_t lanelet_id,
    const std::vector<std::pair<double, TrafficLightArrow>> & phase,
    double time_offset = 0);
  void setColor(std::int64_t lanelet_id, TrafficLightColor color);
  void setArrow(std::int64_t lanelet_id, TrafficLightArrow arrow);
  void update(double step_time);
  TrafficLightArrow getArrow(std::int64_t lanelet_id) const;
  TrafficLightColor getColor(std::int64_t lanelet_id) const;

private:
  std::unordered_map<std::int64_t, std::shared_ptr<TrafficLight>> traffic_lights_;
};
}  // namespace simulation_api

#endif  // SIMULATION_API__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_HPP_
