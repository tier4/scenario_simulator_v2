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

#include <simulation_api/traffic_lights/traffic_light_manager.hpp>
#include <simulation_api/entity/exception.hpp>

#include <memory>

namespace simulation_api
{
TrafficLightManager::TrafficLightManager(std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr)
{
  traffic_lights_ = {};
}

void TrafficLightManager::setColorPhase(
  std::int64_t lanelet_id,
  const std::vector<std::pair<double, TrafficLightColor>> & phase,
  double time_offset)
{
  if (traffic_lights_.count(lanelet_id) == 0) {
    throw SimulationRuntimeError("lanelet id does not match");
  }
  traffic_lights_[lanelet_id]->setColorPhase(phase, time_offset);
}

void TrafficLightManager::setArrowPhase(
  std::int64_t lanelet_id,
  const std::vector<std::pair<double, TrafficLightArrow>> & phase,
  double time_offset)
{
  if (traffic_lights_.count(lanelet_id) == 0) {
    throw SimulationRuntimeError("lanelet id does not match");
  }
  traffic_lights_[lanelet_id]->setArrowPhase(phase, time_offset);
}

void TrafficLightManager::setColor(std::int64_t lanelet_id, TrafficLightColor color)
{
  if (traffic_lights_.count(lanelet_id) == 0) {
    throw SimulationRuntimeError("lanelet id does not match");
  }
  traffic_lights_[lanelet_id]->setColor(color);
}

void TrafficLightManager::setArrow(std::int64_t lanelet_id, TrafficLightArrow arrow)
{
  if (traffic_lights_.count(lanelet_id) == 0) {
    throw SimulationRuntimeError("lanelet id does not match");
  }
  traffic_lights_[lanelet_id]->setArrow(arrow);
}
}  // namespace simulation_api
