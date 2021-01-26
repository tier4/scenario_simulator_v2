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
#include <vector>
#include <utility>

namespace simulation_api
{
TrafficLightManager::TrafficLightManager(std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr)
{
  traffic_lights_ = {};
  const auto ids = hdmap_utils_ptr->getTrafficLightIds();
  for (const auto id : ids) {
    std::shared_ptr<TrafficLight> light_ptr = std::make_shared<TrafficLight>(id);
    traffic_lights_.insert({id, light_ptr});
  }
}

void TrafficLightManager::update(double step_time)
{
  for (const auto light : traffic_lights_) {
    light.second->update(step_time);
  }
}

TrafficLightArrow TrafficLightManager::getArrow(std::int64_t lanelet_id) const
{
  if (traffic_lights_.count(lanelet_id) == 0) {
    throw SimulationRuntimeError("lanelet id does not match");
  }
  return traffic_lights_.at(lanelet_id)->getArrow();
}

TrafficLightColor TrafficLightManager::getColor(std::int64_t lanelet_id) const
{
  if (traffic_lights_.count(lanelet_id) == 0) {
    throw SimulationRuntimeError("lanelet id does not match");
  }
  return traffic_lights_.at(lanelet_id)->getColor();
}

void TrafficLightManager::printState(std::int64_t lanelet_id)
{
  if (traffic_lights_.count(lanelet_id) == 0) {
    throw SimulationRuntimeError("lanelet id does not match");
  }
  traffic_lights_.at(lanelet_id)->printState();
}
}  // namespace simulation_api
