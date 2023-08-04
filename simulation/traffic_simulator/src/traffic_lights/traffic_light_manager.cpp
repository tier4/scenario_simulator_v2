// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <iterator>
#include <memory>
#include <string>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <type_traits>
#include <utility>
#include <vector>

namespace traffic_simulator
{
TrafficLightManager::TrafficLightManager(const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap)
: hdmap_(hdmap)
{
}

auto TrafficLightManager::hasAnyLightChanged() -> bool
{
  return true;
  // return std::any_of(
  //   std::begin(getTrafficLights()), std::end(getTrafficLights()), [](auto && id_and_traffic_light) {
  //     return id_and_traffic_light.second.colorChanged() or
  //            id_and_traffic_light.second.arrowChanged();
  //   });
}

auto TrafficLightManager::getTrafficLight(const LaneletID traffic_light_id) -> TrafficLight &
{
  if (auto iter = traffic_lights_.find(traffic_light_id); iter != std::end(traffic_lights_)) {
    return iter->second;
  } else {
    traffic_lights_.emplace(
      std::piecewise_construct, std::forward_as_tuple(traffic_light_id),
      std::forward_as_tuple(traffic_light_id, *hdmap_));
    return traffic_lights_.at(traffic_light_id);
  }
}

auto TrafficLightManager::getTrafficLights() const -> const TrafficLightMap &
{
  return traffic_lights_;
}

auto TrafficLightManager::getTrafficLights() -> TrafficLightMap & { return traffic_lights_; }

auto TrafficLightManager::getTrafficLights(const LaneletID lanelet_id)
  -> std::vector<std::reference_wrapper<TrafficLight>>
{
  std::vector<std::reference_wrapper<TrafficLight>> traffic_lights;

  if (hdmap_->isTrafficRelation(lanelet_id)) {
    for (auto && traffic_light : hdmap_->getTrafficRelation(lanelet_id)->trafficLights()) {
      traffic_lights.emplace_back(getTrafficLight(traffic_light.id()));
    }
  } else if (hdmap_->isTrafficLight(lanelet_id)) {
    traffic_lights.emplace_back(getTrafficLight(lanelet_id));
  } else {
    throw common::scenario_simulator_exception::Error(
      "Given lanelet ID ", lanelet_id, " is neither a traffic light ID not a traffic relation ID.");
  }

  return traffic_lights;
}

}  // namespace traffic_simulator
