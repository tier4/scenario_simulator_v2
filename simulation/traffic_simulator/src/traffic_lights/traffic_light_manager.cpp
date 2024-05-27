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

#include <lanelet2_core/geometry/Lanelet.h>

#include <iterator>
#include <memory>
#include <string>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/lanelet/traffic_lights.hpp>
#include <type_traits>
#include <utility>
#include <vector>

namespace traffic_simulator
{
TrafficLightManager::TrafficLightManager() {}

auto TrafficLightManager::hasAnyLightChanged() -> bool
{
  return true;
  // return std::any_of(
  //   std::begin(getTrafficLights()), std::end(getTrafficLights()), [](auto && id_and_traffic_light) {
  //     return id_and_traffic_light.second.colorChanged() or
  //            id_and_traffic_light.second.arrowChanged();
  //   });
}

auto TrafficLightManager::getTrafficLight(const lanelet::Id traffic_light_id) -> TrafficLight &
{
  if (auto iter = traffic_lights_.find(traffic_light_id); iter != std::end(traffic_lights_)) {
    return iter->second;
  } else {
    traffic_lights_.emplace(
      std::piecewise_construct, std::forward_as_tuple(traffic_light_id),
      std::forward_as_tuple(traffic_light_id));
    return traffic_lights_.at(traffic_light_id);
  }
}

auto TrafficLightManager::getTrafficLightIds() const -> const lanelet::Ids
{
  lanelet::Ids traffic_light_ids;
  for (const auto & traffic_light_ : getTrafficLights()) {
    traffic_light_ids.push_back(traffic_light_.first);
  }
  return traffic_light_ids;
}

auto TrafficLightManager::getTrafficLights() const -> const TrafficLightMap &
{
  return traffic_lights_;
}

auto TrafficLightManager::getTrafficLights() -> TrafficLightMap & { return traffic_lights_; }

auto TrafficLightManager::getTrafficLights(const lanelet::Id lanelet_id)
  -> std::vector<std::reference_wrapper<TrafficLight>>
{
  std::vector<std::reference_wrapper<TrafficLight>> traffic_lights;

  if (lanelet2::traffic_lights::isTrafficLightRegulatoryElement(lanelet_id)) {
    for (auto && traffic_light :
         traffic_simulator::lanelet2::traffic_lights::getTrafficLightRegulatoryElement(lanelet_id)
           ->trafficLights()) {
      traffic_lights.emplace_back(getTrafficLight(traffic_light.id()));
    }
  } else if (lanelet2::traffic_lights::isTrafficLight(lanelet_id)) {
    traffic_lights.emplace_back(getTrafficLight(lanelet_id));
  } else {
    throw common::scenario_simulator_exception::Error(
      "Given lanelet ID ", lanelet_id, " is neither a traffic light ID not a traffic relation ID.");
  }

  return traffic_lights;
}

auto TrafficLightManager::getDistanceToActiveTrafficLightStopLine(
  const lanelet::Ids & route_lanelets, const math::geometry::CatmullRomSplineInterface & spline)
  -> std::optional<double>
{
  const auto traffic_light_ids =
    traffic_simulator::lanelet2::traffic_lights::getTrafficLightIdsOnPath(route_lanelets);
  if (traffic_light_ids.empty()) {
    return std::nullopt;
  }
  std::set<double> collision_points = {};
  for (const auto id : traffic_light_ids) {
    using Color = traffic_simulator::TrafficLight::Color;
    using Status = traffic_simulator::TrafficLight::Status;
    using Shape = traffic_simulator::TrafficLight::Shape;
    if (auto && traffic_light = getTrafficLight(id);
        traffic_light.contains(Color::red, Status::solid_on, Shape::circle) or
        traffic_light.contains(Color::yellow, Status::solid_on, Shape::circle)) {
      const auto collision_point =
        traffic_simulator::distance::distanceToTrafficLightStopLine(spline, id);
      if (collision_point) {
        collision_points.insert(collision_point.value());
      }
    }
  }
  if (collision_points.empty()) {
    return std::nullopt;
  }
  return *collision_points.begin();
}

auto TrafficLightManager::generateUpdateTrafficLightsRequest()
  -> simulation_api_schema::UpdateTrafficLightsRequest
{
  simulation_api_schema::UpdateTrafficLightsRequest update_traffic_lights_request;
  for (auto && [lanelet_id, traffic_light] : traffic_lights_) {
    *update_traffic_lights_request.add_states() =
      static_cast<simulation_api_schema::TrafficSignal>(traffic_light);
  }
  return update_traffic_lights_request;
}

}  // namespace traffic_simulator
