// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include <traffic_simulator/utils/traffic_lights.hpp>

namespace traffic_simulator
{
namespace traffic_lights
{
auto wayId(const lanelet::Id lanelet_id) -> lanelet::Id
{
  if (lanelet_core::traffic_lights::isTrafficLight(lanelet_id)) {
    return lanelet_id;
  } else {
    // lanelet::RoleName::Refers
    if (auto traffic_light_members =
          traffic_simulator::lanelet_core::traffic_lights::getTrafficLightRegulatoryElement(
            lanelet_id)
            ->getParameters<lanelet::ConstLineString3d>("refers");
        traffic_light_members.size() > 0) {
      // Note: If `lanelet_id` is a relation id, it is okay to use only one of the referred way ids.
      // This is because the output can be guaranteed for the original relation id by the way id.
      return traffic_light_members.front().id();
    } else {
      throw std::invalid_argument(
        "Given lanelet ID " + std::to_string(lanelet_id) + " is neither relation id nor way id.");
    }
  }
}

auto trafficLightsIds(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  lanelet::Ids traffic_lights_ids;
  if (lanelet_core::traffic_lights::isTrafficLightRegulatoryElement(lanelet_id)) {
    for (auto && traffic_light :
         lanelet_core::traffic_lights::getTrafficLightRegulatoryElement(lanelet_id)
           ->trafficLights()) {
      traffic_lights_ids.emplace_back(traffic_light.id());
    }
  } else if (lanelet_core::traffic_lights::isTrafficLight(lanelet_id)) {
    traffic_lights_ids.emplace_back(lanelet_id);
  } else {
    throw std::invalid_argument(
      "Given lanelet ID " + std::to_string(lanelet_id) +
      " is neither a traffic light ID not a traffic relation ID.");
  }
  return traffic_lights_ids;
}
}  // namespace traffic_lights
}  // namespace traffic_simulator
