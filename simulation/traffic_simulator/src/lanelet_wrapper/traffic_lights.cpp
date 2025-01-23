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

#include <traffic_simulator/lanelet_wrapper/traffic_lights.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
namespace traffic_lights
{
auto trafficLightStopLinesPoints(const lanelet::Id traffic_light_id)
  -> std::vector<std::vector<Point>>
{
  std::vector<std::vector<Point>> stop_lines;
  for (const auto & traffic_light : toAutowareTrafficLights(traffic_light_id)) {
    auto & current_stop_line = stop_lines.emplace_back();
    if (const auto & lanelet_stop_line = traffic_light->stopLine()) {
      for (const auto & point : lanelet_stop_line.value()) {
        current_stop_line.push_back(
          geometry_msgs::build<Point>().x(point.x()).y(point.y()).z(point.z()));
      }
    }
  }
  return stop_lines;
}

auto trafficLightIdsOnPath(const lanelet::Ids & route_lanelets) -> lanelet::Ids
{
  lanelet::Ids traffic_light_ids;
  for (const auto & autoware_traffic_lights : autowareTrafficLightsOnPath(route_lanelets)) {
    for (const auto & three_light_bulbs : autoware_traffic_lights->lightBulbs()) {
      if (three_light_bulbs.hasAttribute("traffic_light_id")) {
        if (const auto & traffic_light_id = three_light_bulbs.attribute("traffic_light_id").asId();
            traffic_light_id) {
          traffic_light_ids.push_back(traffic_light_id.value());
        }
      }
    }
  }
  return traffic_light_ids;
}

auto toAutowareTrafficLights(const lanelet::Id traffic_light_id)
  -> std::vector<lanelet::AutowareTrafficLightConstPtr>
{
  auto areBulbsAssignedToTrafficLight =
    [&traffic_light_id](const auto & red_yellow_green_bulbs) -> bool {
    return red_yellow_green_bulbs.hasAttribute("traffic_light_id") and
           red_yellow_green_bulbs.attribute("traffic_light_id").asId() and
           red_yellow_green_bulbs.attribute("traffic_light_id").asId().value() == traffic_light_id;
  };

  std::vector<lanelet::AutowareTrafficLightConstPtr> autoware_traffic_lights;
  const auto & all_lanelets = lanelet::utils::query::laneletLayer(LaneletWrapper::map());
  for (const auto & autoware_traffic_light :
       lanelet::utils::query::autowareTrafficLights(all_lanelets)) {
    if (const auto & light_bulbs = autoware_traffic_light->lightBulbs();
        std::any_of(light_bulbs.cbegin(), light_bulbs.cend(), areBulbsAssignedToTrafficLight)) {
      autoware_traffic_lights.push_back(autoware_traffic_light);
    }
  }

  if (!autoware_traffic_lights.empty()) {
    return autoware_traffic_lights;
  } else {
    THROW_SEMANTIC_ERROR(
      traffic_light_id,
      " lanelet does not have any regulatory elements with light bulbs assigned!");
  }
}

auto autowareTrafficLightsOnPath(const lanelet::Ids & lanelet_ids)
  -> std::vector<lanelet::AutowareTrafficLightConstPtr>
{
  std::vector<lanelet::AutowareTrafficLightConstPtr> autoware_traffic_lights;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto & lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
    for (const auto & traffic_light :
         lanelet.regulatoryElementsAs<const lanelet::autoware::AutowareTrafficLight>()) {
      autoware_traffic_lights.push_back(traffic_light);
    }
  }
  return autoware_traffic_lights;
}
}  // namespace traffic_lights
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
