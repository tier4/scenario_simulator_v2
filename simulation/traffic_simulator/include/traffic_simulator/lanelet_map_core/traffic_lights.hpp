

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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_TRAFFIC_LIGHTS_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_TRAFFIC_LIGHTS_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <optional>
#include <traffic_simulator/lanelet_map_core/lanelet_map_core.hpp>

namespace traffic_simulator
{
namespace lanelet_map_core
{
namespace traffic_lights
{
using Point = geometry_msgs::msg::Point;

auto isTrafficLight(const lanelet::Id lanelet_id) -> bool;

auto isTrafficLightRegulatoryElement(const lanelet::Id lanelet_id) -> bool;

auto getTrafficLights(const lanelet::Id traffic_light_id)
  -> std::vector<lanelet::AutowareTrafficLightConstPtr>;

auto getTrafficLightBulbPosition(const lanelet::Id traffic_light_id, const std::string & color_name)
  -> std::optional<Point>;

auto getTrafficLightStopLinesPoints(const lanelet::Id traffic_light_id)
  -> std::vector<std::vector<Point>>;

auto getTrafficLightRegulatoryElement(const lanelet::Id lanelet_id) -> lanelet::TrafficLight::Ptr;

auto getTrafficLightRegulatoryElementIDsFromTrafficLight(const lanelet::Id traffic_light_way_id)
  -> lanelet::Ids;

// On path
auto getTrafficLightIdsOnPath(const lanelet::Ids & route_lanelets) -> lanelet::Ids;

auto getTrafficSignRegulatoryElementsOnPath(const lanelet::Ids & lanelet_ids)
  -> std::vector<std::shared_ptr<const lanelet::TrafficSign>>;

// private
auto getTrafficLightRegulatoryElementsOnPath(const lanelet::Ids & lanelet_ids)
  -> std::vector<std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>>;
}  // namespace traffic_lights
}  // namespace lanelet_map_core
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_TRAFFIC_LIGHTS_HPP_
