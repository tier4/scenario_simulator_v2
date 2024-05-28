

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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_TRAFFIC_LIGHTS_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_TRAFFIC_LIGHTS_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <optional>
#include <lanelet_map/lanelet_map.hpp>


namespace traffic_simulator
{
namespace lanelet2
{
namespace traffic_lights
{
auto getTrafficSignRegulatoryElementsOnPath(const lanelet::Ids & lanelet_ids)
  -> std::vector<std::shared_ptr<const lanelet::TrafficSign>>;

auto getTrafficLightIdsOnPath(const lanelet::Ids & route_lanelets) -> lanelet::Ids;

auto getTrafficLightStopLinesPoints(const lanelet::Id traffic_light_id)
  -> std::vector<std::vector<geometry_msgs::msg::Point>>;

auto getTrafficLights(const lanelet::Id traffic_light_id)
  -> std::vector<lanelet::AutowareTrafficLightConstPtr>;

auto isTrafficLightRegulatoryElement(const lanelet::Id lanelet_id) -> bool;

auto getTrafficLightRegulatoryElement(const lanelet::Id lanelet_id) -> lanelet::TrafficLight::Ptr;

auto isTrafficLight(const lanelet::Id lanelet_id) -> bool;

auto getTrafficLightBulbPosition(const lanelet::Id traffic_light_id, const std::string & color_name)
  -> std::optional<geometry_msgs::msg::Point>;

auto getTrafficLightRegulatoryElementIDsFromTrafficLight(const lanelet::Id traffic_light_way_id)
  -> lanelet::Ids;

// private for traffic_lights namespace
namespace
{
auto getTrafficLightRegulatoryElementsOnPath(const lanelet::Ids & lanelet_ids)
  -> std::vector<std::shared_ptr<const lanelet::autoware::AutowareTrafficLight>>;
}  // namespace
}  // namespace traffic_lights
}  // namespace lanelet2
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_TRAFFIC_LIGHTS_HPP_
