

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

#ifndef TRAFFIC_SIMULATOR__LANELET_WRAPPER_TRAFFIC_LIGHTS_HPP_
#define TRAFFIC_SIMULATOR__LANELET_WRAPPER_TRAFFIC_LIGHTS_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <optional>
#include <traffic_simulator/lanelet_wrapper/lanelet_wrapper.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
namespace traffic_lights
{
using Point = geometry_msgs::msg::Point;

auto isTrafficLight(const lanelet::Id lanelet_id) -> bool;

auto isTrafficLightRegulatoryElement(const lanelet::Id lanelet_id) -> bool;

auto toTrafficLightRegulatoryElement(const lanelet::Id traffic_light_regulatory_element_id)
  -> lanelet::TrafficLight::Ptr;

auto toAutowareTrafficLights(const lanelet::Id traffic_light_id)
  -> std::vector<lanelet::AutowareTrafficLightConstPtr>;

auto trafficLightBulbPosition(const lanelet::Id traffic_light_id, const std::string & color_name)
  -> std::optional<Point>;

auto trafficLightStopLinesPoints(const lanelet::Id traffic_light_id)
  -> std::vector<std::vector<Point>>;

auto trafficLightRegulatoryElementIDsFromTrafficLight(const lanelet::Id traffic_light_way_id)
  -> lanelet::Ids;

// On path
auto autowareTrafficLightsOnPath(const lanelet::Ids & lanelet_ids)
  -> std::vector<lanelet::AutowareTrafficLightConstPtr>;

auto trafficLightIdsOnPath(const lanelet::Ids & route_lanelets) -> lanelet::Ids;

auto trafficSignsOnPath(const lanelet::Ids & lanelet_ids)
  -> std::vector<std::shared_ptr<const lanelet::TrafficSign>>;
}  // namespace traffic_lights
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__LANELET_WRAPPER_TRAFFIC_LIGHTS_HPP_
