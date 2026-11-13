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

#ifndef TRAFFIC_SIMULATOR__UTILS__TRAFFIC_LIGHTS_HPP_
#define TRAFFIC_SIMULATOR__UTILS__TRAFFIC_LIGHTS_HPP_

#include <traffic_simulator/lanelet_wrapper/traffic_lights.hpp>

namespace traffic_simulator
{
namespace traffic_lights
{
auto wayIds(const lanelet::Id lanelet_id) -> lanelet::Ids;

auto wayId(const lanelet::Id lanelet_id) -> lanelet::Id;

auto trafficLightsIds(const lanelet::Id lanelet_id) -> lanelet::Ids;

template <typename... Ts>
inline auto bulbPosition(Ts &&... xs)
{
  return lanelet_wrapper::traffic_lights::trafficLightBulbPosition(
    std::forward<decltype(xs)>(xs)...);
}

template <typename... Ts>
inline auto trafficLightIdsOnPath(Ts &&... xs)
{
  return lanelet_wrapper::traffic_lights::trafficLightIdsOnPath(std::forward<decltype(xs)>(xs)...);
}

template <typename... Ts>
inline auto trafficLightRegulatoryElementIdsFromTrafficLightId(Ts &&... xs)
{
  return lanelet_wrapper::traffic_lights::trafficLightRegulatoryElementIDsFromTrafficLight(
    std::forward<decltype(xs)>(xs)...);
}
}  // namespace traffic_lights
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__TRAFFIC_LIGHTS_HPP_
