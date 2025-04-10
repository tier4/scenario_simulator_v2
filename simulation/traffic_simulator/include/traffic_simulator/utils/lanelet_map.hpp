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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_MAP_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_MAP_HPP_

#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <traffic_simulator/color_utils/color_utils.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_wrapper.hpp>
#include <traffic_simulator/utils/pose.hpp>

namespace traffic_simulator
{
inline namespace lanelet_map
{
using Point = geometry_msgs::msg::Point;
using Pose = geometry_msgs::msg::Pose;

template <typename... Ts>
inline auto activate(Ts &&... xs)
{
  return lanelet_wrapper::LaneletWrapper::activate(std::forward<decltype(xs)>(xs)...);
}

auto laneletLength(const lanelet::Id lanelet_id) -> double;

template <typename... Ts>
inline auto laneletAltitude(Ts &&... xs)
{
  return lanelet_wrapper::lanelet_map::laneletAltitude(std::forward<decltype(xs)>(xs)...);
}

/// @brief Calculates all poses on the map that have no next lanelet (dead ends)
/// @return A vector of final poses and their corresponding lanelet IDs
auto noNextLaneletPoses() -> std::vector<std::pair<lanelet::Id, Pose>>;
}  // namespace lanelet_map
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_MAP_HPP_
