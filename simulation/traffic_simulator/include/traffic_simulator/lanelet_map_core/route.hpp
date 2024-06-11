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

#ifndef TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_ROUTE_HPP_
#define TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_ROUTE_HPP_

#include <lanelet2_core/geometry/Lanelet.h>

#include <geometry_msgs/msg/point.hpp>

namespace traffic_simulator
{
namespace lanelet_map_core
{
namespace route
{
auto isInRoute(const lanelet::Id lanelet_id, const lanelet::Ids & route) -> bool;

auto speedLimit(const lanelet::Ids & lanelet_ids) -> double;

auto route(
  const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id,
  const bool allow_lane_change = false) -> lanelet::Ids;

auto followingLanelets(
  const lanelet::Id lanelet_id, const lanelet::Ids & candidate_lanelet_ids,
  const double distance = 100, const bool include_self = true) -> lanelet::Ids;

auto followingLanelets(
  const lanelet::Id, const double distance = 100, const bool include_self = true) -> lanelet::Ids;

auto previousLanelets(const lanelet::Id, const double distance = 100) -> lanelet::Ids;
}  // namespace route
}  // namespace lanelet_map_core
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__UTILS__LANELET_CORE_ROUTE_HPP_
