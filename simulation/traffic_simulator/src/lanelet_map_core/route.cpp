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

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <lanelet2_extension/utility/utilities.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/lanelet_map_core/lanelet_map.hpp>
#include <traffic_simulator/lanelet_map_core/lanelet_map_core.hpp>
#include <traffic_simulator/lanelet_map_core/route.hpp>

namespace traffic_simulator
{
namespace lanelet_map_core
{
namespace route
{
auto isInRoute(const lanelet::Id lanelet_id, const lanelet::Ids & route) -> bool
{
  return std::find_if(route.begin(), route.end(), [lanelet_id](const auto id) {
           return lanelet_id == id;
         }) != route.end();
}

auto speedLimit(const lanelet::Ids & lanelet_ids) -> double
{
  std::vector<double> limits;
  if (lanelet_ids.empty()) {
    THROW_SEMANTIC_ERROR("size of the vector lanelet ids should be more than 1");
  }
  for (auto itr = lanelet_ids.begin(); itr != lanelet_ids.end(); itr++) {
    const auto lanelet = LaneletMapCore::map()->laneletLayer.get(*itr);
    const auto limit = LaneletMapCore::trafficRulesVehicle()->speedLimit(lanelet);
    limits.push_back(lanelet::units::KmHQuantity(limit.speedLimit).value() / 3.6);
  }
  return *std::min_element(limits.begin(), limits.end());
}

auto route(
  const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id, const bool allow_lane_change)
  -> lanelet::Ids
{
  if (LaneletMapCore::routeCache().exists(from_lanelet_id, to_lanelet_id, allow_lane_change)) {
    return LaneletMapCore::routeCache().getRoute(from_lanelet_id, to_lanelet_id, allow_lane_change);
  }
  lanelet::Ids ids;
  const auto lanelet = LaneletMapCore::map()->laneletLayer.get(from_lanelet_id);
  const auto to_lanelet = LaneletMapCore::map()->laneletLayer.get(to_lanelet_id);
  lanelet::Optional<lanelet::routing::Route> route =
    LaneletMapCore::vehicleRoutingGraph()->getRoute(lanelet, to_lanelet, 0, allow_lane_change);
  if (!route) {
    LaneletMapCore::routeCache().appendData(from_lanelet_id, to_lanelet_id, allow_lane_change, ids);
    return ids;
  }
  lanelet::routing::LaneletPath shortest_path = route->shortestPath();
  if (shortest_path.empty()) {
    LaneletMapCore::routeCache().appendData(from_lanelet_id, to_lanelet_id, allow_lane_change, ids);
    return ids;
  }
  for (auto lane_itr = shortest_path.begin(); lane_itr != shortest_path.end(); lane_itr++) {
    ids.push_back(lane_itr->id());
  }
  LaneletMapCore::routeCache().appendData(from_lanelet_id, to_lanelet_id, allow_lane_change, ids);
  return ids;
}

auto followingLanelets(
  const lanelet::Id lanelet_id, const lanelet::Ids & candidate_lanelet_ids, const double distance,
  const bool include_self) -> lanelet::Ids
{
  if (candidate_lanelet_ids.empty()) {
    return {};
  }
  lanelet::Ids ids;
  double total_distance = 0.0;
  bool found = false;
  for (const auto id : candidate_lanelet_ids) {
    if (found) {
      ids.emplace_back(id);
      total_distance = total_distance + lanelet_map::laneletLength(id);
      if (total_distance > distance) {
        return ids;
      }
    }
    if (id == lanelet_id) {
      found = true;
      if (include_self) {
        ids.push_back(id);
      }
    }
  }
  if (!found) {
    THROW_SEMANTIC_ERROR("lanelet id does not match");
  }
  if (total_distance > distance) {
    return ids;
  }
  // clang-format off
  return ids + followingLanelets(
    candidate_lanelet_ids[candidate_lanelet_ids.size() - 1],
    distance - total_distance, false);
  // clang-format on
}

auto followingLanelets(
  const lanelet::Id lanelet_id, const double distance, const bool include_self) -> lanelet::Ids
{
  lanelet::Ids ret;
  double total_distance = 0.0;
  if (include_self) {
    ret.push_back(lanelet_id);
  }
  lanelet::Id end_lanelet_id = lanelet_id;
  while (total_distance < distance) {
    if (const auto straight_ids = lanelet_map::nextLaneletIds(end_lanelet_id, "straight");
        !straight_ids.empty()) {
      total_distance = total_distance + lanelet_map::laneletLength(straight_ids[0]);
      ret.push_back(straight_ids[0]);
      end_lanelet_id = straight_ids[0];
      continue;
    } else if (const auto ids = lanelet_map::nextLaneletIds(end_lanelet_id); ids.size() != 0) {
      total_distance = total_distance + lanelet_map::laneletLength(ids[0]);
      ret.push_back(ids[0]);
      end_lanelet_id = ids[0];
      continue;
    } else {
      break;
    }
  }
  return ret;
}

auto previousLanelets(const lanelet::Id lanelet_id, const double distance) -> lanelet::Ids
{
  lanelet::Ids ret;
  double total_distance = 0.0;
  ret.push_back(lanelet_id);
  while (total_distance < distance) {
    auto ids = lanelet_map::previousLaneletIds(lanelet_id, "straight");
    if (ids.size() != 0) {
      total_distance = total_distance + lanelet_map::laneletLength(ids[0]);
      ret.push_back(ids[0]);
      continue;
    } else {
      auto else_ids = lanelet_map::previousLaneletIds(lanelet_id);
      if (else_ids.size() != 0) {
        total_distance = total_distance + lanelet_map::laneletLength(else_ids[0]);
        ret.push_back(else_ids[0]);
        continue;
      } else {
        break;
      }
    }
  }
  return ret;
}
}  // namespace route
}  // namespace lanelet_map_core
}  // namespace traffic_simulator
