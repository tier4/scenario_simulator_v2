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

#include <lanelet2_core/utility/Units.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <lanelet2_extension/utility/utilities.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/utils/lanelet/memory.hpp>
#include <traffic_simulator/utils/lanelet/other.hpp>
#include <traffic_simulator/utils/lanelet/route.hpp>

namespace traffic_simulator
{
namespace lanelet2
{
namespace route
{
auto getRoute(
  const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id, bool allow_lane_change)
  -> lanelet::Ids
{
  if (Memory::routeCache().exists(from_lanelet_id, to_lanelet_id, allow_lane_change)) {
    return Memory::routeCache().getRoute(from_lanelet_id, to_lanelet_id, allow_lane_change);
  }
  lanelet::Ids ids;
  const auto lanelet = Memory::laneletMap()->laneletLayer.get(from_lanelet_id);
  const auto to_lanelet = Memory::laneletMap()->laneletLayer.get(to_lanelet_id);
  lanelet::Optional<lanelet::routing::Route> route =
    Memory::vehicleRoutingGraph()->getRoute(lanelet, to_lanelet, 0, allow_lane_change);
  if (!route) {
    Memory::routeCache().appendData(from_lanelet_id, to_lanelet_id, allow_lane_change, ids);
    return ids;
  }
  lanelet::routing::LaneletPath shortest_path = route->shortestPath();
  if (shortest_path.empty()) {
    Memory::routeCache().appendData(from_lanelet_id, to_lanelet_id, allow_lane_change, ids);
    return ids;
  }
  for (auto lane_itr = shortest_path.begin(); lane_itr != shortest_path.end(); lane_itr++) {
    ids.push_back(lane_itr->id());
  }
  Memory::routeCache().appendData(from_lanelet_id, to_lanelet_id, allow_lane_change, ids);
  return ids;
}

auto getFollowingLanelets(
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
      total_distance = total_distance + other::getLaneletLength(id);
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
  return ids + getFollowingLanelets(
    candidate_lanelet_ids[candidate_lanelet_ids.size() - 1],
    distance - total_distance, false);
  // clang-format on
}

auto getFollowingLanelets(
  const lanelet::Id lanelet_id, const double distance, const bool include_self) -> lanelet::Ids
{
  lanelet::Ids ret;
  double total_distance = 0.0;
  if (include_self) {
    ret.push_back(lanelet_id);
  }
  lanelet::Id end_lanelet_id = lanelet_id;
  while (total_distance < distance) {
    if (const auto straight_ids = other::getNextLaneletIds(end_lanelet_id, "straight");
        !straight_ids.empty()) {
      total_distance = total_distance + other::getLaneletLength(straight_ids[0]);
      ret.push_back(straight_ids[0]);
      end_lanelet_id = straight_ids[0];
      continue;
    } else if (const auto ids = other::getNextLaneletIds(end_lanelet_id); ids.size() != 0) {
      total_distance = total_distance + other::getLaneletLength(ids[0]);
      ret.push_back(ids[0]);
      end_lanelet_id = ids[0];
      continue;
    } else {
      break;
    }
  }
  return ret;
}

auto getSpeedLimit(const lanelet::Ids & lanelet_ids) -> double
{
  std::vector<double> limits;
  if (lanelet_ids.empty()) {
    THROW_SEMANTIC_ERROR("size of the vector lanelet ids should be more than 1");
  }
  for (auto itr = lanelet_ids.begin(); itr != lanelet_ids.end(); itr++) {
    const auto lanelet = Memory::laneletMap()->laneletLayer.get(*itr);
    const auto limit = Memory::trafficRulesVehicle()->speedLimit(lanelet);
    limits.push_back(lanelet::units::KmHQuantity(limit.speedLimit).value() / 3.6);
  }
  return *std::min_element(limits.begin(), limits.end());
}

auto getPreviousLanelets(const lanelet::Id lanelet_id, const double distance) -> lanelet::Ids
{
  lanelet::Ids ret;
  double total_distance = 0.0;
  ret.push_back(lanelet_id);
  while (total_distance < distance) {
    auto ids = other::getPreviousLaneletIds(lanelet_id, "straight");
    if (ids.size() != 0) {
      total_distance = total_distance + other::getLaneletLength(ids[0]);
      ret.push_back(ids[0]);
      continue;
    } else {
      auto else_ids = other::getPreviousLaneletIds(lanelet_id);
      if (else_ids.size() != 0) {
        total_distance = total_distance + other::getLaneletLength(else_ids[0]);
        ret.push_back(else_ids[0]);
        continue;
      } else {
        break;
      }
    }
  }
  return ret;
}

auto getRightOfWayLaneletIds(const lanelet::Ids & lanelet_ids)
  -> std::unordered_map<lanelet::Id, lanelet::Ids>
{
  std::unordered_map<lanelet::Id, lanelet::Ids> ret;
  for (const auto & lanelet_id : lanelet_ids) {
    ret.emplace(lanelet_id, getRightOfWayLaneletIds(lanelet_id));
  }
  return ret;
}

auto getRightOfWayLaneletIds(const lanelet::Id lanelet_id) -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & right_of_way : Memory::laneletMap()
                                     ->laneletLayer.get(lanelet_id)
                                     .regulatoryElementsAs<lanelet::RightOfWay>()) {
    for (const auto & ll : right_of_way->rightOfWayLanelets()) {
      if (lanelet_id != ll.id()) {
        ids.push_back(ll.id());
      }
    }
  }
  return ids;
}

auto getConflictingCrosswalkIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids
{
  lanelet::Ids ids;
  std::vector<lanelet::routing::RoutingGraphConstPtr> graphs;
  graphs.emplace_back(Memory::vehicleRoutingGraph());
  graphs.emplace_back(Memory::pedestrianRoutingGraph());
  lanelet::routing::RoutingGraphContainer container(graphs);
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = Memory::laneletMap()->laneletLayer.get(lanelet_id);
    double height_clearance = 4;
    size_t routing_graph_id = 1;
    const auto conflicting_crosswalks =
      container.conflictingInGraph(lanelet, routing_graph_id, height_clearance);
    for (const auto & crosswalk : conflicting_crosswalks) {
      ids.emplace_back(crosswalk.id());
    }
  }
  return ids;
}

auto getConflictingLaneIds(const lanelet::Ids & lanelet_ids) -> lanelet::Ids
{
  lanelet::Ids ids;
  for (const auto & lanelet_id : lanelet_ids) {
    const auto lanelet = Memory::laneletMap()->laneletLayer.get(lanelet_id);
    const auto conflicting_lanelets =
      lanelet::utils::getConflictingLanelets(Memory::vehicleRoutingGraph(), lanelet);
    for (const auto & conflicting_lanelet : conflicting_lanelets) {
      ids.emplace_back(conflicting_lanelet.id());
    }
  }
  return ids;
}
}  // namespace route
}  // namespace lanelet2
}  // namespace traffic_simulator
