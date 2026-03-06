
// Copyright 2015 Tier IV, Inc. All rights reserved.
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

#include <traffic_simulator/lanelet_wrapper/lanelet_map.hpp>
#include <traffic_simulator/lanelet_wrapper/lanelet_wrapper.hpp>
#include <traffic_simulator/lanelet_wrapper/route.hpp>

namespace traffic_simulator
{
namespace lanelet_wrapper
{
namespace route
{
auto isInRoute(const lanelet::Id lanelet_id, const lanelet::Ids & route_lanelets_ids) -> bool
{
  return std::find(route_lanelets_ids.begin(), route_lanelets_ids.end(), lanelet_id) !=
         route_lanelets_ids.end();
}

// it returns the speed limit in meters per second
auto speedLimit(const lanelet::Ids & lanelet_ids, const RoutingGraphType type) -> double
{
  if (lanelet_ids.empty()) {
    THROW_SEMANTIC_ERROR("size of the vector lanelet ids should be more than 1");
  } else {
    std::vector<double> limits;
    limits.reserve(lanelet_ids.size());
    for (const auto & lanelet_id : lanelet_ids) {
      const auto & lanelet = LaneletWrapper::map()->laneletLayer.get(lanelet_id);
      const auto & limit = LaneletWrapper::trafficRules(type)->speedLimit(lanelet);
      limits.push_back(lanelet::units::KmHQuantity(limit.speedLimit).value() / 3.6);
    }
    return *std::min_element(limits.begin(), limits.end());
  }
}

auto routeFromGraph(
  const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id,
  const RoutingConfiguration & routing_configuration) -> lanelet::Ids
{
  /// @todo improve architecture in terms of access to cache of the graph
  return LaneletWrapper::routeCache(routing_configuration.routing_graph_type)
    .getRoute(
      from_lanelet_id, to_lanelet_id, LaneletWrapper::map(), routing_configuration,
      LaneletWrapper::routingGraph(routing_configuration.routing_graph_type));
}

auto followingLanelets(
  const lanelet::Id current_lanelet_id, const lanelet::Ids & route, const double horizon,
  const bool include_current_lanelet_id, const RoutingGraphType type) -> lanelet::Ids
{
  const auto is_following_lanelet =
    [&type](const auto & current_lanelet, const auto & candidate_lanelet) {
      const auto next_ids = lanelet_map::nextLaneletIds(current_lanelet, type);
      return std::find(next_ids.cbegin(), next_ids.cend(), candidate_lanelet) != next_ids.cend();
    };

  lanelet::Ids following_lanelets_ids;

  if (route.empty()) {
    return following_lanelets_ids;
  }

  double total_distance = 0.0;
  bool found_starting_lanelet = false;

  for (const auto & candidate_lanelet_id : route) {
    if (found_starting_lanelet) {
      if (const auto previous_lanelet =
            following_lanelets_ids.empty() ? current_lanelet_id : following_lanelets_ids.back();
          not is_following_lanelet(previous_lanelet, candidate_lanelet_id)) {
        THROW_SEMANTIC_ERROR(
          candidate_lanelet_id + " is not the follower of lanelet " + previous_lanelet);
      }
      following_lanelets_ids.push_back(candidate_lanelet_id);
      total_distance += lanelet_map::laneletLength(candidate_lanelet_id);
      if (total_distance > horizon) {
        break;
      }
    } else if (candidate_lanelet_id == current_lanelet_id) {
      found_starting_lanelet = true;
      if (include_current_lanelet_id) {
        following_lanelets_ids.push_back(candidate_lanelet_id);
      }
    }
  }
  if (following_lanelets_ids.empty()) {
    THROW_SEMANTIC_ERROR("lanelet id does not match");
  } else if (total_distance <= horizon) {
    const auto remaining_lanelets =
      followingLanelets(route.back(), horizon - total_distance, false, type);
    following_lanelets_ids.insert(
      following_lanelets_ids.end(), remaining_lanelets.begin(), remaining_lanelets.end());
  }

  return following_lanelets_ids;
}

auto followingLanelets(
  const lanelet::Id lanelet_id, const double distance, const bool include_self,
  const RoutingGraphType type) -> lanelet::Ids
{
  lanelet::Ids following_lanelets_ids;
  if (include_self) {
    following_lanelets_ids.push_back(lanelet_id);
  }
  double total_distance = 0.0;
  auto reference_lanelet_id = lanelet_id;
  while (total_distance < distance) {
    if (const auto straight_lanelet_ids =
          lanelet_map::nextLaneletIds(reference_lanelet_id, "straight", type);
        !straight_lanelet_ids.empty()) {
      total_distance = total_distance + lanelet_map::laneletLength(straight_lanelet_ids[0]);
      following_lanelets_ids.push_back(straight_lanelet_ids[0]);
    } else if (const auto non_straight_lanelet_ids =
                 lanelet_map::nextLaneletIds(reference_lanelet_id, type);
               !non_straight_lanelet_ids.empty()) {
      total_distance = total_distance + lanelet_map::laneletLength(non_straight_lanelet_ids[0]);
      following_lanelets_ids.push_back(non_straight_lanelet_ids[0]);
    } else {
      break;
    }
    reference_lanelet_id = following_lanelets_ids.back();
  }
  return following_lanelets_ids;
}

auto previousLanelets(
  const lanelet::Id current_lanelet_id, const double backward_horizon, const RoutingGraphType type)
  -> lanelet::Ids
{
  lanelet::Ids previous_lanelets_ids;
  double total_distance = 0.0;
  previous_lanelets_ids.push_back(current_lanelet_id);
  while (total_distance < backward_horizon) {
    const auto & reference_lanelet_id = previous_lanelets_ids.back();
    if (const auto straight_lanelet_ids =
          lanelet_map::previousLaneletIds(reference_lanelet_id, "straight", type);
        not straight_lanelet_ids.empty()) {
      total_distance = total_distance + lanelet_map::laneletLength(straight_lanelet_ids[0]);
      previous_lanelets_ids.push_back(straight_lanelet_ids[0]);
    } else if (auto non_straight_lanelet_ids =
                 lanelet_map::previousLaneletIds(reference_lanelet_id, type);
               not non_straight_lanelet_ids.empty()) {
      total_distance = total_distance + lanelet_map::laneletLength(non_straight_lanelet_ids[0]);
      previous_lanelets_ids.push_back(non_straight_lanelet_ids[0]);
    } else {
      break;
    }
  }
  return previous_lanelets_ids;
}
}  // namespace route
}  // namespace lanelet_wrapper
}  // namespace traffic_simulator
