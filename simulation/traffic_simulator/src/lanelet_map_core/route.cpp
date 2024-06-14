
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
auto isInRoute(const lanelet::Id lanelet_id, const lanelet::Ids & route_lanelets_ids) -> bool
{
  return std::find(route_lanelets_ids.begin(), route_lanelets_ids.end(), lanelet_id) !=
         route_lanelets_ids.end();
}

// it returns the speed limit in meters per second
auto speedLimit(const lanelet::Ids & lanelet_ids) -> double
{
  if (lanelet_ids.empty()) {
    THROW_SEMANTIC_ERROR("size of the vector lanelet ids should be more than 1");
  } else {
    std::vector<double> limits;
    limits.reserve(lanelet_ids.size());
    for (const auto & lanelet_id : lanelet_ids) {
      const auto & lanelet = LaneletMapCore::map()->laneletLayer.get(lanelet_id);
      const auto & limit = LaneletMapCore::trafficRulesVehicle()->speedLimit(lanelet);
      limits.push_back(lanelet::units::KmHQuantity(limit.speedLimit).value() / 3.6);
    }
    return *std::min_element(limits.begin(), limits.end());
  }
}

auto route(
  const lanelet::Id from_lanelet_id, const lanelet::Id to_lanelet_id, const bool allow_lane_change)
  -> lanelet::Ids
{
  return LaneletMapCore::routeCache().getRoute(
    from_lanelet_id, to_lanelet_id, allow_lane_change, LaneletMapCore::map(),
    LaneletMapCore::vehicleRoutingGraph());
}

auto followingLanelets(
  const lanelet::Id lanelet_id, const lanelet::Ids & candidate_lanelet_ids, const double distance,
  const bool include_self) -> lanelet::Ids
{
  if (candidate_lanelet_ids.empty()) {
    return {};
  } else {
    lanelet::Ids following_lanelets_ids;
    double total_distance = 0.0;
    bool found_reference_lanelet_id = false;
    for (const auto & candidate_lanelet_id : candidate_lanelet_ids) {
      if (found_reference_lanelet_id) {
        following_lanelets_ids.push_back(candidate_lanelet_id);
        total_distance += lanelet_map::laneletLength(candidate_lanelet_id);
        if (total_distance > distance) {
          return following_lanelets_ids;
        }
      }
      if (!found_reference_lanelet_id && candidate_lanelet_id == lanelet_id) {
        found_reference_lanelet_id = true;
        if (include_self) {
          following_lanelets_ids.push_back(candidate_lanelet_id);
        }
      }
    }

    if (!found_reference_lanelet_id) {
      THROW_SEMANTIC_ERROR("lanelet id does not match");
    } else if (total_distance > distance) {
      return following_lanelets_ids;
    } else {
      const auto remaining_lanelets =
        followingLanelets(candidate_lanelet_ids.back(), distance - total_distance, false);
      following_lanelets_ids.insert(
        following_lanelets_ids.end(), remaining_lanelets.begin(), remaining_lanelets.end());
      return following_lanelets_ids;
    }
  }
}

auto followingLanelets(const lanelet::Id lanelet_id, const double distance, const bool include_self)
  -> lanelet::Ids
{
  lanelet::Ids following_lanelets_ids;
  if (include_self) {
    following_lanelets_ids.push_back(lanelet_id);
  }
  double total_distance = 0.0;
  auto reference_lanelet_id = lanelet_id;
  while (total_distance < distance) {
    if (const auto straight_lanelet_ids =
          lanelet_map::nextLaneletIds(reference_lanelet_id, "straight");
        !straight_lanelet_ids.empty()) {
      total_distance = total_distance + lanelet_map::laneletLength(straight_lanelet_ids[0]);
      following_lanelets_ids.push_back(straight_lanelet_ids[0]);
    } else if (const auto non_straight_lanelet_ids =
                 lanelet_map::nextLaneletIds(reference_lanelet_id);
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

auto previousLanelets(const lanelet::Id lanelet_id, const double distance) -> lanelet::Ids
{
  /// @note it has been modified because there was probably a bug
  lanelet::Ids previous_lanelets_ids;
  double total_distance = 0.0;
  previous_lanelets_ids.push_back(lanelet_id);
  while (total_distance < distance) {
    const auto & reference_lanelet_id = previous_lanelets_ids.back();
    if (const auto straight_lanelet_ids =
          lanelet_map::previousLaneletIds(reference_lanelet_id, "straight");
        !straight_lanelet_ids.empty()) {
      total_distance = total_distance + lanelet_map::laneletLength(straight_lanelet_ids[0]);
      previous_lanelets_ids.push_back(straight_lanelet_ids[0]);
    } else if (auto non_straight_lanelet_ids =
                 lanelet_map::previousLaneletIds(reference_lanelet_id);
               !non_straight_lanelet_ids.empty()) {
      total_distance = total_distance + lanelet_map::laneletLength(non_straight_lanelet_ids[0]);
      previous_lanelets_ids.push_back(non_straight_lanelet_ids[0]);
    } else {
      break;
    }
  }
  return previous_lanelets_ids;
}
}  // namespace route
}  // namespace lanelet_map_core
}  // namespace traffic_simulator
