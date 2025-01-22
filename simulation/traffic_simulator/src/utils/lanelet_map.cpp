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

#include <traffic_simulator/utils/lanelet_map.hpp>

namespace traffic_simulator
{
inline namespace lanelet_map
{
auto laneletLength(const lanelet::Id lanelet_id) -> double
{
  return lanelet_wrapper::lanelet_map::laneletLength(lanelet_id);
}
auto laneletAltitude(
  const lanelet::Id & lanelet_id, const geometry_msgs::msg::Pose & pose,
  const double matching_distance) -> std::optional<double>
{
  return lanelet_wrapper::lanelet_map::laneletAltitude(lanelet_id, pose, matching_distance);
}

auto nearbyLaneletIds(
  const Pose & pose, const double distance_thresh, const bool include_crosswalk,
  const std::size_t search_count) -> lanelet::Ids
{
  return lanelet_wrapper::lanelet_map::nearbyLaneletIds(
    pose.position, distance_thresh, include_crosswalk, search_count);
}

auto noNextLaneletPoses() -> std::vector<std::pair<lanelet::Id, Pose>>
{
  std::vector<std::pair<lanelet::Id, Pose>> no_next_lanelet_poses;
  for (const auto & lanelet_id : lanelet_wrapper::lanelet_map::laneletIds()) {
    if (lanelet_wrapper::lanelet_map::nextLaneletIds(lanelet_id).empty()) {
      LaneletPose lanelet_pose;
      lanelet_pose.lanelet_id = lanelet_id;
      lanelet_pose.s = lanelet_map::laneletLength(lanelet_id);
      no_next_lanelet_poses.emplace_back(lanelet_id, pose::toMapPose(lanelet_pose));
    }
  }
  return no_next_lanelet_poses;
}
}  // namespace lanelet_map
}  // namespace traffic_simulator
