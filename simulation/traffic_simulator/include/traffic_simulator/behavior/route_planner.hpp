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

#ifndef BEHAVIOR_TREE_PLUGIN__ROUTE_PLANNER_HPP_
#define BEHAVIOR_TREE_PLUGIN__ROUTE_PLANNER_HPP_

#include <deque>
#include <memory>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <vector>

namespace traffic_simulator
{
/// @brief This class cache waypoints and planning waypoints.
/// @todo Currently, in order to cache waypoints, cancelRoute() member function has const. This class should be refactored.
class RoutePlanner
{
public:
  explicit RoutePlanner(const std::shared_ptr<hdmap_utils::HdMapUtils> &);

  auto getRouteLanelets(const CanonicalizedLaneletPose & entity_lanelet_pose, double horizon = 100)
    const -> std::vector<std::int64_t>;

  auto setWaypoints(const std::vector<CanonicalizedLaneletPose> & waypoints) -> void;

  auto cancelRoute() const -> void;
  auto getGoalPoses() const -> std::vector<CanonicalizedLaneletPose>;
  auto getGoalPosesInWorldFrame() const -> std::vector<geometry_msgs::msg::Pose>;

private:
  auto cancelWaypoint(const CanonicalizedLaneletPose & entity_lanelet_pose) const -> void;

  auto updateRoute(const CanonicalizedLaneletPose & entity_lanelet_pose) const -> void;

  mutable std::optional<std::vector<std::int64_t>> route_;
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;

  /*
     What we need is a queue, but we need to be able to iterate over the
     elements for getGoalPoses, so we use std::deque instead of std::queue
     which is not iterable.
  */
  mutable std::deque<traffic_simulator::CanonicalizedLaneletPose> waypoint_queue_;
};
}  // namespace traffic_simulator

#endif  // BEHAVIOR_TREE_PLUGIN__ROUTE_PLANNER_HPP_
