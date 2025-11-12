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

#include "context_gamma_planner/planner/pedestrian/follow_lane_planner.hpp"

namespace context_gamma_planner::pedestrian
{

FollowLanePlanner::FollowLanePlanner(const double goal_threshold) : GoalPlannerBase(goal_threshold)
{
}

void FollowLanePlanner::setWaypoints(
  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils,
  const std::vector<lanelet::Id> & route_ids)
{
  goal_poses_.clear();
  for (const auto & point : hdmap_utils->getCenterPoints(route_ids)) {
    geometry_msgs::msg::Pose pose;
    pose.position = point;
    goal_poses_.emplace_back(pose);
  }
}

// Sync to a nearby waypoint, drop reached points,
// and return the next unreached target (or nullopt).
auto FollowLanePlanner::calculateNextGoalPoint() -> std::optional<geometry_msgs::msg::Point>
{
  if (!status_) {
    THROW_SIMULATION_ERROR(
      "Current entity status should be set before you want to calculate goal pose.");
  }
  if (goal_poses_.empty()) {
    THROW_SIMULATION_ERROR("Goal poses should be set before you want to calculate goal pose.");
  }
  while (!goal_poses_.empty()) {
    // 1) Skip leading points that are farther than goal_threshold
    //    until we arrive at a neighborhood (distance < threshold).
    if (
      std::hypot(
        (status_->getMapPose().position.x - goal_poses_.front().position.x),
        (status_->getMapPose().position.y - goal_poses_.front().position.y)) >= goal_threshold) {
      goal_poses_.pop_front();
    } else {
      break;
    }
  }
  // If everything was discarded without finding a nearby point, stop safely.
  if (goal_poses_.empty()) {
    return std::nullopt;
  }
  while (goal_poses_.size() > 1) {
    // 2) Treat points within goal_threshold as reached and drop them.
    //    The first point at or beyond the threshold becomes the next target.
    if (
      std::hypot(
        (status_->getMapPose().position.x - goal_poses_.front().position.x),
        (status_->getMapPose().position.y - goal_poses_.front().position.y)) < goal_threshold) {
      goal_poses_.pop_front();
    } else {
      return goal_poses_.front().position;
    }
  }
  return goal_poses_.front().position;
}

void FollowLanePlanner::clear() { goal_poses_.clear(); }

}  // namespace context_gamma_planner::pedestrian
