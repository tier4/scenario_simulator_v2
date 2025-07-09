// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <algorithm>
#include <cmath>
#include <context_gamma_planner/planner/goal_planner_base.hpp>
#include <geometry/transform.hpp>
#include <iostream>
#include <scenario_simulator_exception/exception.hpp>

namespace context_gamma_planner
{
// void GoalPlannerBase::setRouteLanelets(const std::vector<lanelet::Id> & route) { route_ = route; }

GoalPlannerBase::GoalPlannerBase(const double goal_threshold) : goal_threshold(goal_threshold) {}

void GoalPlannerBase::setCurrentStatus(
  const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> & status)
{
  status_ = *status;
}

traffic_simulator_msgs::msg::WaypointsArray GoalPlannerBase::getWaypoints() const
{
  traffic_simulator_msgs::msg::WaypointsArray waypoints;
  waypoints.waypoints.emplace_back(status_->getMapPose().position);
  std::transform(
    goal_poses_.begin(), goal_poses_.end(), std::back_inserter(waypoints.waypoints),
    [](const auto & pose) { return pose.position; });
  return waypoints;
}

void GoalPlannerBase::appendGoalPoses(const std::vector<geometry_msgs::msg::Pose> & goal_poses)
{
  goal_poses_.insert(goal_poses_.end(), goal_poses.begin(), goal_poses.end());
}

void GoalPlannerBase::appendGoalPoints(const std::vector<geometry_msgs::msg::Point> & goal_points)
{
  for (const auto point : goal_points) {
    geometry_msgs::msg::Pose pose;
    pose.position = point;
    goal_poses_.emplace_back(pose);
  }
}

void GoalPlannerBase::setMaxSpeed(const double & max_speed) { max_speed_ = max_speed; }

std::vector<geometry_msgs::msg::Pose> GoalPlannerBase::getGoalPoses() const
{
  std::vector<geometry_msgs::msg::Pose> goal_poses;
  std::transform(
    goal_poses_.begin(), goal_poses_.end(), std::back_inserter(goal_poses),
    [](const auto & pose) { return pose; });
  return goal_poses;
}

}  // namespace context_gamma_planner
