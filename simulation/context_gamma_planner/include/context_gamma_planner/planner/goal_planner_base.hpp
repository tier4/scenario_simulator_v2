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

#ifndef CONTEXT_GAMMA_PLANNER__GOAL_PLANNER_BASE_HPP_
#define CONTEXT_GAMMA_PLANNER__GOAL_PLANNER_BASE_HPP_

#include <algorithm>
#include <deque>
#include <geometry_msgs/msg/pose.hpp>
#include <optional>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>

namespace context_gamma_planner
{
class GoalPlannerBase
{
public:
  explicit GoalPlannerBase(const double goal_threshold);
  virtual ~GoalPlannerBase() = default;

  void setCurrentStatus(
    const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> & status);

  void appendGoalPoses(const std::vector<geometry_msgs::msg::Pose> & goal_poses);

  void appendGoalPoints(const std::vector<geometry_msgs::msg::Point> & goal_points);

  void setMaxSpeed(const double max_speed);

  double getMaxSpeed() const { return max_speed_; }

  std::vector<geometry_msgs::msg::Pose> getGoalPoses() const;

  traffic_simulator_msgs::msg::WaypointsArray getWaypoints() const;

  geometry_msgs::msg::Pose getNextGoalPose() const { return goal_poses_.front(); }

  bool isReachedGoal() const { return goal_poses_.empty(); }

  virtual auto calculateNextGoalPoint() -> std::optional<geometry_msgs::msg::Point> = 0;

  virtual void clear() = 0;
  const double goal_threshold;

protected:
  std::optional<traffic_simulator::CanonicalizedEntityStatus> status_;
  std::deque<geometry_msgs::msg::Pose> goal_poses_;
  double max_speed_ = 0.0;
};
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__GOAL_PLANNER_BASE_HPP_
