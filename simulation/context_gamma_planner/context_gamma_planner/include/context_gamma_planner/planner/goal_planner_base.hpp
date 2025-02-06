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

#ifndef CONTEXT_GAMMA_PLANNER__GOAL_PLANNER_BASE_HPP_
#define CONTEXT_GAMMA_PLANNER__GOAL_PLANNER_BASE_HPP_

#include <deque>
#include <geometry_msgs/msg/pose.hpp>
#include <optional>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/waypoints_array.hpp>

namespace context_gamma_planner
{
class GoalPlannerBase
{
public:
  GoalPlannerBase(const double goal_threshold);
  /**
   * @brief Sets the entity status 
   * @param status entity status
   */
  void setCurrentStatus(
    const std::shared_ptr<traffic_simulator::CanonicalizedEntityStatus> & status);
  /**
   * @brief Appends a vector of goal poses to the existing goal poses.
   * @param goal_poses The vector of goal poses to be appended.
   */
  void appendGoalPoses(const std::vector<geometry_msgs::msg::Pose> & goal_poses);
  /**
   * @brief Appends a vector of goal points to the existing goal points.
   * @param goal_points The vector of goal points to be appended.
   */
  void appendGoalPoints(const std::vector<geometry_msgs::msg::Point> & goal_points);
  /**
   * @brief Sets the maximum speed.
   * @param max_speed [m/s] The maximum speed.
   */
  void setMaxSpeed(const double & max_speed);
  /**
   * @brief Get the maximum speed.
   * @return [m/s] The maximum speed.
   */
  double getMaxSpeed() const { return max_speed_; }
  /**
   * @brief Get the goal poses.
   * @return std::vector<geometry_msgs::msg::Pose> The goal poses.
   */
  std::vector<geometry_msgs::msg::Pose> getGoalPoses() const;
  /**
   * @brief Get the waypoints.
   * @return traffic_simulator_msgs::msg::WaypointsArray The waypoints.
   */
  traffic_simulator_msgs::msg::WaypointsArray getWaypoints() const;
  /**
   * @brief Get the next goal pose.
   * @return geometry_msgs::msg::Pose The next goal pose.
   */
  geometry_msgs::msg::Pose getNextGoalPose() const { return goal_poses_.front(); }
  /**
   * @brief Checks if the goal has been reached.
   * @return true if the goal has been reached, false otherwise.
   */
  bool isReachedGoal() const { return goal_poses_.empty(); }
  /**
   * @brief Calculates the next goal point from a point on the waypoint and returns a value.
   * @return The next goal point.
   */
  virtual auto calculateNextGoalPoint() -> std::optional<geometry_msgs::msg::Point> = 0;
  /**
   * @brief Clears the goal planner.
   */
  virtual void clear() = 0;
  const double goal_threshold;

protected:
  std::optional<traffic_simulator::CanonicalizedEntityStatus> status_;
  std::deque<geometry_msgs::msg::Pose> goal_poses_;
  double max_speed_ = 0.0;
};
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNER__GOAL_PLANNER_BASE_HPP_
