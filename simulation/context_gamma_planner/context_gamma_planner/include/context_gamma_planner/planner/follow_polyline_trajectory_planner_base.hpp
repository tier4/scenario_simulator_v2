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

#ifndef CONTEXT_GAMMA_PLANNER__FOLLOW_POLYLINE_TRAJECTORY_PLANNER_BASE_HPP_
#define CONTEXT_GAMMA_PLANNER__FOLLOW_POLYLINE_TRAJECTORY_PLANNER_BASE_HPP_

#include <context_gamma_planner/planner/goal_planner_base.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>

namespace context_gamma_planner
{
class FollowPolylineTrajectoryPlannerBase : public GoalPlannerBase
{
public:
  FollowPolylineTrajectoryPlannerBase(const double goal_threshold);
  /**
   * @brief Sets the waypoints for the planner.
   * @param trajectory The polyline trajectory containing the waypoints.
   */
  void setWaypoints(
    const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> trajectory);
  /**
   * @brief the next goal point for the vehicle.
   * @return An optional containing the next goal point as a geometry_msgs::msg::Point, or an empty optional if the calculation fails.
   */
  auto calculateNextGoalPoint() -> std::optional<geometry_msgs::msg::Point> override;
  /**
   * @brief Get the target speed at a given time.
   * @param current_time The current time.
   * @return [m/s] The target speed at the given time.
   */
  double getTargetSpeed(const double current_time) const;
  /**
   * @brief Clears the goal planner.
   */
  void clear() override;

private:
  size_t getCurrentTrajectoryWaypointIndex() const;
  std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> trajectory_;
};
}  // namespace context_gamma_planner

#endif  // CONTEXT_GAMMA_PLANNE__FOLLOW_POLYLINE_TRAJECTORY_PLANNER_HPP_
