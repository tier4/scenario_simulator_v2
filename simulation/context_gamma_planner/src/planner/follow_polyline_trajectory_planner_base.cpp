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

#include "context_gamma_planner/planner/follow_polyline_trajectory_planner_base.hpp"

#include <geometry/spline/catmull_rom_spline.hpp>

namespace context_gamma_planner
{

FollowPolylineTrajectoryPlannerBase::FollowPolylineTrajectoryPlannerBase(
  const double goal_threshold)
: GoalPlannerBase(goal_threshold)
{
}

void FollowPolylineTrajectoryPlannerBase::setWaypoints(
  const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> trajectory)
{
  trajectory_ = trajectory;
  goal_poses_.clear();
  if (trajectory->dynamic_constraints_ignorable) {
    for (const auto & vertex : trajectory->shape.vertices) {
      goal_poses_.emplace_back(vertex.position);
    }
  } else {
    std::vector<geometry_msgs::msg::Point> vertices_points;
    for (const auto & vertex : trajectory->shape.vertices) {
      vertices_points.emplace_back(vertex.position.position);
    }
    const math::geometry::CatmullRomSpline spline(vertices_points);
    for (const auto & trajectory_point : spline.getTrajectory(0.0, spline.getLength(), 1.0)) {
      geometry_msgs::msg::Pose pose;
      pose.position = trajectory_point;
      goal_poses_.emplace_back(pose);
    }
  }
}

size_t FollowPolylineTrajectoryPlannerBase::getCurrentTrajectoryWaypointIndex() const
{
  size_t current_trajectory_waypoint_index = 0;
  for (const auto & vertex : trajectory_->shape.vertices) {
    if (math::geometry::getRelativePose(status_->getMapPose(), vertex.position).position.x >= 0) {
      return current_trajectory_waypoint_index;
    }
    ++current_trajectory_waypoint_index;
  }
  return trajectory_->shape.vertices.size() - 1;
}

auto FollowPolylineTrajectoryPlannerBase::calculateNextGoalPoint()
  -> std::optional<geometry_msgs::msg::Point>
{
  if (!status_) {
    THROW_SIMULATION_ERROR(
      "Current entity status should be set before you want to calculate goal pose.");
  }
  if (goal_poses_.empty()) {
    THROW_SIMULATION_ERROR("goal poses is empty");
  }
  if (!trajectory_) {
    THROW_SIMULATION_ERROR(
      "Trajectory does not specified. ",
      "This message is not originally intended to be displayed, if you see it, "
      "please contact the developer of traffic_simulator.");
  }
  while (!goal_poses_.empty()) {
    if (
      math::geometry::getRelativePose(status_->getMapPose(), goal_poses_.front()).position.x >= 0 &&
      std::hypot(
        (status_->getMapPose().position.x - goal_poses_.front().position.x),
        (status_->getMapPose().position.y - goal_poses_.front().position.y)) <= goal_threshold) {
      if (trajectory_->closed) {
        goal_poses_.emplace_back(goal_poses_.front());
      }
      goal_poses_.pop_front();
    } else {
      return goal_poses_.front().position;
    }
  }
  return std::nullopt;
}

double FollowPolylineTrajectoryPlannerBase::getTargetSpeed(const double current_time) const
{
  if (trajectory_->base_time > current_time) {
    return 0.0;
  }
  if (goal_poses_.empty()) {
    THROW_SIMULATION_ERROR("goal poses is empty");
  }
  double goal_time = trajectory_->base_time +
                     trajectory_->shape.vertices.at(getCurrentTrajectoryWaypointIndex()).time;
  goal_time = std::fmod(goal_time, trajectory_->shape.vertices.back().time);
  if (goal_time <= current_time) {
    return max_speed_;
  }
  return std::hypot(
           (status_->getMapPose().position.x - goal_poses_.front().position.x),
           (status_->getMapPose().position.y - goal_poses_.front().position.y)) /
         (goal_time - current_time);
}

void FollowPolylineTrajectoryPlannerBase::clear() { goal_poses_.clear(); }

}  // namespace context_gamma_planner
