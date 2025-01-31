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
#include <context_gamma_planner/planner/vehicle/lane_change_planner.hpp>
#include <geometry/transform.hpp>
#include <iostream>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/lanelet_wrapper/pose.hpp>

namespace context_gamma_planner
{
namespace vehicle
{
LaneChangePlanner::LaneChangePlanner(const double goal_threshold) : GoalPlannerBase(goal_threshold)
{
}

void LaneChangePlanner::setWaypoints(
  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils,
  const std::vector<lanelet::Id> & route_ids,
  const traffic_simulator::lane_change::Parameter & lane_change_parameter)
{
  clear();
  // calc trajectory
  const auto lanelet_pose =
    traffic_simulator::lanelet_wrapper::pose::toLaneletPose(status_->getMapPose(), route_ids);
  if (!lanelet_pose) {
    THROW_SIMULATION_ERROR("Failed to get lanelet pose");
  }
  std::optional<std::pair<math::geometry::HermiteCurve, double>> lanechange_trajectory;
  traffic_simulator::LaneletPose along_pose, goal_pose;
  switch (lane_change_parameter.constraint.type) {
    case traffic_simulator::lane_change::Constraint::Type::NONE:
      /**
      @note Hard coded parameter, 
      10.0 is a maximum_curvature_threshold (If the curvature of the trajectory is over 10.0, the trajectory was not selected.)
      20.0 is a target_trajectory_length (The one with the closest length to 20 m is selected from the candidate trajectories.)
      1.0 is a forward_distance_threshold (If the goal x position in the cartesian coordinate was under 1.0, the goal was rejected.)
      */
      lanechange_trajectory = hdmap_utils->getLaneChangeTrajectory(
        traffic_simulator::lanelet_wrapper::pose::toMapPose(lanelet_pose.value()).pose,
        lane_change_parameter, 10.0, 20.0, 1.0);
      along_pose = traffic_simulator::lanelet_wrapper::pose::alongLaneletPose(
        lanelet_pose.value(),
        traffic_simulator::lane_change::Parameter::default_lanechange_distance);
      break;
    case traffic_simulator::lane_change::Constraint::Type::LATERAL_VELOCITY:
      lanechange_trajectory =
        hdmap_utils->getLaneChangeTrajectory(lanelet_pose.value(), lane_change_parameter);
      along_pose = traffic_simulator::lanelet_wrapper::pose::alongLaneletPose(
        lanelet_pose.value(),
        traffic_simulator::lane_change::Parameter::default_lanechange_distance);
      break;
    case traffic_simulator::lane_change::Constraint::Type::LONGITUDINAL_DISTANCE:
      lanechange_trajectory =
        hdmap_utils->getLaneChangeTrajectory(lanelet_pose.value(), lane_change_parameter);
      along_pose = traffic_simulator::lanelet_wrapper::pose::alongLaneletPose(
        lanelet_pose.value(), lane_change_parameter.constraint.value);
      break;
    case traffic_simulator::lane_change::Constraint::Type::TIME:
      lanechange_trajectory =
        hdmap_utils->getLaneChangeTrajectory(lanelet_pose.value(), lane_change_parameter);
      along_pose = traffic_simulator::lanelet_wrapper::pose::alongLaneletPose(
        lanelet_pose.value(), lane_change_parameter.constraint.value);
      break;
  }
  if (!lanechange_trajectory) {
    THROW_SIMULATION_ERROR("Failed to get lane change trajectory");
  }
  for (const auto & lanechange_point : lanechange_trajectory.value().first.getTrajectory()) {
    geometry_msgs::msg::Pose pose;
    pose.position = lanechange_point;
    goal_poses_.emplace_back(pose);
  }

  // calc velocity
  const auto curve_ = lanechange_trajectory->first;
  goal_pose.lanelet_id = lane_change_parameter.target.lanelet_id;
  goal_pose.s = lanechange_trajectory->second;
  double offset = std::fabs(math::geometry::getRelativePose(
                              traffic_simulator::lanelet_wrapper::pose::toMapPose(along_pose).pose,
                              traffic_simulator::lanelet_wrapper::pose::toMapPose(goal_pose).pose)
                              .position.y);
  switch (lane_change_parameter.constraint.type) {
    case traffic_simulator::lane_change::Constraint::Type::NONE:
      target_speed_ = max_speed_;
      break;
    case traffic_simulator::lane_change::Constraint::Type::LATERAL_VELOCITY:
      target_speed_ = curve_.getLength() / (offset / lane_change_parameter.constraint.value);
      break;
    case traffic_simulator::lane_change::Constraint::Type::LONGITUDINAL_DISTANCE:
      target_speed_ = max_speed_;
      break;
    case traffic_simulator::lane_change::Constraint::Type::TIME:
      target_speed_ = curve_.getLength() / lane_change_parameter.constraint.value;
      break;
  }
}

auto LaneChangePlanner::calculateNextGoalPoint() -> std::optional<geometry_msgs::msg::Point>
{
  if (!status_) {
    THROW_SIMULATION_ERROR(
      "Current entity status should be set before you want to calculate goal pose.");
  }
  if (goal_poses_.empty()) {
    THROW_SIMULATION_ERROR("goal poses is empty");
  }

  while (!goal_poses_.empty()) {
    if (
      std::hypot(
        (status_->getMapPose().position.x - goal_poses_.front().position.x),
        (status_->getMapPose().position.y - goal_poses_.front().position.y)) <= goal_threshold) {
      goal_poses_.pop_front();
    } else {
      return goal_poses_.front().position;
    }
  }
  return std::nullopt;
}

void LaneChangePlanner::clear() { goal_poses_.clear(); }

}  // namespace vehicle
}  // namespace context_gamma_planner
