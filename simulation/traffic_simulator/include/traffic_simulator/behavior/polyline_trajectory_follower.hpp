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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER_HPP_

#include <optional>
#include <traffic_simulator/behavior/follow_waypoint_controller.hpp>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{
struct PolylineTrajectoryFollower
{
public:
  explicit PolylineTrajectoryFollower(
    const traffic_simulator_msgs::msg::EntityStatus & entity_status,
    const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils, const double step_time);

  auto makeUpdatedEntityStatus(
    const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
    const double matching_distance,
    const std::optional<double> target_speed /*= std::nullopt*/) const
    -> std::optional<EntityStatus>;

private:
  const traffic_simulator_msgs::msg::EntityStatus entity_status;
  const traffic_simulator_msgs::msg::BehaviorParameter behavior_parameter;
  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils;
  const double step_time;

  auto discardTheFrontWaypointAndRecurse(
    const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
    const double matching_distance, const std::optional<double> target_speed) const
    -> std::optional<EntityStatus>;
  auto buildUpdatedEntityStatus(const geometry_msgs::msg::Vector3 & desired_velocity) const
    noexcept(true) -> EntityStatus;
  auto getValidatedEntityAcceleration() const noexcept(false) -> double;
  auto getValidatedEntitySpeed() const noexcept(false) -> double;
  auto getValidatedEntityMaxAcceleration(const double acceleration) const noexcept(false) -> double;
  auto getValidatedEntityMinAcceleration(const double acceleration) const noexcept(false) -> double;
  auto getValidatedEntityPosition() const noexcept(false) -> geometry_msgs::msg::Point;
  auto getValidatedEntityTargetPosition(
    const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory) const
    noexcept(false) -> geometry_msgs::msg::Point;
  auto getValidatedEntityDesiredAcceleration(
    const traffic_simulator::follow_trajectory::FollowWaypointController &
      follow_waypoint_controller,
    const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
    const double remaining_time, const double distance, const double acceleration,
    const double speed) const noexcept(false) -> double;
  auto getValidatedEntityDesiredVelocity(
    const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
    const geometry_msgs::msg::Point & target_position, const geometry_msgs::msg::Point & position,
    const double desired_speed) const noexcept(false) -> geometry_msgs::msg::Vector3;
  auto getValidatedEntityDesiredSpeed(
    const double entity_speed, const double desired_acceleration) const noexcept(false) -> double;
};
}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER_HPP_
