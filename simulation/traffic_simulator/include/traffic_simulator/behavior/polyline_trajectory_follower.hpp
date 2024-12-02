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
using PolylineTrajectory = traffic_simulator_msgs::msg::PolylineTrajectory;
using EntityStatus = traffic_simulator_msgs::msg::EntityStatus;
using BehaviorParameter = traffic_simulator_msgs::msg::BehaviorParameter;
using Point = geometry_msgs::msg::Point;
using Pose = geometry_msgs::msg::Pose;
using Vector3 = geometry_msgs::msg::Vector3;
using Quaternion = geometry_msgs::msg::Quaternion;

struct VehicleState
{
public:
  explicit VehicleState(
    const EntityStatus & entity_status, const BehaviorParameter & behavior_parameter,
    const double step_time);

  const EntityStatus entity_status;
  const BehaviorParameter behavior_parameter;

  const std::string name;
  const double time;
  const Pose pose;
  const double linear_speed;
  const double linear_acceleration;
  const Vector3 velocity;
  const bool lanelet_pose_valid;
  auto validatedPose() const noexcept(false) -> Pose;
  auto validatedLinearSpeed() const noexcept(false) -> double;
  auto validatedLinearAcceleration(const double step_time) const noexcept(false) -> double;
  auto validatedVelocity() -> Vector3;

  auto updatedEntityStatus(
    const geometry_msgs::msg::Vector3 & desired_velocity, const double step_time) const
    noexcept(true) -> EntityStatus;
};

struct PolylineTrajectoryFollower
{
public:
  explicit PolylineTrajectoryFollower(
    const VehicleState & vehicle_state, PolylineTrajectory & polyline_trajectory,
    const double matching_distance, const std::optional<double> target_speed,
    const double step_time, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr);

  auto updatedEntityStatus() -> std::optional<EntityStatus>;

private:
  const VehicleState vehicle_state;

  PolylineTrajectory polyline_trajectory;
  const double matching_distance;
  const std::optional<double> target_speed;

  const std::vector<traffic_simulator_msgs::msg::Vertex>::const_iterator
    first_waypoint_with_time_it;

  const double step_time;
  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr;

  auto firstWaypointWithArrivalTimeSpecified() const
    -> std::vector<traffic_simulator_msgs::msg::Vertex>::const_iterator;

  auto distanceAlongLanelet(
    const geometry_msgs::msg::Point & from, const geometry_msgs::msg::Point & to) const -> double;

  auto calculateDistanceAndRemainingTime(const double distance_to_front_waypoint) const
    -> std::tuple<double, double>;

  auto discardTheFrontWaypointAndRecurse() -> std::optional<EntityStatus>;

  auto validatedTargetPosition(const PolylineTrajectory & polyline_trajectory) const noexcept(false)
    -> Point;

  auto validatedDesiredAcceleration(
    const FollowWaypointController & follow_waypoint_controller,
    const PolylineTrajectory & polyline_trajectory, const double remaining_time,
    const double distance, const double acceleration, const double speed) const noexcept(false)
    -> double;

  auto validatedDesiredVelocity(
    const PolylineTrajectory & polyline_trajectory, const Point & target_position,
    const Point & position, const double desired_speed) const noexcept(false)
    -> geometry_msgs::msg::Vector3;

  auto validatedDesiredSpeed(const double entity_speed, const double desired_acceleration) const
    noexcept(false) -> double;
};
}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER_HPP_
