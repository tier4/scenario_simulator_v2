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
using WaypointIterator = std::vector<traffic_simulator_msgs::msg::Vertex>::const_iterator;
using EntityStatus = traffic_simulator_msgs::msg::EntityStatus;
using BehaviorParameter = traffic_simulator_msgs::msg::BehaviorParameter;
using Point = geometry_msgs::msg::Point;
using Pose = geometry_msgs::msg::Pose;
using Vector3 = geometry_msgs::msg::Vector3;
using Quaternion = geometry_msgs::msg::Quaternion;
using BoundingBox = traffic_simulator_msgs::msg::BoundingBox;

struct ValidatedEntityStatus
{
public:
  ValidatedEntityStatus(
    const EntityStatus & entity_status, const BehaviorParameter & behavior_parameter,
    const double step_time);

  auto updatedEntityStatus(const Vector3 & desired_velocity, const double step_time) const
    -> EntityStatus;

  const std::string & name;
  const double time;
  const Pose & pose;
  const double linear_speed;
  const double linear_acceleration;
  const Vector3 velocity;
  const bool lanelet_pose_valid;
  const BoundingBox & bounding_box;
  const BehaviorParameter behavior_parameter;

private:
  auto validatedPose() const noexcept(false) -> const Pose &;

  auto validatedLinearSpeed() const noexcept(false) -> double;

  auto validatedLinearAcceleration(const double step_time) const noexcept(false) -> double;

  auto validatedVelocity() const -> Vector3;

  const EntityStatus entity_status;
};

class PolylineTrajectoryFollower
{
public:
  static auto updatedEntityStatus(
    const ValidatedEntityStatus & validated_status, PolylineTrajectory & polyline_trajectory,
    const std::optional<double> target_speed, const double step_time,
    const double matching_distance,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) noexcept(false)
    -> std::optional<EntityStatus>;

private:
  static auto discardTheFrontWaypoint(
    PolylineTrajectory & polyline_trajectory, const double current_time) noexcept(false) -> void;
};

class PolylineTrajectoryFollowerStep
{
public:
  explicit PolylineTrajectoryFollowerStep(
    const ValidatedEntityStatus & validated_status, const PolylineTrajectory & polyline_trajectory,
    const std::optional<double> & target_speed, const double step_time,
    const double matching_distance,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr)
  : validated_status(validated_status),
    polyline_trajectory(polyline_trajectory),
    nearest_waypoint_position(validatedNearestWaypointPosition()),
    nearest_waypoint_with_specified_time_it(nearestWaypointWithSpecifiedTimeIterator()),
    step_time(step_time),
    distance_to_nearest_waypoint(distanceAlongLanelet(
      validated_status.pose.position, nearest_waypoint_position, matching_distance,
      hdmap_utils_ptr)),
    total_remining_distance(totalRemainingDistance(matching_distance, hdmap_utils_ptr)),
    time_to_nearest_waypoint(
      (std::isnan(polyline_trajectory.base_time) ? 0.0 : polyline_trajectory.base_time) +
      polyline_trajectory.shape.vertices.front().time - validated_status.time),
    total_remaining_time(totalRemainingTime()),
    /// @todo (may be considered) add passing constrains from ValidatedEntityStatus instead of BehaviorParameters
    // after adding constraints to ValidatedEntityStatus, it can be renamed to something like ValidatedEntityModel
    follow_waypoint_controller(FollowWaypointController(
      validated_status.behavior_parameter, step_time,
      isNearestWaypointWithSpecifiedTimeSameAsLastWaypoint(),
      std::isinf(total_remaining_time) ? target_speed : std::nullopt))
  {
  }

  auto updatedEntityStatus() const -> std::optional<EntityStatus>;

private:
  // for calculations in the constructor
  auto nearestWaypointWithSpecifiedTimeIterator() const -> WaypointIterator;

  auto isNearestWaypointWithSpecifiedTimeSameAsLastWaypoint() const -> bool;

  auto validatedNearestWaypointPosition() const noexcept(false) -> const Point &;

  auto distanceAlongLanelet(
    const Point & from, const Point & to, const double matching_distance,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) const -> double;

  auto totalRemainingDistance(
    const double matching_distance,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) const -> double;

  auto totalRemainingTime() const -> double;

  // for validation in updatedEntityStatus()
  auto validatedDesiredAcceleration() const noexcept(false) -> double;

  auto validatedDesiredSpeed(const double desired_acceleration) const noexcept(false) -> double;

  auto validatedDesiredVelocity(const double desired_speed) const noexcept(false) -> Vector3;

  auto validatePredictedState(const double desired_acceleration) const noexcept(false) -> void;

  const ValidatedEntityStatus validated_status;
  const PolylineTrajectory polyline_trajectory;
  const Point nearest_waypoint_position;
  const WaypointIterator nearest_waypoint_with_specified_time_it;

  const double step_time;
  const double distance_to_nearest_waypoint;
  const double total_remining_distance;
  const double time_to_nearest_waypoint;
  const double total_remaining_time;

  const FollowWaypointController follow_waypoint_controller;
};
}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER_HPP_
