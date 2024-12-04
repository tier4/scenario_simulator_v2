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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER_STEP_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER_STEP_HPP_

#include <optional>
#include <traffic_simulator/behavior/follow_waypoint_controller.hpp>
#include <traffic_simulator/behavior/validated_entity_status.hpp>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{

struct PolylineTrajectoryFollowerStep
{
public:
  explicit PolylineTrajectoryFollowerStep(
    const ValidatedEntityStatus & validated_entity_status,
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr,
    const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter,
    const double step_time);

  auto makeUpdatedEntityStatus(
    const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
    const double matching_distance, const std::optional<double> target_speed) const
    -> std::optional<EntityStatus>;

private:
  const ValidatedEntityStatus validated_entity_status;
  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr;
  const traffic_simulator_msgs::msg::BehaviorParameter behavior_parameter;
  const double step_time;

  auto calculateCurrentVelocity(const double speed) const -> geometry_msgs::msg::Vector3;
  auto calculateDistanceAndRemainingTime(
    const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
    const double matching_distance, const double distance_to_front_waypoint,
    const double step_time) const -> std::tuple<double, double>;
  auto validatedEntityTargetPosition(
    const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory) const
    noexcept(false) -> geometry_msgs::msg::Point;
  auto validatedEntityDesiredAcceleration(
    const traffic_simulator::follow_trajectory::FollowWaypointController &
      follow_waypoint_controller,
    const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
    const double remaining_time, const double distance, const double acceleration,
    const double speed) const noexcept(false) -> double;
  auto validatedEntityDesiredVelocity(
    const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
    const geometry_msgs::msg::Point & target_position, const geometry_msgs::msg::Point & position,
    const double desired_speed) const noexcept(false) -> geometry_msgs::msg::Vector3;
  auto validatedEntityDesiredSpeed(
    const double entity_speed, const double desired_acceleration) const noexcept(false) -> double;
};
}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER_HPP_
