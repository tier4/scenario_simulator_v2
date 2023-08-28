// Copyright 2023 TIER IV, Inc. All rights reserved.
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
#include <traffic_simulator/behavior/follow_trajectory/vehicle_model.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{

class PolylineTrajectoryFollower
{
public:
  PolylineTrajectoryFollower() = default;
  ~PolylineTrajectoryFollower() = default;

  virtual auto setParameters(
    const traffic_simulator_msgs::msg::EntityStatus &,
    const traffic_simulator_msgs::msg::BehaviorParameter &, const double,
    const traffic_simulator_msgs::msg::VehicleParameters & vehicle_parameters) -> void = 0;
  virtual auto setParameters(
    const traffic_simulator_msgs::msg::EntityStatus &,
    const traffic_simulator_msgs::msg::BehaviorParameter &, const double) -> void = 0;
  virtual std::optional<traffic_simulator_msgs::msg::EntityStatus> followTrajectory(
    std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & polyline_trajectory) = 0;

protected:
  auto getTargetPosition() const -> geometry_msgs::msg::Point;
  auto getAccelerationLimits(const double acceleration) const -> std::tuple<double, double>;
  auto getTimeRemainingToFrontWaypoint(
    double remaining_time_to_front_waypoint, double distance_to_front_waypoint,
    double desired_speed) const -> std::optional<double>;
  auto getDesiredSpeed(
    double desired_acceleration, double min_acceleration, double max_acceleration,
    double speed) const -> double;
  auto getDesiredAcceleration(
    double remaining_time, double acceleration, double distance, double speed) const -> double;
  auto getTargetPositionAndDesiredSpeed() const
    -> std::optional<std::tuple<geometry_msgs::msg::Point, double>>;

  virtual void discardTheFrontWaypointFromTrajectory() const = 0;
  virtual auto getDistanceAndTimeToFrontWaypoint(
    const geometry_msgs::msg::Point & target_position,
    const geometry_msgs::msg::Point & position) const
    -> std::optional<std::tuple<double, double>> = 0;
  virtual auto getDistanceAndTimeToWaypointWithSpecifiedTime(
    double distance_to_front_waypoint) const -> std::optional<std::tuple<double, double>> = 0;

  double step_time_m;
  Vehicle vehicle;
  traffic_simulator_msgs::msg::BehaviorParameter behavior_parameter_m;
  mutable std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> polyline_trajectory_m;
};

}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER_HPP_
