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

  auto setParameters(
    const traffic_simulator_msgs::msg::EntityStatus &,
    const traffic_simulator_msgs::msg::BehaviorParameter &, const double,
    const std::optional<traffic_simulator_msgs::msg::VehicleParameters> & vehicle_parameters =
      std::nullopt) -> void;

  std::optional<traffic_simulator_msgs::msg::EntityStatus> followTrajectory(
    std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & polyline_trajectory);

protected:
  auto getCurrentPosition() const -> geometry_msgs::msg::Point;
  auto getTargetPosition() const -> geometry_msgs::msg::Point;
  auto getCurrentAcceleration() const -> double;
  auto getAccelerationLimits(const double acceleration) const -> std::tuple<double, double>;
  auto getCurrentSpeed() const -> double;
  auto getTimeRemainingToFrontWaypoint(
    double remaining_time_to_front_waypoint, double distance_to_front_waypoint,
    double desired_speed) const -> std::optional<double>;

  virtual void discardTheFrontWaypointFromTrajectory() = 0;
  virtual auto createUpdatedEntityStatus(const geometry_msgs::msg::Vector3 & velocity) const
    -> traffic_simulator_msgs::msg::EntityStatus = 0;
  virtual auto getUpdatedVelocity(
    const geometry_msgs::msg::Vector3 & desired_velocity, double desired_speed) const
    -> geometry_msgs::msg::Vector3 = 0;
  virtual auto getDesiredAcceleration(
    double remaining_time, double acceleration, double distance, double speed) const -> double = 0;
  virtual auto getDesiredSpeed(
    double desired_acceleration, double min_acceleration, double max_acceleration,
    double speed) const -> double = 0;
  virtual auto getDesiredVelocity(
    const geometry_msgs::msg::Point & target_position, const geometry_msgs::msg::Point & position,
    double desired_speed) -> geometry_msgs::msg::Vector3 = 0;
  virtual auto getDistanceAndTimeToFrontWaypoint(
    const geometry_msgs::msg::Point & target_position,
    const geometry_msgs::msg::Point & position) const
    -> std::optional<std::tuple<double, double>> = 0;
  virtual auto getDistanceAndTimeToWaypointWithSpecifiedTime(
    double distance_to_front_waypoint) const -> std::optional<std::tuple<double, double>> = 0;

  double step_time_m;
  traffic_simulator_msgs::msg::EntityStatus entity_status_m;
  traffic_simulator_msgs::msg::BehaviorParameter behavior_parameter_m;
  std::optional<traffic_simulator_msgs::msg::VehicleParameters> vehicle_parameters_m;
  std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> polyline_trajectory_m;
};

class PositionModePolylineTrajectoryFollower : public PolylineTrajectoryFollower
{
public:
  PositionModePolylineTrajectoryFollower() : PolylineTrajectoryFollower(){};

private:
  virtual auto getUpdatedVelocity(
    const geometry_msgs::msg::Vector3 & desired_velocity, double desired_speed) const
    -> geometry_msgs::msg::Vector3 override;
  virtual auto getDesiredAcceleration(
    double remaining_time, double acceleration, double distance, double speed) const
    -> double override;
  virtual auto getDesiredSpeed(
    double desired_acceleration, double min_acceleration, double max_acceleration,
    double speed) const -> double override;
  virtual auto getDesiredVelocity(
    const geometry_msgs::msg::Point & target_position, const geometry_msgs::msg::Point & position,
    double desired_speed) -> geometry_msgs::msg::Vector3 override;
  virtual auto getDistanceAndTimeToFrontWaypoint(
    const geometry_msgs::msg::Point & target_position,
    const geometry_msgs::msg::Point & position) const
    -> std::optional<std::tuple<double, double>> override;
  virtual auto getDistanceAndTimeToWaypointWithSpecifiedTime(
    double distance_to_front_waypoint) const -> std::optional<std::tuple<double, double>> override;
  virtual auto createUpdatedEntityStatus(const geometry_msgs::msg::Vector3 & velocity) const
    -> traffic_simulator_msgs::msg::EntityStatus override;

  virtual void discardTheFrontWaypointFromTrajectory() override;

};

class FollowModePolylineTrajectoryFollower : public PolylineTrajectoryFollower
{
public:
  FollowModePolylineTrajectoryFollower() : PolylineTrajectoryFollower(){};

private:
  virtual auto getUpdatedVelocity(
    const geometry_msgs::msg::Vector3 & desired_velocity, double desired_speed) const
    -> geometry_msgs::msg::Vector3 override;
  virtual auto getDesiredAcceleration(
    double remaining_time, double acceleration, double distance, double speed) const
    -> double override;
  virtual auto getDesiredSpeed(
    double desired_acceleration, double min_acceleration, double max_acceleration,
    double speed) const -> double override;
  virtual auto getDesiredVelocity(
    const geometry_msgs::msg::Point & target_position, const geometry_msgs::msg::Point & position,
    double desired_speed) -> geometry_msgs::msg::Vector3 override;
  virtual auto getDistanceAndTimeToFrontWaypoint(
    const geometry_msgs::msg::Point & target_position,
    const geometry_msgs::msg::Point & position) const
    -> std::optional<std::tuple<double, double>> override;
  virtual auto getDistanceAndTimeToWaypointWithSpecifiedTime(
    double distance_to_front_waypoint) const -> std::optional<std::tuple<double, double>> override;
  virtual auto createUpdatedEntityStatus(const geometry_msgs::msg::Vector3 & velocity) const
    -> traffic_simulator_msgs::msg::EntityStatus override;

  virtual void discardTheFrontWaypointFromTrajectory() override;


  std::optional<geometry_msgs::msg::Point> previous_target;
  double steering = 0.0;
};
}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER_HPP_
