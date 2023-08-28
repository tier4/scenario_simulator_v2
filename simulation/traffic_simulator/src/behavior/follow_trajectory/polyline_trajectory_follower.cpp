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

#include <arithmetic/floating_point/comparison.hpp>
#include <geometry/vector3/operator.hpp>
#include <iostream>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/behavior/follow_trajectory/polyline_trajectory_follower.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{

auto PolylineTrajectoryFollower::getTargetPositionAndDesiredSpeed() const
  -> std::optional<std::tuple<geometry_msgs::msg::Point, double>>
{
  const auto position = vehicle.getCurrentPosition();
  const auto target_position = getTargetPosition();
  const auto front_waypoint_data = getDistanceAndTimeToFrontWaypoint(target_position, position);
  if (!front_waypoint_data) {
    discardTheFrontWaypointFromTrajectory();
    return std::nullopt;
  }
  const auto [distance_to_front_waypoint, remaining_time_to_front_waypoint] =
    front_waypoint_data.value();

  const auto timed_waypoint_data =
    getDistanceAndTimeToWaypointWithSpecifiedTime(distance_to_front_waypoint);
  if (!timed_waypoint_data) {
    discardTheFrontWaypointFromTrajectory();
    return std::nullopt;
  }
  const auto [distance_to_timed_waypoint, remaining_time_to_timed_waypoint] =
    timed_waypoint_data.value();

  const auto acceleration = vehicle.getCurrentAcceleration();
  const auto [min_acceleration, max_acceleration] = getAccelerationLimits(acceleration);
  const auto speed = vehicle.getCurrentSpeed();
  const auto desired_acceleration = getDesiredAcceleration(
    remaining_time_to_timed_waypoint, acceleration, distance_to_timed_waypoint, speed);
  const auto desired_speed =
    getDesiredSpeed(desired_acceleration, min_acceleration, max_acceleration, speed);
  const auto remaining_time_to_arrival_to_front_waypoint = getTimeRemainingToFrontWaypoint(
    remaining_time_to_front_waypoint, distance_to_front_waypoint, desired_speed);
  if (!remaining_time_to_arrival_to_front_waypoint) {
    discardTheFrontWaypointFromTrajectory();
    return std::nullopt;
  }

  return std::make_tuple(target_position, desired_speed);
}

auto PolylineTrajectoryFollower::getTargetPosition() const -> geometry_msgs::msg::Point
{
  /*
       We've made sure that polyline_trajectory_m->shape.vertices is not empty,
       so a reference to vertices.front() always succeeds. vertices.front() is
       referenced only this once in this member function.
    */
  const auto target_position = polyline_trajectory_m->shape.vertices.front().position.position;

  if (any(is_infinity_or_nan, target_position)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(vehicle.getName()),
      "'s target position coordinate value contains NaN or infinity. The value is [",
      target_position.x, ", ", target_position.y, ", ", target_position.z, "].");
  }
  return target_position;
}

auto PolylineTrajectoryFollower::getAccelerationLimits(double acceleration) const
  -> std::tuple<double, double>
{
  const auto max_acceleration = std::min(
    acceleration /* [m/s^2] */ +
      behavior_parameter_m.dynamic_constraints.max_acceleration_rate /* [m/s^3] */ *
        step_time_m /* [s] */,
    +behavior_parameter_m.dynamic_constraints.max_acceleration /* [m/s^2] */);

  if (std::isinf(max_acceleration) or std::isnan(max_acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(vehicle.getName()),
      "'s maximum acceleration value is NaN or infinity. The value is ", max_acceleration, ".");
  }

  const auto min_acceleration = std::max(
    acceleration /* [m/s^2] */ -
      behavior_parameter_m.dynamic_constraints.max_deceleration_rate /* [m/s^3] */ *
        step_time_m /* [s] */,
    -behavior_parameter_m.dynamic_constraints.max_deceleration /* [m/s^2] */);

  if (std::isinf(min_acceleration) or std::isnan(min_acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(vehicle.getName()),
      "'s minimum acceleration value is NaN or infinity. The value is ", min_acceleration, ".");
  }

  return std::make_tuple(min_acceleration, max_acceleration);
}

auto PolylineTrajectoryFollower::getTimeRemainingToFrontWaypoint(
  double remaining_time_to_front_waypoint, double distance_to_front_waypoint,
  double desired_speed) const -> std::optional<double>
{
  /*
       It's okay for this value to be infinite.
    */
  const auto remaining_time_to_arrival_to_front_waypoint =
    distance_to_front_waypoint / desired_speed;  // [s]

  /*
       If the target point is reached during this step, it is considered
       reached.
    */
  if (math::arithmetic::isDefinitelyLessThan(
        remaining_time_to_arrival_to_front_waypoint, step_time_m)) {
    /*
         The condition "Is remaining time to front waypoint less than remaining
         time to arrival to front waypoint + step time?" means "If arrival is
         next frame, is it too late?". This clause is executed only if the
         front waypoint is expected to arrive during this frame. That is, the
         conjunction of these conditions means "Did the vehicle arrive at the
         front waypoint exactly on time?" Otherwise the vehicle will have
         reached the front waypoint too early.

         This means that the vehicle did not slow down enough to reach the
         current waypoint after passing the previous waypoint. In order to cope
         with such situations, it is necessary to perform speed planning
         considering not only the front of the waypoint queue but also the
         waypoints after it. However, even with such speed planning, there is a
         possibility that on-time arrival may not be possible depending on the
         relationship between waypoint intervals, specified arrival times,
         vehicle parameters, and dynamic restraints. For example, there is a
         situation in which a large speed change is required over a short
         distance while the permissible jerk is small.

         This implementation does simple velocity planning that considers only
         the nearest waypoints in favor of simplicity of implementation.

         Note: There is no need to consider the case of arrival too late.
         Because that case has already been verified when calculating the
         remaining time.

         Note: If remaining time to front waypoint is nan, there is no need to
         verify whether the arrival is too early. This arrival determination is
         only interesting for the front waypoint. Verifying whether or not the
         arrival time is specified in the front waypoint is exactly the
         condition of "Is remaining time to front waypoint nan?"
      */
    if (
      std::isnan(remaining_time_to_front_waypoint) or
      math::arithmetic::isDefinitelyLessThan(
        remaining_time_to_front_waypoint,
        remaining_time_to_arrival_to_front_waypoint + step_time_m)) {
      return std::nullopt;
    } else {
      throw common::SimulationError(
        "Vehicle ", std::quoted(vehicle.getName()), " arrived at the waypoint in trajectory ",
        remaining_time_to_front_waypoint,
        " seconds earlier than the specified time. This may be due to unrealistic conditions of "
        "arrival time specification compared to vehicle parameters and dynamic constraints.");
    }
  }

  return remaining_time_to_arrival_to_front_waypoint;
}

auto PolylineTrajectoryFollower::getDesiredSpeed(
  double desired_acceleration, double min_acceleration, double max_acceleration, double speed) const
  -> double
{
  /*
       However, the desired acceleration is unrealistically large in terms of
       vehicle performance and dynamic constraints, so it is clamped to a
       realistic value.
    */
  const auto desired_speed =
    speed + std::clamp(desired_acceleration, min_acceleration, max_acceleration) * step_time_m;
  if (std::isinf(desired_speed) or std::isnan(desired_speed)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(vehicle.getName()), "'s desired speed value is NaN or infinity. The value is ",
      desired_speed, ".");
  }
  return desired_speed;
}

auto PolylineTrajectoryFollower::getDesiredAcceleration(
  double remaining_time, double acceleration, double distance, double speed) const -> double
{
  /*
       The desired acceleration is the acceleration at which the destination
       can be reached exactly at the specified time (= time remaining at zero).

       If no arrival time is specified for subsequent waypoints, there is no
       need to accelerate or decelerate, so the current acceleration will be
       the desired speed.
    */
  const double desired_acceleration = [&]() {
    if (std::isinf(remaining_time)) {
      return acceleration;  /// @todo Accelerate to match speed with `target_speed`.
    } else {
      /*
                        v [m/s]
                          ^
                          |
            desired_speed +   /|
                          |  / |
                          | /  |
                    speed +/   |
                          |    |
                          |    |
                          +----+-------------> t [s]
                        0     remaining_time

            desired_speed = speed + desired_acceleration * remaining_time

            distance = (speed + desired_speed) * remaining_time * 1/2

                    = (speed + speed + desired_acceleration * remaining_time) * remaining_time * 1/2

                    = speed * remaining_time + desired_acceleration * remaining_time^2 * 1/2
        */
      return 2 * distance / std::pow(remaining_time, 2) - 2 * speed / remaining_time;
    }
  }();

  if (std::isinf(desired_acceleration) or std::isnan(desired_acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(vehicle.getName()),
      "'s desired acceleration value contains NaN or infinity. The value is ", desired_acceleration,
      ".");
  }

  return desired_acceleration;
}

}  // namespace follow_trajectory
}  // namespace traffic_simulator
