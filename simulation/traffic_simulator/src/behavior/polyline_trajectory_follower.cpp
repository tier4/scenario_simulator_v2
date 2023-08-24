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

#include <quaternion_operation/quaternion_operation.h>

#include <arithmetic/floating_point/comparison.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/normalize.hpp>
#include <geometry/vector3/operator.hpp>
#include <geometry/vector3/truncate.hpp>
#include <iostream>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/behavior/polyline_trajectory_follower.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{
template <typename F, typename T, typename... Ts>
auto any(F f, T && x, Ts &&... xs)
{
  if constexpr (math::geometry::IsLikeVector3<std::decay_t<decltype(x)>>::value) {
    return any(f, x.x, x.y, x.z);
  } else if constexpr (0 < sizeof...(xs)) {
    return f(x) or any(f, std::forward<decltype(xs)>(xs)...);
  } else {
    return f(x);
  }
}

auto is_infinity_or_nan = [](auto x) constexpr { return std::isinf(x) or std::isnan(x); };

auto PolylineTrajectoryFollower::setParameters(
  const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter, const double step_time,
  const std::optional<traffic_simulator_msgs::msg::VehicleParameters> &
    vehicle_parameters /* = std::nullopt */) -> void
{
  entity_status_m = entity_status;
  behavior_parameter_m = behavior_parameter;
  step_time_m = step_time;
  vehicle_parameters_m = vehicle_parameters;
}

std::optional<traffic_simulator_msgs::msg::EntityStatus>
PolylineTrajectoryFollower::followTrajectory(
  std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & polyline_trajectory)
{
  if (polyline_trajectory->shape.vertices.empty()) {
    return std::nullopt;
  }

  polyline_trajectory_m = polyline_trajectory;
  const auto position = getCurrentPosition();
  const auto target_position = getTargetPosition();
  const auto front_waypoint_data = getDistanceAndTimeToFrontWaypoint(target_position, position);
  if (!front_waypoint_data) {
    discardTheFrontWaypointFromTrajectory();
    return followTrajectory(polyline_trajectory_m);
  }
  const auto [distance_to_front_waypoint, remaining_time_to_front_waypoint] =
    front_waypoint_data.value();

  const auto timed_waypoint_data =
    getDistanceAndTimeToWaypointWithSpecifiedTime(distance_to_front_waypoint);
  if (!timed_waypoint_data) {
    discardTheFrontWaypointFromTrajectory();
    return followTrajectory(polyline_trajectory_m);
  }
  const auto [distance_to_timed_waypoint, remaining_time_to_timed_waypoint] =
    timed_waypoint_data.value();

  const auto acceleration = getCurrentAcceleration();
  const auto [min_acceleration, max_acceleration] = getAccelerationLimits(acceleration);
  const auto speed = getCurrentSpeed();
  const auto desired_acceleration = getDesiredAcceleration(
    remaining_time_to_timed_waypoint, acceleration, distance_to_timed_waypoint, speed);
  const auto desired_speed =
    getDesiredSpeed(desired_acceleration, min_acceleration, max_acceleration, speed);
  const auto desired_velocity = getDesiredVelocity(target_position, position, desired_speed);

  const auto remaining_time_to_arrival_to_front_waypoint = getTimeRemainingToFrontWaypoint(
    remaining_time_to_front_waypoint, distance_to_front_waypoint, desired_speed);
  if (!remaining_time_to_arrival_to_front_waypoint) {
    discardTheFrontWaypointFromTrajectory();
    return followTrajectory(polyline_trajectory_m);
  }

  const auto velocity = getUpdatedVelocity(desired_velocity, desired_speed); 

  if constexpr (false) {
    // clang-format off
      using math::geometry::operator+;
      using math::geometry::operator-;
      using math::geometry::operator*;
      using math::geometry::operator/;

      std::cout << std::fixed << std::boolalpha << std::string(80, '-') << std::endl;

      std::cout << "acceleration "
                << "== " << acceleration
                << std::endl;

      std::cout << "min_acceleration "
                << "== std::max(acceleration - max_deceleration_rate * step_time, -max_deceleration) "
                << "== std::max(" << acceleration << " - " << behavior_parameter_m.dynamic_constraints.max_deceleration_rate << " * " << step_time_m << ", " << -behavior_parameter_m.dynamic_constraints.max_deceleration << ") "
                << "== std::max(" << acceleration << " - " << behavior_parameter_m.dynamic_constraints.max_deceleration_rate * step_time_m << ", " << -behavior_parameter_m.dynamic_constraints.max_deceleration << ") "
                << "== std::max(" << (acceleration - behavior_parameter_m.dynamic_constraints.max_deceleration_rate * step_time_m) << ", " << -behavior_parameter_m.dynamic_constraints.max_deceleration << ") "
                << "== " << min_acceleration
                << std::endl;

      std::cout << "max_acceleration "
                << "== std::min(acceleration + max_acceleration_rate * step_time, +max_acceleration) "
                << "== std::min(" << acceleration << " + " << behavior_parameter_m.dynamic_constraints.max_acceleration_rate << " * " << step_time_m << ", " << behavior_parameter_m.dynamic_constraints.max_acceleration << ") "
                << "== std::min(" << acceleration << " + " << behavior_parameter_m.dynamic_constraints.max_acceleration_rate * step_time_m << ", " << behavior_parameter_m.dynamic_constraints.max_acceleration << ") "
                << "== std::min(" << (acceleration + behavior_parameter_m.dynamic_constraints.max_acceleration_rate * step_time_m) << ", " << behavior_parameter_m.dynamic_constraints.max_acceleration << ") "
                << "== " << max_acceleration
                << std::endl;

      std::cout << "min_acceleration < acceleration < max_acceleration "
                << "== " << min_acceleration << " < " << acceleration << " < " << max_acceleration << std::endl;

      std::cout << "desired_acceleration "
                << "== 2 * distance / std::pow(remaining_time, 2) - 2 * speed / remaining_time "
                << "== 2 * " << distance_to_timed_waypoint << " / " << std::pow(remaining_time_to_timed_waypoint, 2) << " - 2 * " << speed << " / " << remaining_time_to_timed_waypoint << " "
                << "== " << (2 * distance_to_timed_waypoint / std::pow(remaining_time_to_timed_waypoint, 2)) << " - " << (2 * speed / remaining_time_to_timed_waypoint) << " "
                << "== " << desired_acceleration << " "
                << "(acceleration < desired_acceleration == " << (acceleration < desired_acceleration) << " == need to " <<(acceleration < desired_acceleration ? "accelerate" : "decelerate") << ")"
                << std::endl;

      std::cout << "desired_speed "
                << "== speed + std::clamp(desired_acceleration, min_acceleration, max_acceleration) * step_time "
                << "== " << speed << " + std::clamp(" << desired_acceleration << ", " << min_acceleration << ", " << max_acceleration << ") * " << step_time_m << " "
                << "== " << speed << " + " << std::clamp(desired_acceleration, min_acceleration, max_acceleration) << " * " << step_time_m << " "
                << "== " << speed << " + " << std::clamp(desired_acceleration, min_acceleration, max_acceleration) * step_time_m << " "
                << "== " << desired_speed
                << std::endl;

      std::cout << "distance_to_front_waypoint "
                << "== " << distance_to_front_waypoint
                << std::endl;

      std::cout << "remaining_time_to_arrival_to_front_waypoint "
                << "== " << remaining_time_to_arrival_to_front_waypoint.value()
                << std::endl;

      std::cout << "distance "
                << "== " << distance_to_timed_waypoint
                << std::endl;

      std::cout << "remaining_time "
                << "== " << remaining_time_to_timed_waypoint
                << std::endl;

      std::cout << "remaining_time_to_arrival_to_front_waypoint "
                << "("
                << "== distance_to_front_waypoint / desired_speed "
                << "== " << distance_to_front_waypoint << " / " << desired_speed << " "
                << "== " << remaining_time_to_arrival_to_front_waypoint.value()
                << ")"
                << std::endl;

      std::cout << "arrive during this frame? "
                << "== remaining_time_to_arrival_to_front_waypoint < step_time "
                << "== " << remaining_time_to_arrival_to_front_waypoint.value() << " < " << step_time_m << " "
                << "== " << math::arithmetic::isDefinitelyLessThan(remaining_time_to_arrival_to_front_waypoint.value(), step_time_m)
                << std::endl;

      std::cout << "not too early? "
                << "== std::isnan(remaining_time_to_front_waypoint) or remaining_time_to_front_waypoint < remaining_time_to_arrival_to_front_waypoint + step_time "
                << "== std::isnan(" << remaining_time_to_front_waypoint << ") or " << remaining_time_to_front_waypoint << " < " << remaining_time_to_arrival_to_front_waypoint.value() << " + " << step_time_m << " "
                << "== " << std::isnan(remaining_time_to_front_waypoint) << " or " << math::arithmetic::isDefinitelyLessThan(remaining_time_to_front_waypoint, remaining_time_to_arrival_to_front_waypoint.value() + step_time_m) << " "
                << "== " << (std::isnan(remaining_time_to_front_waypoint) or math::arithmetic::isDefinitelyLessThan(remaining_time_to_front_waypoint, remaining_time_to_arrival_to_front_waypoint.value() + step_time_m))
                << std::endl;
    // clang-format on
  }
  return createUpdatedEntityStatus(velocity);
}

auto PolylineTrajectoryFollower::getCurrentPosition() const -> geometry_msgs::msg::Point
{
  const auto position = entity_status_m.pose.position;

  if (any(is_infinity_or_nan, position)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status_m.name),
      " coordinate value contains NaN or infinity. The value is [", position.x, ", ", position.y,
      ", ", position.z, "].");
  }

  return position;
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
      std::quoted(entity_status_m.name),
      "'s target position coordinate value contains NaN or infinity. The value is [",
      target_position.x, ", ", target_position.y, ", ", target_position.z, "].");
  }
  return target_position;
}

auto PolylineTrajectoryFollower::getCurrentAcceleration() const -> double
{
  const auto acceleration = entity_status_m.action_status.accel.linear.x;  // m/s^2

  if (std::isinf(acceleration) or std::isnan(acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status_m.name), "'s acceleration value is NaN or infinity. The value is ",
      acceleration, ".");
  }
  return acceleration;
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
      std::quoted(entity_status_m.name),
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
      std::quoted(entity_status_m.name),
      "'s minimum acceleration value is NaN or infinity. The value is ", min_acceleration, ".");
  }

  return std::make_tuple(min_acceleration, max_acceleration);
}

auto PolylineTrajectoryFollower::getCurrentSpeed() const -> double
{
  const auto speed = entity_status_m.action_status.twist.linear.x;  // [m/s]
  if (std::isinf(speed) or std::isnan(speed)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status_m.name), "'s speed value is NaN or infinity. The value is ", speed,
      ".");
  }
  return speed;
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
        "Vehicle ", std::quoted(entity_status_m.name), " arrived at the waypoint in trajectory ",
        remaining_time_to_front_waypoint,
        " seconds earlier than the specified time. This may be due to unrealistic conditions of "
        "arrival time specification compared to vehicle parameters and dynamic constraints.");
    }
  }

  return remaining_time_to_arrival_to_front_waypoint;
}

auto PositionModePolylineTrajectoryFollower::createUpdatedEntityStatus(
  const geometry_msgs::msg::Vector3 & velocity) const -> traffic_simulator_msgs::msg::EntityStatus
{
  using math::geometry::operator/;
  using math::geometry::operator-;
  using math::geometry::operator*;
  using math::geometry::operator+=;

  auto updated_status = entity_status_m;

  updated_status.pose.position += velocity * step_time_m;

  updated_status.pose.orientation = [&]() {
    geometry_msgs::msg::Vector3 direction;
    direction.x = 0;
    direction.y = 0;
    direction.z = std::atan2(velocity.y, velocity.x);
    return quaternion_operation::convertEulerAngleToQuaternion(direction);
  }();

  updated_status.action_status.twist.linear.x = math::geometry::norm(velocity);

  updated_status.action_status.twist.linear.y = 0;

  updated_status.action_status.twist.linear.z = 0;

  updated_status.action_status.twist.angular =
    quaternion_operation::convertQuaternionToEulerAngle(quaternion_operation::getRotation(
      entity_status_m.pose.orientation, updated_status.pose.orientation)) /
    step_time_m;

  updated_status.action_status.accel.linear =
    (updated_status.action_status.twist.linear - entity_status_m.action_status.twist.linear) /
    step_time_m;
  
  updated_status.action_status.accel.angular =
    (updated_status.action_status.twist.angular - entity_status_m.action_status.twist.angular) /
    step_time_m;

  updated_status.time = entity_status_m.time + step_time_m;

  updated_status.lanelet_pose_valid = false;
  return updated_status;
}

auto PositionModePolylineTrajectoryFollower::getUpdatedVelocity(
  const geometry_msgs::msg::Vector3 & desired_velocity, double desired_speed) const
  -> geometry_msgs::msg::Vector3
{
  using math::geometry::operator*;
  using math::geometry::operator-;
  using math::geometry::operator+;

  const auto current_velocity =
    quaternion_operation::convertQuaternionToEulerAngle(entity_status_m.pose.orientation) *
    entity_status_m.action_status.twist.linear.x;

  /*
       Note: If obstacle avoidance is to be implemented, the steering behavior
       known by the name "collision avoidance" should be synthesized here into
       steering.
    */
  const auto steering = desired_velocity - current_velocity;

  const auto velocity = math::geometry::truncate(current_velocity + steering, desired_speed);

  return velocity;
}

auto PositionModePolylineTrajectoryFollower::getDesiredAcceleration(
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
      std::quoted(entity_status_m.name),
      "'s desired acceleration value contains NaN or infinity. The value is ", desired_acceleration,
      ".");
  }

  return desired_acceleration;
}

auto PositionModePolylineTrajectoryFollower::getDesiredSpeed(
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
      std::quoted(entity_status_m.name), "'s desired speed value is NaN or infinity. The value is ",
      desired_speed, ".");
  }
  return desired_speed;
}

auto PositionModePolylineTrajectoryFollower::getDesiredVelocity(
  const geometry_msgs::msg::Point & target_position, const geometry_msgs::msg::Point & position,
  double desired_speed) -> geometry_msgs::msg::Vector3
{
  using math::geometry::operator-;
  using math::geometry::operator*;

  const geometry_msgs::msg::Vector3 desired_velocity =
    math::geometry::normalize(target_position - position) * desired_speed;

  if (any(is_infinity_or_nan, desired_velocity)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status_m.name),
      "'s desired velocity contains NaN or infinity. The value is [", desired_velocity.x, ", ",
      desired_velocity.y, ", ", desired_velocity.z, "].");
  }

  return desired_velocity;
}

auto PositionModePolylineTrajectoryFollower::getDistanceAndTimeToFrontWaypoint(
  const geometry_msgs::msg::Point & target_position,
  const geometry_msgs::msg::Point & position) const -> std::optional<std::tuple<double, double>>
{
  const auto [distance_to_front_waypoint, remaining_time_to_front_waypoint] = std::make_tuple(
    math::geometry::hypot(position, target_position),
    (not std::isnan(polyline_trajectory_m->base_time) ? polyline_trajectory_m->base_time : 0.0) +
      polyline_trajectory_m->shape.vertices.front().time - entity_status_m.time);
  /*
       This clause is to avoid division-by-zero errors in later clauses with
       distance_to_front_waypoint as the denominator if the distance
       miraculously becomes zero.
    */
  if (math::arithmetic::isApproximatelyEqualTo(distance_to_front_waypoint, 0.0)) {
    return std::nullopt;
  }
  return std::make_tuple(distance_to_front_waypoint, remaining_time_to_front_waypoint);
}

auto PositionModePolylineTrajectoryFollower::getDistanceAndTimeToWaypointWithSpecifiedTime(
  double distance_to_front_waypoint) const -> std::optional<std::tuple<double, double>>
{
  const auto [distance, remaining_time] = [&]() {
    if (const auto first_waypoint_with_arrival_time_specified = std::find_if(
          std::begin(polyline_trajectory_m->shape.vertices),
          std::end(polyline_trajectory_m->shape.vertices),
          [](auto && vertex) { return not std::isnan(vertex.time); });
        first_waypoint_with_arrival_time_specified !=
        std::end(polyline_trajectory_m->shape.vertices)) {
      /*
            Note for anyone working on adding support for followingMode follow
            to this function (FollowPolylineTrajectoryAction::tick) in the
            future: if followingMode is follow, this distance calculation may be
            inappropriate.
        */
      auto total_distance_to = [&](auto last) {
        auto total_distance = 0.0;
        for (auto iter = std::begin(polyline_trajectory_m->shape.vertices);
             0 < std::distance(iter, last); ++iter) {
          total_distance +=
            math::geometry::hypot(iter->position.position, std::next(iter)->position.position);
        }
        return total_distance;
      };

      if (const auto remaining_time =
            (not std::isnan(polyline_trajectory_m->base_time) ? polyline_trajectory_m->base_time
                                                              : 0.0) +
            first_waypoint_with_arrival_time_specified->time - entity_status_m.time;
          /*
                The condition below should ideally be remaining_time < 0.

                The simulator runs at a constant frame rate, so the step time is
                1/FPS. If the simulation time is an accumulation of step times
                expressed as rational numbers, times that are integer multiples
                of the frame rate will always be exact integer seconds.
                Therefore, the timing of remaining_time == 0 always exists, and
                the velocity planning of this member function (tick) aims to
                reach the waypoint exactly at that timing. So the ideal timeout
                condition is remaining_time < 0.

                But actually the step time is expressed as a float and the
                simulation time is its accumulation. As a result, it is not
                guaranteed that there will be times when the simulation time is
                exactly zero. For example, remaining_time == -0.00006 and it was
                judged to be out of time.

                For the above reasons, the condition is remaining_time <
                -step_time. In other words, the conditions are such that a delay
                of 1 step time is allowed.
            */
          remaining_time < -step_time_m) {
        throw common::Error(
          "Vehicle ", std::quoted(entity_status_m.name),
          " failed to reach the trajectory waypoint at the specified time. The specified time "
          "is ",
          first_waypoint_with_arrival_time_specified->time, " (in ",
          (not std::isnan(polyline_trajectory_m->base_time) ? "absolute" : "relative"),
          " simulation time). This may be due to unrealistic conditions of arrival time "
          "specification compared to vehicle parameters and dynamic constraints.");
      } else {
        return std::make_tuple(
          distance_to_front_waypoint +
            total_distance_to(first_waypoint_with_arrival_time_specified),
          remaining_time);
      }
    } else {
      return std::make_tuple(distance_to_front_waypoint, std::numeric_limits<double>::infinity());
    }
  }();
  if (math::arithmetic::isApproximatelyEqualTo(distance, 0.0)) {
    return std::nullopt;
  }

  return std::make_tuple(distance, remaining_time);
}

void PositionModePolylineTrajectoryFollower::discardTheFrontWaypointFromTrajectory()
{
  //        The OpenSCENARIO standard does not define the behavior when the value of
  //        Timing.domainAbsoluteRelative is "relative". The standard only states
  //        "Definition of time value context as either absolute or relative", and
  //        it is completely unclear when the relative time starts.

  //        This implementation has interpreted the specification as follows:
  //        Relative time starts from the start of FollowTrajectoryAction or from
  //        the time of reaching the previous "waypoint with arrival time".

  //         Note: not std::isnan(polyline_trajectory_m->base_time) means
  //         "Timing.domainAbsoluteRelative is relative".

  //         Note: not std::isnan(polyline_trajectory_m->shape.vertices.front().time)
  //         means "The waypoint about to be popped is the waypoint with the
  //         specified arrival time".
  //

  if (
    not std::isnan(polyline_trajectory_m->base_time) and
    not std::isnan(polyline_trajectory_m->shape.vertices.front().time)) {
    polyline_trajectory_m->base_time = entity_status_m.time;
  }

  if (std::rotate(
        std::begin(polyline_trajectory_m->shape.vertices),
        std::begin(polyline_trajectory_m->shape.vertices) + 1,
        std::end(polyline_trajectory_m->shape.vertices));
      not polyline_trajectory_m->closed) {
    polyline_trajectory_m->shape.vertices.pop_back();
  }
}

auto FollowModePolylineTrajectoryFollower::getDistanceAndTimeToFrontWaypoint(
  const geometry_msgs::msg::Point & target_position,
  const geometry_msgs::msg::Point & position) const -> std::optional<std::tuple<double, double>>
{
  // todo distance and time to traget

  const auto [distance_to_front_waypoint, remaining_time_to_front_waypoint] = std::make_tuple(
    math::geometry::hypot(position, target_position), // TODO DIFFERENT DISTANCE CALCULATION
    (not std::isnan(polyline_trajectory_m->base_time) ? polyline_trajectory_m->base_time : 0.0) +
      polyline_trajectory_m->shape.vertices.front().time - entity_status_m.time);
  /*
       This clause is to avoid division-by-zero errors in later clauses with
       distance_to_front_waypoint as the denominator if the distance
       miraculously becomes zero.
    */
  // if (math::arithmetic::isApproximatelyEqualTo(distance_to_front_waypoint, 0.0)) {
  //   return std::nullopt;
  // }
  if(std::abs(distance_to_front_waypoint) <= 2.0)
  {
    return std::nullopt;
  }
  return std::make_tuple(distance_to_front_waypoint, remaining_time_to_front_waypoint);
}

auto FollowModePolylineTrajectoryFollower::getDistanceAndTimeToWaypointWithSpecifiedTime(
  double distance_to_front_waypoint) const -> std::optional<std::tuple<double, double>>
{
  const auto [distance, remaining_time] = [&]() {
    if (const auto first_waypoint_with_arrival_time_specified = std::find_if(
          std::begin(polyline_trajectory_m->shape.vertices),
          std::end(polyline_trajectory_m->shape.vertices),
          [](auto && vertex) { return not std::isnan(vertex.time); });
        first_waypoint_with_arrival_time_specified !=
        std::end(polyline_trajectory_m->shape.vertices)) {
      /*
            Note for anyone working on adding support for followingMode follow
            to this function (FollowPolylineTrajectoryAction::tick) in the
            future: if followingMode is follow, this distance calculation may be
            inappropriate.
        */
      auto total_distance_to = [&](auto last) {
        auto total_distance = 0.0;
        for (auto iter = std::begin(polyline_trajectory_m->shape.vertices);
             0 < std::distance(iter, last); ++iter) {
          total_distance +=
            math::geometry::hypot(iter->position.position, std::next(iter)->position.position);
        }
        return total_distance;
      };

      if (const auto remaining_time =
            (not std::isnan(polyline_trajectory_m->base_time) ? polyline_trajectory_m->base_time
                                                              : 0.0) +
            first_waypoint_with_arrival_time_specified->time - entity_status_m.time;
          /*
                The condition below should ideally be remaining_time < 0.

                The simulator runs at a constant frame rate, so the step time is
                1/FPS. If the simulation time is an accumulation of step times
                expressed as rational numbers, times that are integer multiples
                of the frame rate will always be exact integer seconds.
                Therefore, the timing of remaining_time == 0 always exists, and
                the velocity planning of this member function (tick) aims to
                reach the waypoint exactly at that timing. So the ideal timeout
                condition is remaining_time < 0.

                But actually the step time is expressed as a float and the
                simulation time is its accumulation. As a result, it is not
                guaranteed that there will be times when the simulation time is
                exactly zero. For example, remaining_time == -0.00006 and it was
                judged to be out of time.

                For the above reasons, the condition is remaining_time <
                -step_time. In other words, the conditions are such that a delay
                of 1 step time is allowed.
            */
          remaining_time < -step_time_m) {
        // throw common::Error(
        //   "Vehicle ", std::quoted(entity_status_m.name),
        //   " failed to reach the trajectory waypoint at the specified time. The specified time "
        //   "is ",
        //   first_waypoint_with_arrival_time_specified->time, " (in ",
        //   (not std::isnan(polyline_trajectory_m->base_time) ? "absolute" : "relative"),
        //   " simulation time). This may be due to unrealistic conditions of arrival time "
        //   "specification compared to vehicle parameters and dynamic constraints.");
        return std::make_tuple(
          distance_to_front_waypoint +
            total_distance_to(first_waypoint_with_arrival_time_specified),
          remaining_time + step_time_m);
      } else {
        return std::make_tuple(
          distance_to_front_waypoint +
            total_distance_to(first_waypoint_with_arrival_time_specified),
          remaining_time);
      }
    } else {
      return std::make_tuple(distance_to_front_waypoint, std::numeric_limits<double>::infinity());
    }
  }();
  // if (math::arithmetic::isApproximatelyEqualTo(distance, 0.0)) {
  //   return std::nullopt;
  // }
  if(std::abs(distance) <= 2.0)
  {
    return std::nullopt;
  }
  return std::make_tuple(distance, remaining_time);
}

auto FollowModePolylineTrajectoryFollower::createUpdatedEntityStatus(
  const geometry_msgs::msg::Vector3 & velocity) const -> traffic_simulator_msgs::msg::EntityStatus
{
  using math::geometry::operator/;
  using math::geometry::operator-;
  using math::geometry::operator*;
  using math::geometry::operator+=;

  auto updated_status = entity_status_m;

  auto position_change = [&](){
    geometry_msgs::msg::Vector3 change;
    change.x = std::cos(quaternion_operation::convertQuaternionToEulerAngle(updated_status.pose.orientation).z) * math::geometry::norm(velocity) * step_time_m;
    change.y = std::sin(quaternion_operation::convertQuaternionToEulerAngle(updated_status.pose.orientation).z) * math::geometry::norm(velocity) * step_time_m;
    change.z = 0.0;
    return change;
  }();

  updated_status.pose.position += position_change;

  updated_status.pose.orientation = [&]() {
    geometry_msgs::msg::Vector3 direction;
    direction.x = 0;
    direction.y = 0;
    direction.z = quaternion_operation::convertQuaternionToEulerAngle(updated_status.pose.orientation).z + math::geometry::norm(velocity) * std::tan(steering) * step_time_m / vehicle_parameters_m.value().axles.front_axle.position_x;
    
    return quaternion_operation::convertEulerAngleToQuaternion(direction);
  }();

  updated_status.action_status.twist.linear.x = math::geometry::norm(velocity);

  updated_status.action_status.twist.linear.y = 0;

  updated_status.action_status.twist.linear.z = 0;

  updated_status.action_status.twist.angular =
    quaternion_operation::convertQuaternionToEulerAngle(quaternion_operation::getRotation(
      entity_status_m.pose.orientation, updated_status.pose.orientation)) /
    step_time_m;

  updated_status.action_status.accel.linear =
    (updated_status.action_status.twist.linear - entity_status_m.action_status.twist.linear) /
    step_time_m;
  
  updated_status.action_status.accel.angular =
    (updated_status.action_status.twist.angular - entity_status_m.action_status.twist.angular) /
    step_time_m;

  updated_status.time = entity_status_m.time + step_time_m;

  updated_status.lanelet_pose_valid = false;

  return updated_status;
}

void FollowModePolylineTrajectoryFollower::discardTheFrontWaypointFromTrajectory()
{
  //        The OpenSCENARIO standard does not define the behavior when the value of
  //        Timing.domainAbsoluteRelative is "relative". The standard only states
  //        "Definition of time value context as either absolute or relative", and
  //        it is completely unclear when the relative time starts.

  //        This implementation has interpreted the specification as follows:
  //        Relative time starts from the start of FollowTrajectoryAction or from
  //        the time of reaching the previous "waypoint with arrival time".
  //

  if (
    not std::isnan(polyline_trajectory_m->base_time) and
    not std::isnan(polyline_trajectory_m->shape.vertices.front().time)) {
    polyline_trajectory_m->base_time = entity_status_m.time;
  }

  previous_target = polyline_trajectory_m->shape.vertices.front().position.position;

  if (std::rotate(
        std::begin(polyline_trajectory_m->shape.vertices),
        std::begin(polyline_trajectory_m->shape.vertices) + 1,
        std::end(polyline_trajectory_m->shape.vertices));
      not polyline_trajectory_m->closed) {
        std::cout<<"pop back"<<std::endl;
    polyline_trajectory_m->shape.vertices.pop_back();
  }
}

auto FollowModePolylineTrajectoryFollower::getDesiredAcceleration(
  double remaining_time, double acceleration, double distance, double speed) const -> double
{
  // if( remaining_time == acceleration && distance == speed)
  // {
  //   std::cout<<":00"<<std::endl;
  // }
  // return 0.05;
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
      std::quoted(entity_status_m.name),
      "'s desired acceleration value contains NaN or infinity. The value is ", desired_acceleration,
      ".");
  }

  return desired_acceleration;
}

auto FollowModePolylineTrajectoryFollower::getDesiredSpeed(
  double desired_acceleration, double min_acceleration, double max_acceleration, double speed) const
  -> double
{
  // if( desired_acceleration == min_acceleration && max_acceleration == speed)
  // {
  //   return 0.05;
  // }
  // return 0.05;
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
      std::quoted(entity_status_m.name), "'s desired speed value is NaN or infinity. The value is ",
      desired_speed, ".");
  }
  return desired_speed;
}

double normalizeAngle(double angle) {
    angle = std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
    if (angle > M_PI / 2.0) {
        angle = M_PI - angle;
    } else if (angle < -M_PI / 2.0) {
        angle = -M_PI - angle;
    }
    return angle;
}

auto FollowModePolylineTrajectoryFollower::getDesiredVelocity(
  const geometry_msgs::msg::Point & target_position, const geometry_msgs::msg::Point & position,
  double desired_speed) -> geometry_msgs::msg::Vector3
{
  using math::geometry::operator-;
  using math::geometry::operator*;
  using math::geometry::operator+;

  auto theta = quaternion_operation::convertQuaternionToEulerAngle(entity_status_m.pose.orientation).z;
  auto front_axle_position = [&]() {
    geometry_msgs::msg::Vector3 front_axle_coords;
    front_axle_coords.x = position.x + std::cos(theta)*vehicle_parameters_m.value().axles.front_axle.position_x;
    front_axle_coords.y = position.y + std::sin(theta)*vehicle_parameters_m.value().axles.front_axle.position_x;
    front_axle_coords.z = 0.0;
    return front_axle_coords;
  }();


  if (!previous_target)
  {
    previous_target = geometry_msgs::msg::Point();
    previous_target->x = front_axle_position.x;
    previous_target->y = front_axle_position.y;
    previous_target->z = front_axle_position.z;
  }

  auto a = previous_target.value().y - target_position.y;
  auto b = target_position.x - previous_target.value().x;
  auto c = previous_target.value().x*target_position.y - target_position.x*previous_target.value().y;

  auto gain = 1;
  auto parameter = 1;
  auto heading_error = std::remainder(std::atan2(-1*a,b) - theta, 2*M_PI);
  auto cross_track_error = -1*(a*front_axle_position.x + b*front_axle_position.y + c)/std::sqrt(a*a + b*b);

  
  auto cross_track_steering = std::atan2(gain*cross_track_error,(parameter + desired_speed));
  auto steering_input = heading_error + cross_track_steering;

  steering_input = std::clamp(steering_input, vehicle_parameters_m.value().axles.front_axle.max_steering * -1, vehicle_parameters_m.value().axles.front_axle.max_steering);

  steering = steering_input;
  geometry_msgs::msg::Vector3 desired_velocity;
  desired_velocity.x = std::cos(theta);
  desired_velocity.y = std::sin(theta);
  desired_velocity.z = 0.0;
  desired_velocity = math::geometry::normalize(desired_velocity) * desired_speed;

  if (any(is_infinity_or_nan, desired_velocity)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status_m.name),
      "'s desired velocity contains NaN or infinity. The value is [", desired_velocity.x, ", ",
      desired_velocity.y, ", ", desired_velocity.z, "].");
  }

  return desired_velocity;
}

auto FollowModePolylineTrajectoryFollower::getUpdatedVelocity(
  const geometry_msgs::msg::Vector3 & desired_velocity, double desired_speed) const
  -> geometry_msgs::msg::Vector3
{
  if( desired_speed == 0.0)
  {
    std::cout<<"0"<<std::endl;
  }
  return desired_velocity;
}

}  // namespace follow_trajectory
}  // namespace traffic_simulator
