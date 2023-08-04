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

#include <quaternion_operation/quaternion_operation.h>

#include <arithmetic/floating_point/comparison.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/normalize.hpp>
#include <geometry/vector3/operator.hpp>
#include <geometry/vector3/truncate.hpp>
#include <iostream>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/behavior/follow_trajectory.hpp>

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

auto makeUpdatedStatus(
  const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & polyline_trajectory,
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter, double step_time)
  -> std::optional<traffic_simulator_msgs::msg::EntityStatus>
{
  using math::arithmetic::isApproximatelyEqualTo;
  using math::arithmetic::isDefinitelyLessThan;

  using math::geometry::operator+;
  using math::geometry::operator-;
  using math::geometry::operator*;
  using math::geometry::operator/;
  using math::geometry::operator+=;

  using math::geometry::hypot;
  using math::geometry::norm;
  using math::geometry::normalize;
  using math::geometry::truncate;

  auto discard_the_front_waypoint_and_recurse = [&]() {
    /*
       The OpenSCENARIO standard does not define the behavior when the value of
       Timing.domainAbsoluteRelative is "relative". The standard only states
       "Definition of time value context as either absolute or relative", and
       it is completely unclear when the relative time starts.

       This implementation has interpreted the specification as follows:
       Relative time starts from the start of FollowTrajectoryAction or from
       the time of reaching the previous "waypoint with arrival time".

       Note: not std::isnan(polyline_trajectory->base_time) means
       "Timing.domainAbsoluteRelative is relative".

       Note: not std::isnan(polyline_trajectory->shape.vertices.front().time)
       means "The waypoint about to be popped is the waypoint with the
       specified arrival time".
    */
    if (
      not std::isnan(polyline_trajectory->base_time) and
      not std::isnan(polyline_trajectory->shape.vertices.front().time)) {
      polyline_trajectory->base_time = entity_status.time;
    }

    if (std::rotate(
          std::begin(polyline_trajectory->shape.vertices),
          std::begin(polyline_trajectory->shape.vertices) + 1,
          std::end(polyline_trajectory->shape.vertices));
        not polyline_trajectory->closed) {
      polyline_trajectory->shape.vertices.pop_back();
    }

    return makeUpdatedStatus(entity_status, polyline_trajectory, behavior_parameter, step_time);
  };

  auto is_infinity_or_nan = [](auto x) constexpr { return std::isinf(x) or std::isnan(x); };

  /*
     The following code implements the steering behavior known as "seek". See
     "Steering Behaviors For Autonomous Characters" by Craig Reynolds for more
     information.

     See https://www.researchgate.net/publication/2495826_Steering_Behaviors_For_Autonomous_Characters
  */
  if (polyline_trajectory->shape.vertices.empty()) {
    return std::nullopt;
  } else if (const auto position = entity_status.pose.position; any(is_infinity_or_nan, position)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name), " coordinate value contains NaN or infinity. The value is [",
      position.x, ", ", position.y, ", ", position.z, "].");
  } else if (
    /*
       We've made sure that polyline_trajectory->shape.vertices is not empty,
       so a reference to vertices.front() always succeeds. vertices.front() is
       referenced only this once in this member function.
    */
    const auto target_position = polyline_trajectory->shape.vertices.front().position.position;
    any(is_infinity_or_nan, target_position)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name),
      "'s target position coordinate value contains NaN or infinity. The value is [",
      target_position.x, ", ", target_position.y, ", ", target_position.z, "].");
  } else if (
    /*
       If not dynamic_constraints_ignorable, the linear distance should cause
       problems.
    */
    const auto [distance_to_front_waypoint, remaining_time_to_front_waypoint] = std::make_tuple(
      hypot(position, target_position),
      (not std::isnan(polyline_trajectory->base_time) ? polyline_trajectory->base_time : 0.0) +
        polyline_trajectory->shape.vertices.front().time - entity_status.time);
    /*
       This clause is to avoid division-by-zero errors in later clauses with
       distance_to_front_waypoint as the denominator if the distance
       miraculously becomes zero.
    */
    isApproximatelyEqualTo(distance_to_front_waypoint, 0.0)) {
    return discard_the_front_waypoint_and_recurse();
  } else if (
    const auto [distance, remaining_time] =
      [&]() {
        if (const auto first_waypoint_with_arrival_time_specified = std::find_if(
              std::begin(polyline_trajectory->shape.vertices),
              std::end(polyline_trajectory->shape.vertices),
              [](auto && vertex) { return not std::isnan(vertex.time); });
            first_waypoint_with_arrival_time_specified !=
            std::end(polyline_trajectory->shape.vertices)) {
          /*
             Note for anyone working on adding support for followingMode follow
             to this function (FollowPolylineTrajectoryAction::tick) in the
             future: if followingMode is follow, this distance calculation may be
             inappropriate.
          */
          auto total_distance_to = [&](auto last) {
            auto total_distance = 0.0;
            for (auto iter = std::begin(polyline_trajectory->shape.vertices);
                 0 < std::distance(iter, last); ++iter) {
              total_distance += hypot(iter->position.position, std::next(iter)->position.position);
            }
            return total_distance;
          };

          if (const auto remaining_time =
                (not std::isnan(polyline_trajectory->base_time) ? polyline_trajectory->base_time
                                                                : 0.0) +
                first_waypoint_with_arrival_time_specified->time - entity_status.time;
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
              remaining_time < -step_time) {
            throw common::Error(
              "Vehicle ", std::quoted(entity_status.name),
              " failed to reach the trajectory waypoint at the specified time. The specified time "
              "is ",
              first_waypoint_with_arrival_time_specified->time, " (in ",
              (not std::isnan(polyline_trajectory->base_time) ? "absolute" : "relative"),
              " simulation time). This may be due to unrealistic conditions of arrival time "
              "specification compared to vehicle parameters and dynamic constraints.");
          } else {
            return std::make_tuple(
              distance_to_front_waypoint +
                total_distance_to(first_waypoint_with_arrival_time_specified),
              remaining_time);
          }
        } else {
          return std::make_tuple(
            distance_to_front_waypoint, std::numeric_limits<double>::infinity());
        }
      }();
    isApproximatelyEqualTo(distance, 0.0)) {
    return discard_the_front_waypoint_and_recurse();
  } else if (const auto acceleration = entity_status.action_status.accel.linear.x;  // [m/s^2]
             isinf(acceleration) or isnan(acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name), "'s acceleration value is NaN or infinity. The value is ",
      acceleration, ".");
  } else if (const auto max_acceleration = std::min(
               acceleration /* [m/s^2] */ +
                 behavior_parameter.dynamic_constraints.max_acceleration_rate /* [m/s^3] */ *
                   step_time /* [s] */,
               +behavior_parameter.dynamic_constraints.max_acceleration /* [m/s^2] */);
             isinf(max_acceleration) or isnan(max_acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name),
      "'s maximum acceleration value is NaN or infinity. The value is ", max_acceleration, ".");
  } else if (const auto min_acceleration = std::max(
               acceleration /* [m/s^2] */ -
                 behavior_parameter.dynamic_constraints.max_deceleration_rate /* [m/s^3] */ *
                   step_time /* [s] */,
               -behavior_parameter.dynamic_constraints.max_deceleration /* [m/s^2] */);
             isinf(min_acceleration) or isnan(min_acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name),
      "'s minimum acceleration value is NaN or infinity. The value is ", min_acceleration, ".");
  } else if (const auto speed = entity_status.action_status.twist.linear.x;  // [m/s]
             isinf(speed) or isnan(speed)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name), "'s speed value is NaN or infinity. The value is ", speed,
      ".");
  } else if (
    /*
       The desired acceleration is the acceleration at which the destination
       can be reached exactly at the specified time (= time remaining at zero).

       If no arrival time is specified for subsequent waypoints, there is no
       need to accelerate or decelerate, so the current acceleration will be
       the desired speed.
    */
    const auto desired_acceleration =
      [&]() {
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
    std::isinf(desired_acceleration) or std::isnan(desired_acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name),
      "'s desired acceleration value contains NaN or infinity. The value is ", desired_acceleration,
      ".");
  } else if (
    /*
       However, the desired acceleration is unrealistically large in terms of
       vehicle performance and dynamic constraints, so it is clamped to a
       realistic value.
    */
    const auto desired_speed =
      speed + std::clamp(desired_acceleration, min_acceleration, max_acceleration) * step_time;
    std::isinf(desired_speed) or std::isnan(desired_speed)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name), "'s desired speed value is NaN or infinity. The value is ",
      desired_speed, ".");
  } else if (const auto desired_velocity =
               [&]() {
                 /*
                    Note: The followingMode in OpenSCENARIO is passed as
                    variable dynamic_constraints_ignorable. the value of the
                    variable is `followingMode == position`.
                 */
                 if (polyline_trajectory->dynamic_constraints_ignorable) {
                   return normalize(target_position - position) * desired_speed;  // [m/s]
                 } else {
                   /*
                      Note: The vector returned if
                      dynamic_constraints_ignorable == true ignores parameters
                      such as the maximum rudder angle of the vehicle entry. In
                      this clause, such parameters must be respected and the
                      rotation angle difference of the z-axis center of the
                      vector must be kept below a certain value.
                   */
                   throw common::SimulationError(
                     "The followingMode is only supported for position.");
                 }
               }();
             any(is_infinity_or_nan, desired_velocity)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name),
      "'s desired velocity contains NaN or infinity. The value is [", desired_velocity.x, ", ",
      desired_velocity.y, ", ", desired_velocity.z, "].");
  } else {
    /*
       It's okay for this value to be infinite.
    */
    const auto remaining_time_to_arrival_to_front_waypoint =
      distance_to_front_waypoint / desired_speed;  // [s]

    if constexpr (false) {
      // clang-format off
      std::cout << std::fixed << std::boolalpha << std::string(80, '-') << std::endl;

      std::cout << "acceleration "
                << "== " << acceleration
                << std::endl;

      std::cout << "min_acceleration "
                << "== std::max(acceleration - max_deceleration_rate * step_time, -max_deceleration) "
                << "== std::max(" << acceleration << " - " << behavior_parameter.dynamic_constraints.max_deceleration_rate << " * " << step_time << ", " << -behavior_parameter.dynamic_constraints.max_deceleration << ") "
                << "== std::max(" << acceleration << " - " << behavior_parameter.dynamic_constraints.max_deceleration_rate * step_time << ", " << -behavior_parameter.dynamic_constraints.max_deceleration << ") "
                << "== std::max(" << (acceleration - behavior_parameter.dynamic_constraints.max_deceleration_rate * step_time) << ", " << -behavior_parameter.dynamic_constraints.max_deceleration << ") "
                << "== " << min_acceleration
                << std::endl;

      std::cout << "max_acceleration "
                << "== std::min(acceleration + max_acceleration_rate * step_time, +max_acceleration) "
                << "== std::min(" << acceleration << " + " << behavior_parameter.dynamic_constraints.max_acceleration_rate << " * " << step_time << ", " << behavior_parameter.dynamic_constraints.max_acceleration << ") "
                << "== std::min(" << acceleration << " + " << behavior_parameter.dynamic_constraints.max_acceleration_rate * step_time << ", " << behavior_parameter.dynamic_constraints.max_acceleration << ") "
                << "== std::min(" << (acceleration + behavior_parameter.dynamic_constraints.max_acceleration_rate * step_time) << ", " << behavior_parameter.dynamic_constraints.max_acceleration << ") "
                << "== " << max_acceleration
                << std::endl;

      std::cout << "min_acceleration < acceleration < max_acceleration "
                << "== " << min_acceleration << " < " << acceleration << " < " << max_acceleration << std::endl;

      std::cout << "desired_acceleration "
                << "== 2 * distance / std::pow(remaining_time, 2) - 2 * speed / remaining_time "
                << "== 2 * " << distance << " / " << std::pow(remaining_time, 2) << " - 2 * " << speed << " / " << remaining_time << " "
                << "== " << (2 * distance / std::pow(remaining_time, 2)) << " - " << (2 * speed / remaining_time) << " "
                << "== " << desired_acceleration << " "
                << "(acceleration < desired_acceleration == " << (acceleration < desired_acceleration) << " == need to " <<(acceleration < desired_acceleration ? "accelerate" : "decelerate") << ")"
                << std::endl;

      std::cout << "desired_speed "
                << "== speed + std::clamp(desired_acceleration, min_acceleration, max_acceleration) * step_time "
                << "== " << speed << " + std::clamp(" << desired_acceleration << ", " << min_acceleration << ", " << max_acceleration << ") * " << step_time << " "
                << "== " << speed << " + " << std::clamp(desired_acceleration, min_acceleration, max_acceleration) << " * " << step_time << " "
                << "== " << speed << " + " << std::clamp(desired_acceleration, min_acceleration, max_acceleration) * step_time << " "
                << "== " << desired_speed
                << std::endl;

      std::cout << "distance_to_front_waypoint "
                << "== " << distance_to_front_waypoint
                << std::endl;

      std::cout << "remaining_time_to_arrival_to_front_waypoint "
                << "== " << remaining_time_to_arrival_to_front_waypoint
                << std::endl;

      std::cout << "distance "
                << "== " << distance
                << std::endl;

      std::cout << "remaining_time "
                << "== " << remaining_time
                << std::endl;

      std::cout << "remaining_time_to_arrival_to_front_waypoint "
                << "("
                << "== distance_to_front_waypoint / desired_speed "
                << "== " << distance_to_front_waypoint << " / " << desired_speed << " "
                << "== " << remaining_time_to_arrival_to_front_waypoint
                << ")"
                << std::endl;

      std::cout << "arrive during this frame? "
                << "== remaining_time_to_arrival_to_front_waypoint < step_time "
                << "== " << remaining_time_to_arrival_to_front_waypoint << " < " << step_time << " "
                << "== " << isDefinitelyLessThan(remaining_time_to_arrival_to_front_waypoint, step_time)
                << std::endl;

      std::cout << "not too early? "
                << "== std::isnan(remaining_time_to_front_waypoint) or remaining_time_to_front_waypoint < remaining_time_to_arrival_to_front_waypoint + step_time "
                << "== std::isnan(" << remaining_time_to_front_waypoint << ") or " << remaining_time_to_front_waypoint << " < " << remaining_time_to_arrival_to_front_waypoint << " + " << step_time << " "
                << "== " << std::isnan(remaining_time_to_front_waypoint) << " or " << isDefinitelyLessThan(remaining_time_to_front_waypoint, remaining_time_to_arrival_to_front_waypoint + step_time) << " "
                << "== " << (std::isnan(remaining_time_to_front_waypoint) or isDefinitelyLessThan(remaining_time_to_front_waypoint, remaining_time_to_arrival_to_front_waypoint + step_time))
                << std::endl;
      // clang-format on
    }

    /*
       If the target point is reached during this step, it is considered
       reached.
    */
    if (isDefinitelyLessThan(remaining_time_to_arrival_to_front_waypoint, step_time)) {
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
        isDefinitelyLessThan(
          remaining_time_to_front_waypoint,
          remaining_time_to_arrival_to_front_waypoint + step_time)) {
        return discard_the_front_waypoint_and_recurse();
      } else {
        throw common::SimulationError(
          "Vehicle ", std::quoted(entity_status.name), " arrived at the waypoint in trajectory ",
          remaining_time_to_front_waypoint,
          " seconds earlier than the specified time. This may be due to unrealistic conditions of "
          "arrival time specification compared to vehicle parameters and dynamic constraints.");
      }
    }

    const auto current_velocity =
      quaternion_operation::convertQuaternionToEulerAngle(entity_status.pose.orientation) *
      entity_status.action_status.twist.linear.x;

    /*
       Note: If obstacle avoidance is to be implemented, the steering behavior
       known by the name "collision avoidance" should be synthesized here into
       steering.
    */
    const auto steering = desired_velocity - current_velocity;

    const auto velocity = truncate(current_velocity + steering, desired_speed);

    auto updated_status = entity_status;

    updated_status.pose.position += velocity * step_time;

    updated_status.pose.orientation = [&]() {
      geometry_msgs::msg::Vector3 direction;
      direction.x = 0;
      direction.y = 0;
      direction.z = std::atan2(velocity.y, velocity.x);
      return quaternion_operation::convertEulerAngleToQuaternion(direction);
    }();

    updated_status.action_status.twist.linear.x = norm(velocity);

    updated_status.action_status.twist.linear.y = 0;

    updated_status.action_status.twist.linear.z = 0;

    updated_status.action_status.twist.angular =
      quaternion_operation::convertQuaternionToEulerAngle(quaternion_operation::getRotation(
        entity_status.pose.orientation, updated_status.pose.orientation)) /
      step_time;

    updated_status.action_status.accel.linear =
      (updated_status.action_status.twist.linear - entity_status.action_status.twist.linear) /
      step_time;

    updated_status.action_status.accel.angular =
      (updated_status.action_status.twist.angular - entity_status.action_status.twist.angular) /
      step_time;

    updated_status.time = entity_status.time + step_time;

    updated_status.lanelet_pose_valid = false;

    return updated_status;
  }
}
}  // namespace follow_trajectory
}  // namespace traffic_simulator
