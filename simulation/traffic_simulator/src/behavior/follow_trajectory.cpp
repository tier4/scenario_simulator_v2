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

#include <arithmetic/floating_point/comparison.hpp>
#include <geometry/quaternion/direction_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/inner_product.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/normalize.hpp>
#include <geometry/vector3/operator.hpp>
#include <geometry/vector3/truncate.hpp>
#include <iostream>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/behavior/follow_trajectory.hpp>
#include <traffic_simulator/behavior/follow_waypoint_controller.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator/utils/pose.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{
constexpr auto verbose_precision = 32;
constexpr auto verbose_status_changes = true;
constexpr auto verbose_discard_reason = true;

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
  traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter, const double step_time,
  const double matching_distance, std::optional<double> target_speed) -> std::optional<EntityStatus>
{
  std::cout << std::fixed << std::setprecision(verbose_precision);

  using math::arithmetic::isApproximatelyEqualTo;
  using math::arithmetic::isDefinitelyLessThan;

  using math::geometry::operator+;
  using math::geometry::operator-;
  using math::geometry::operator*;
  using math::geometry::operator/;
  using math::geometry::operator+=;

  using math::geometry::CatmullRomSpline;
  using math::geometry::convertDirectionToQuaternion;
  using math::geometry::hypot;
  using math::geometry::innerProduct;
  using math::geometry::norm;
  using math::geometry::normalize;
  using math::geometry::truncate;

  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{false};
  constexpr bool allow_lane_change{true};

  const auto include_crosswalk = [](const auto & entity_type) {
    return (traffic_simulator_msgs::msg::EntityType::PEDESTRIAN == entity_type.type) ||
           (traffic_simulator_msgs::msg::EntityType::MISC_OBJECT == entity_type.type);
  }(entity_status.type);

  auto distance_along_lanelet =
    [&](const geometry_msgs::msg::Point & from, const geometry_msgs::msg::Point & to) -> double {
    using geometry_msgs::msg::Pose;
    using geometry_msgs::msg::Vector3;

    const RoutingConfiguration routing_configuration{allow_lane_change};

    const auto quaternion = convertDirectionToQuaternion(
      geometry_msgs::build<Vector3>().x(to.x - from.x).y(to.y - from.y).z(to.z - from.z));
    const auto from_pose = geometry_msgs::build<Pose>().position(from).orientation(quaternion);
    if (
      const auto from_canonicalized_lanelet_pose = pose::toCanonicalizedLaneletPose(
        from_pose, entity_status.bounding_box, include_crosswalk, matching_distance)) {
      const auto to_pose = geometry_msgs::build<Pose>().position(to).orientation(quaternion);
      if (
        const auto to_canonicalized_lanelet_pose = pose::toCanonicalizedLaneletPose(
          to_pose, entity_status.bounding_box, include_crosswalk, matching_distance)) {
        if (const auto longitudinal_distance = distance::longitudinalDistance(
              from_canonicalized_lanelet_pose.value(), to_canonicalized_lanelet_pose.value(),
              include_adjacent_lanelet, include_opposite_direction, routing_configuration);
            longitudinal_distance.has_value()
            /**
             * DIRTY HACK!
             * Negative longitudinal distance (calculated along lanelet in opposite direction)
             * causes some scenarios to fail because of an unrelated issue with lanelet matching.
             * The issue is caused by wrongly matched lanelet poses and thus wrong distances.
             * When lanelet matching errors are fixed, this dirty hack can be removed.
             */
            and longitudinal_distance.value() >= 0.0) {
          if (
            const auto lateral_distance = distance::lateralDistance(
              from_canonicalized_lanelet_pose.value(), to_canonicalized_lanelet_pose.value(),
              routing_configuration)) {
            return std::hypot(longitudinal_distance.value(), lateral_distance.value());
          }
        }
      }
    }
    return hypot(from, to);
  };

  auto discard_the_front_waypoint_and_recurse = [&]() {
    /*
       The OpenSCENARIO standard does not define the behavior when the value of
       Timing.domainAbsoluteRelative is "relative". The standard only states
       "Definition of time value context as either absolute or relative", and
       it is completely unclear when the relative time starts.

       This implementation has interpreted the specification as follows:
       Relative time starts from the start of FollowTrajectoryAction or from
       the time of reaching the previous "waypoint with arrival time".

       Note: not std::isnan(polyline_trajectory.base_time) means
       "Timing.domainAbsoluteRelative is relative".

       Note: not std::isnan(polyline_trajectory.shape.vertices.front().time)
       means "The waypoint about to be popped is the waypoint with the
       specified arrival time".
    */
    if (
      not std::isnan(polyline_trajectory.base_time) and
      not std::isnan(polyline_trajectory.shape.vertices.front().time)) {
      polyline_trajectory.base_time = entity_status.time;
    }
    if (verbose_discard_reason) {
      std::cout << "DISCARD WAYPOINT NR " << (polyline_trajectory.shape.vertices.size() - 1)
                << ": x=" << polyline_trajectory.shape.vertices.front().position.position.x
                << ", y=" << polyline_trajectory.shape.vertices.front().position.position.y
                << ", z=" << polyline_trajectory.shape.vertices.front().position.position.z
                << ", waypoint_time=" << polyline_trajectory.shape.vertices.front().time
                << ", current_time=" << entity_status.time << std::endl;
    }

    if (std::rotate(
          std::begin(polyline_trajectory.shape.vertices),
          std::begin(polyline_trajectory.shape.vertices) + 1,
          std::end(polyline_trajectory.shape.vertices));
        not polyline_trajectory.closed) {
      polyline_trajectory.shape.vertices.pop_back();
    }
    return makeUpdatedStatus(
      entity_status, polyline_trajectory, behavior_parameter, step_time, matching_distance,
      target_speed);
  };

  auto is_infinity_or_nan = [](auto x) constexpr { return std::isinf(x) or std::isnan(x); };

  auto first_waypoint_with_arrival_time_specified = [&]() {
    return std::find_if(
      polyline_trajectory.shape.vertices.begin(), polyline_trajectory.shape.vertices.end(),
      [](auto && vertex) { return not std::isnan(vertex.time); });
  };

  auto is_breaking_waypoint = [&]() {
    return first_waypoint_with_arrival_time_specified() >=
           std::prev(polyline_trajectory.shape.vertices.end());
  };

  auto update_entity_status = [step_time, include_crosswalk](
                                const traffic_simulator_msgs::msg::EntityStatus & entity_status,
                                const geometry_msgs::msg::Vector3 & desired_velocity, bool verbose)
    -> traffic_simulator_msgs::msg::EntityStatus {
    const auto current_velocity = [&]() {
      const auto pitch =
        -math::geometry::convertQuaternionToEulerAngle(entity_status.pose.orientation).y;
      const auto yaw =
        math::geometry::convertQuaternionToEulerAngle(entity_status.pose.orientation).z;
      return geometry_msgs::build<geometry_msgs::msg::Vector3>()
        .x(std::cos(pitch) * std::cos(yaw) * entity_status.action_status.twist.linear.x)
        .y(std::cos(pitch) * std::sin(yaw) * entity_status.action_status.twist.linear.x)
        .z(std::sin(pitch) * entity_status.action_status.twist.linear.x);
    }();

    auto updated_status = entity_status;
    updated_status.pose.position += (current_velocity + desired_velocity) * 0.5 * step_time;

    updated_status.pose.orientation = [&]() {
      if (norm(desired_velocity) > std::numeric_limits<double>::epsilon()) {
        /// @note if there is a designed_velocity vector with sufficient magnitude, set the orientation in the direction of it
        return math::geometry::convertDirectionToQuaternion(desired_velocity);
      } else {
        /// @note do not change orientation if desired_velocity vector is too small to determine reliable direction
        return entity_status.pose.orientation;
      }
    }();

    /// @note if it is the transition between lanelets: overwrite position to improve precision
    if (entity_status.lanelet_pose_valid) {
      constexpr bool desired_velocity_is_global{true};
      const auto canonicalized_lanelet_pose =
        traffic_simulator::pose::toCanonicalizedLaneletPose(entity_status.lanelet_pose);
      const auto estimated_next_canonicalized_lanelet_pose =
        traffic_simulator::pose::toCanonicalizedLaneletPose(updated_status.pose, include_crosswalk);
      if (canonicalized_lanelet_pose && estimated_next_canonicalized_lanelet_pose) {
        const auto next_lanelet_id =
          static_cast<LaneletPose>(estimated_next_canonicalized_lanelet_pose.value()).lanelet_id;
        /// @note Handle lanelet transition
        if (
          const auto updated_position = pose::updatePositionForLaneletTransition(
            canonicalized_lanelet_pose.value(), next_lanelet_id, desired_velocity,
            desired_velocity_is_global, step_time)) {
          updated_status.pose.position = updated_position.value();
        }
      }
    }

    updated_status.action_status.twist.linear.x = norm(desired_velocity);
    updated_status.action_status.twist.linear.y = 0;
    updated_status.action_status.twist.linear.z = 0;
    updated_status.action_status.twist.angular =
      math::geometry::convertQuaternionToEulerAngle(math::geometry::getRotation(
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

    if (verbose) {
      std::cout << "=== AFTER UPDATE ===" << std::endl;
      const auto this_step_distance = (current_velocity + desired_velocity) * 0.5 * step_time;
      const auto no_transition_position = entity_status.pose.position + this_step_distance;
      std::cout << "This step distance: " << norm(this_step_distance) << " x=" << this_step_distance.x << ", y=" << this_step_distance.y << ", z=" << this_step_distance.z << std::endl;
      std::cout << "Future Position (using desired_velocity  ): x=" << no_transition_position.x << ", y=" << no_transition_position.y << ", z=" << no_transition_position.z << std::endl;
      std::cout << "Future Position (after lanelet transition): x=" << updated_status.pose.position.x << ", y=" << updated_status.pose.position.y << ", z=" << updated_status.pose.position.z << std::endl;
      std::cout << "Future Orientation: x=" << updated_status.pose.orientation.x << ", y=" << updated_status.pose.orientation.y << ", z=" << updated_status.pose.orientation.z << ", w=" << updated_status.pose.orientation.w << " (yaw=" << math::geometry::convertQuaternionToEulerAngle(updated_status.pose.orientation).z << ")" << std::endl;
      std::cout << "Future Linear velocity: x=" << updated_status.action_status.twist.linear.x << ", y=" << updated_status.action_status.twist.linear.y << ", z=" << updated_status.action_status.twist.linear.z << std::endl;
      std::cout << "Future Angular velocity: x=" << updated_status.action_status.twist.angular.x << ", y=" << updated_status.action_status.twist.angular.y << ", z=" << updated_status.action_status.twist.angular.z << std::endl;
      std::cout << "Future Linear acceleration: x=" << updated_status.action_status.accel.linear.x << ", y=" << updated_status.action_status.accel.linear.y << ", z=" << updated_status.action_status.accel.linear.z << std::endl;
      std::cout << "Future Angular acceleration: x=" << updated_status.action_status.accel.angular.x << ", y=" << updated_status.action_status.accel.angular.y << ", z=" << updated_status.action_status.accel.angular.z << std::endl;
    }
    return updated_status;
  };

  /*
     The following code implements the steering behavior known as "seek". See
     "Steering Behaviors For Autonomous Characters" by Craig Reynolds for more
     information.

     See https://www.researchgate.net/publication/2495826_Steering_Behaviors_For_Autonomous_Characters
  */


  if(verbose_status_changes) {
    const auto& position = entity_status.pose.position;
    const auto& orientation = entity_status.pose.orientation;
    const auto& linear_velocity = entity_status.action_status.twist.linear;
    const auto& angular_velocity = entity_status.action_status.twist.angular;
    const auto& linear_acceleration = entity_status.action_status.accel.linear;
    const auto& angular_acceleration = entity_status.action_status.accel.angular;
    std::cout << std::endl << "===============================" << std::endl<< std::endl << std::endl;
    std::cout << "=== BEFORE UPDATE ===" << std::endl;
    std::cout << "Current Time: " << entity_status.time << std::endl;
    std::cout << "Current Position: x=" << position.x << ", y=" << position.y << ", z=" << position.z << std::endl;
    std::cout << "Current Orientation: x=" << orientation.x << ", y=" << orientation.y << ", z=" << orientation.z << ", w=" << orientation.w << " (yaw=" << math::geometry::convertQuaternionToEulerAngle(orientation).z << ")" << std::endl;
    std::cout << "Current Linear velocity: x=" << linear_velocity.x << ", y=" << linear_velocity.y << ", z=" << linear_velocity.z << std::endl;
    std::cout << "Current Angular velocity: x=" << angular_velocity.x << ", y=" << angular_velocity.y << ", z=" << angular_velocity.z << std::endl;
    std::cout << "Current Linear acceleration: x=" << linear_acceleration.x << ", y=" << linear_acceleration.y << ", z=" << linear_acceleration.z << std::endl;
    std::cout << "Current Angular acceleration: x=" << angular_acceleration.x << ", y=" << angular_acceleration.y << ", z=" << angular_acceleration.z << std::endl;
    if (!polyline_trajectory.shape.vertices.empty()) {
      const auto& nearest_waypoint = polyline_trajectory.shape.vertices.front().position.position;
      auto first_waypoint_with_arrival_time_specified = std::find_if(
        polyline_trajectory.shape.vertices.begin(), polyline_trajectory.shape.vertices.end(),
        [](auto && vertex) { return not std::isnan(vertex.time); });

      const auto remaining_time_to_timed_waypoint =
        (first_waypoint_with_arrival_time_specified != std::end(polyline_trajectory.shape.vertices))
        ? ((not std::isnan(polyline_trajectory.base_time) ? polyline_trajectory.base_time : 0.0) +
           first_waypoint_with_arrival_time_specified->time - entity_status.time)
        : std::numeric_limits<double>::infinity();

      std::cout << "Waypoints count: " << polyline_trajectory.shape.vertices.size() << std::endl;
      std::cout << "Nearest waypoint: x=" << nearest_waypoint.x << ", y=" << nearest_waypoint.y << ", z=" << nearest_waypoint.z << std::endl;
      std::cout << "Euclidean distance to nearest waypoint: " << hypot(entity_status.pose.position, nearest_waypoint) << std::endl;
      std::cout << "Remaining time: ";
      if (std::isinf(remaining_time_to_timed_waypoint)) {
        std::cout << "infinity (no waypoint with specified time)";
      } else {
        std::cout << remaining_time_to_timed_waypoint << " s";
      }
      std::cout << std::endl;
    } else {
      std::cout << "No waypoints available" << std::endl;
    }
  }

  if (polyline_trajectory.shape.vertices.empty() and std::abs(entity_status.action_status.twist.linear.x) < FollowWaypointController::local_epsilon && std::abs(entity_status.action_status.accel.linear.x) < FollowWaypointController::local_epsilon) {
    return std::nullopt;
  } else if (polyline_trajectory.shape.vertices.empty()) {
    const auto follow_waypoint_controller = FollowWaypointController(behavior_parameter, step_time, true, 0.0);
    const auto desired_acceleration = std::max(
      follow_waypoint_controller.accelerationWithJerkConstraint(
        entity_status.action_status.twist.linear.x, 0.0,
        behavior_parameter.dynamic_constraints.max_deceleration_rate),
      -behavior_parameter.dynamic_constraints.max_deceleration);
    const auto desired_speed = entity_status.action_status.twist.linear.x + desired_acceleration * step_time;
    const auto desired_velocity = [&]() {
      const auto pitch =
        -math::geometry::convertQuaternionToEulerAngle(entity_status.pose.orientation).y;
      const auto yaw =
        math::geometry::convertQuaternionToEulerAngle(entity_status.pose.orientation).z;
      return geometry_msgs::build<geometry_msgs::msg::Vector3>()
        .x(std::cos(pitch) * std::cos(yaw) * desired_speed)
        .y(std::cos(pitch) * std::sin(yaw) * desired_speed)
        .z(std::sin(pitch) * desired_speed);
    }();
    
    if (verbose_status_changes) {
      std::cout << "=== FIX AFTER OVERSHOOT ===" << std::endl;
    }

    return update_entity_status(entity_status, desired_velocity, verbose_status_changes);
  } else if (const auto position = entity_status.pose.position; any(is_infinity_or_nan, position)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name), " coordinate value contains NaN or infinity. The value is [",
      position.x, ", ", position.y, ", ", position.z, "].");
  } else if (
    /*
       We've made sure that polyline_trajectory.shape.vertices is not empty, so
       a reference to vertices.front() always succeeds. vertices.front() is
       referenced only this once in this member function.
    */
    const auto target_position = polyline_trajectory.shape.vertices.front().position.position;
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
      distance_along_lanelet(position, target_position),
      (not std::isnan(polyline_trajectory.base_time) ? polyline_trajectory.base_time : 0.0) +
        polyline_trajectory.shape.vertices.front().time - entity_status.time);
    /*
       This clause is to avoid division-by-zero errors in later clauses with
       distance_to_front_waypoint as the denominator if the distance
       miraculously becomes zero.
    */
    isDefinitelyLessThan(distance_to_front_waypoint, std::numeric_limits<double>::epsilon())) {
    if (verbose_discard_reason) {
      std::cout << "DISCARD WAYPOINT NR " << (polyline_trajectory.shape.vertices.size() - 1)
                << " REASON: distance_to_front_waypoint is too small (< epsilon = "
                << std::numeric_limits<double>::epsilon() << ")" << std::endl;
    }
    return discard_the_front_waypoint_and_recurse();
  } else if (
    const auto [distance, remaining_time] =
      [&]() {
        /*
           Note for anyone working on adding support for followingMode follow
           to this function (FollowPolylineTrajectoryAction::tick) in the
           future: if followingMode is follow, this distance calculation may be
           inappropriate.
        */
        auto total_distance_to = [&](auto last) {
          auto total_distance = 0.0;
          for (auto iter = std::begin(polyline_trajectory.shape.vertices);
               0 < std::distance(iter, last); ++iter) {
            total_distance +=
              distance_along_lanelet(iter->position.position, std::next(iter)->position.position);
          }
          return total_distance;
        };

        if (
          first_waypoint_with_arrival_time_specified() !=
          std::end(polyline_trajectory.shape.vertices)) {
          if (const auto remaining_time =
                (not std::isnan(polyline_trajectory.base_time) ? polyline_trajectory.base_time
                                                               : 0.0) +
                first_waypoint_with_arrival_time_specified()->time - entity_status.time;
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
              first_waypoint_with_arrival_time_specified()->time, " (in ",
              (not std::isnan(polyline_trajectory.base_time) ? "absolute" : "relative"),
              " simulation time). This may be due to unrealistic conditions of arrival time "
              "specification compared to vehicle parameters and dynamic constraints.");
          } else {
            return std::make_tuple(
              distance_to_front_waypoint +
                total_distance_to(first_waypoint_with_arrival_time_specified()),
              remaining_time != 0 ? remaining_time : std::numeric_limits<double>::epsilon());
          }
        } else {
          return std::make_tuple(
            distance_to_front_waypoint +
              total_distance_to(std::end(polyline_trajectory.shape.vertices) - 1),
            std::numeric_limits<double>::infinity());
        }
      }();
    isDefinitelyLessThan(distance, std::numeric_limits<double>::epsilon())) {
    if (verbose_discard_reason) {
      std::cout << "DISCARD WAYPOINT NR " << (polyline_trajectory.shape.vertices.size() - 1)
                << " REASON: total distance is too small (< epsilon = "
                << std::numeric_limits<double>::epsilon() << ")" << std::endl;
    }
    return discard_the_front_waypoint_and_recurse();
  } else if (const auto acceleration = entity_status.action_status.accel.linear.x;  // [m/s^2]
             std::isinf(acceleration) or std::isnan(acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name), "'s acceleration value is NaN or infinity. The value is ",
      acceleration, ". ");
  } else if (const auto max_acceleration = std::min(
               acceleration /* [m/s^2] */ +
                 behavior_parameter.dynamic_constraints.max_acceleration_rate /* [m/s^3] */ *
                   step_time /* [s] */,
               +behavior_parameter.dynamic_constraints.max_acceleration /* [m/s^2] */);
             std::isinf(max_acceleration) or std::isnan(max_acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name),
      "'s maximum acceleration value is NaN or infinity. The value is ", max_acceleration, ". ");
  } else if (const auto min_acceleration = std::max(
               acceleration /* [m/s^2] */ -
                 behavior_parameter.dynamic_constraints.max_deceleration_rate /* [m/s^3] */ *
                   step_time /* [s] */,
               -behavior_parameter.dynamic_constraints.max_deceleration /* [m/s^2] */);
             std::isinf(min_acceleration) or std::isnan(min_acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name),
      "'s minimum acceleration value is NaN or infinity. The value is ", min_acceleration, ". ");
  } else if (const auto speed = entity_status.action_status.twist.linear.x;  // [m/s]
             std::isinf(speed) or std::isnan(speed)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name), "'s speed value is NaN or infinity. The value is ", speed,
      ". ");
  } else if (
    /*
       The controller provides the ability to calculate acceleration using constraints from the
       behavior_parameter. The value is_breaking_waypoint() determines whether the calculated
       acceleration takes braking into account - it is true if the nearest waypoint with the
       specified time is the last waypoint or the nearest waypoint without the specified time is the
       last waypoint.

       If an arrival time was specified for any of the remaining waypoints, priority is given to
       meeting the arrival time, and the vehicle is driven at a speed at which the arrival time can
       be met.

       However, the controller allows passing target_speed as a speed which is followed by the
       controller. target_speed is passed only if no arrival time was specified for any of the
       remaining waypoints. If despite no arrival time in the remaining waypoints, target_speed is
       not set (it is std::nullopt), target_speed is assumed to be the same as max_speed from the
       behaviour_parameter.
    */
    const auto follow_waypoint_controller = FollowWaypointController(
      behavior_parameter, step_time, is_breaking_waypoint(),
      std::isinf(remaining_time) ? target_speed : std::nullopt);
    false) {
  } else if (
    /*
       The desired acceleration is the acceleration at which the destination
       can be reached exactly at the specified time (= time remaining at zero).

       The desired acceleration is calculated to the nearest waypoint with a specified arrival time.
       It is calculated in such a way as to reach a constant linear speed as quickly as possible,
       ensuring arrival at a waypoint at the precise time and with the shortest possible distance.
       More precisely, the controller selects acceleration to minimize the distance to the waypoint
       that will be reached in a time step defined as the expected arrival time.
       In addition, the controller ensures a smooth stop at the last waypoint of the trajectory,
       with linear speed equal to zero and acceleration equal to zero.
    */
    const auto desired_acceleration = [&]() -> double {
      try {
        return follow_waypoint_controller.getAcceleration(
          remaining_time, distance, entity_status, update_entity_status, distance_along_lanelet);
      } catch (const ControllerError & e) {
        throw common::Error(
          "Vehicle ", std::quoted(entity_status.name),
          " - controller operation problem encountered. ",
          follow_waypoint_controller.getFollowedWaypointDetails(polyline_trajectory), e.what());
      }
    }();
    std::isinf(desired_acceleration) or std::isnan(desired_acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name),
      "'s desired acceleration value contains NaN or infinity. The value is ", desired_acceleration,
      ". ");
  } else if (const auto desired_speed = speed + desired_acceleration * step_time;
             std::isinf(desired_speed) or std::isnan(desired_speed)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name), "'s desired speed value is NaN or infinity. The value is ",
      desired_speed, ". ");
  } else if (const auto desired_velocity =
               [&]() {
                 /*
                    Note: The followingMode in OpenSCENARIO is passed as
                    variable dynamic_constraints_ignorable. the value of the
                    variable is `followingMode == position`.
                 */
                 if (polyline_trajectory.dynamic_constraints_ignorable) {
                   const auto dx = target_position.x - position.x;
                   const auto dy = target_position.y - position.y;
                   /// @note if entity is on lane use pitch from lanelet, otherwise use pitch on target
                   const auto pitch =
                     entity_status.lanelet_pose_valid
                       ? -math::geometry::convertQuaternionToEulerAngle(
                            entity_status.pose.orientation)
                            .y
                       : std::atan2(target_position.z - position.z, std::hypot(dy, dx));
                   const auto yaw = std::atan2(dy, dx);  // Use yaw on target
                   return geometry_msgs::build<geometry_msgs::msg::Vector3>()
                     .x(std::cos(pitch) * std::cos(yaw) * desired_speed)
                     .y(std::cos(pitch) * std::sin(yaw) * desired_speed)
                     .z(std::sin(pitch) * desired_speed);
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
  } else if (const auto current_velocity =
               [&]() {
                 const auto pitch =
                   -math::geometry::convertQuaternionToEulerAngle(entity_status.pose.orientation).y;
                 const auto yaw =
                   math::geometry::convertQuaternionToEulerAngle(entity_status.pose.orientation).z;
                 return geometry_msgs::build<geometry_msgs::msg::Vector3>()
                   .x(std::cos(pitch) * std::cos(yaw) * speed)
                   .y(std::cos(pitch) * std::sin(yaw) * speed)
                   .z(std::sin(pitch) * speed);
               }();
             distance_to_front_waypoint < FollowWaypointController::finish_distance_tolerance &&
             ((speed <= 0.0 && desired_acceleration < -std::numeric_limits<double>::epsilon()) || (norm(desired_velocity) > std::numeric_limits<double>::epsilon() && innerProduct(desired_velocity, current_velocity) < 0.0))) {
    if (verbose_discard_reason) {
      std::cout
        << "DISCARD WAYPOINT NR " << (polyline_trajectory.shape.vertices.size() - 1)
        << " REASON: algorithm attempts to set velocity in the opposite direction, overshoot "
           "occured and the final waypoint is behind the vehicle. However overshoot distance equal to "
        << distance_to_front_waypoint << " is acceptable: ";
      std::cout << "distance in this step = "
                << (speed + desired_acceleration * step_time * 0.5) * step_time
                << ",  distance_to_front_waypoint = " << distance_to_front_waypoint << " < "
                << FollowWaypointController::finish_distance_tolerance;
      std::cout << ",  innerProduct(desired_velocity, current_velocity) = "
                << innerProduct(desired_velocity, current_velocity) << " < 0" << std::endl;
      std::cout<< "current_velocity: "<< current_velocity.x << ", "<<current_velocity.y<<", "<<current_velocity.z<<std::endl;
      std::cout<< "desired_velocity: "<< desired_velocity.x << ", "<<desired_velocity.y<<", "<<desired_velocity.z<<std::endl;
    }
    return discard_the_front_waypoint_and_recurse();
  } else {
    if (std::isnan(remaining_time_to_front_waypoint)) {
      /// @note If the nearest waypoint is arrived at in this step without a specific arrival time, it will
      /// be considered as achieved
      const auto this_step_distance = norm(current_velocity + desired_velocity) * 0.5 * step_time;
      if (std::isinf(remaining_time) && polyline_trajectory.shape.vertices.size() == 1) {
        /// @note If the trajectory has only waypoints with unspecified time, the last one is followed using
        /// maximum speed including braking - in this case accuracy of arrival is checked
        if (
          this_step_distance > distance_to_front_waypoint + FollowWaypointController::finish_distance_tolerance) {
          throw common::Error(
            "Too much overshoot of the last waypoint detected for vehicle ",
            std::quoted(entity_status.name),
            ". Overshoot distance: ", this_step_distance - distance_to_front_waypoint,
            " m (this step distance: ", this_step_distance,
            " m, remaining distance: ", distance_to_front_waypoint,
            " m, overshoot tolerance: ", FollowWaypointController::finish_distance_tolerance,
            "). This error indicates a bug in the algorithm - report the details to the "
            "developer.");
        } else if (follow_waypoint_controller.areConditionsOfArrivalMet(
                     acceleration, speed, distance_to_front_waypoint)) {
          if (verbose_discard_reason) {
            std::cout
              << "DISCARD WAYPOINT NR " << (polyline_trajectory.shape.vertices.size() - 1)
              << " REASON: conditions of arrival met for last waypoint with unspecified time"
              << std::endl;
          }
          return discard_the_front_waypoint_and_recurse();
        }
      } else {
        /// @note If it is an intermediate waypoint with an unspecified time, the accuracy of the arrival is
        /// irrelevant and even a large overshoot is acceptable
        if (this_step_distance > distance_to_front_waypoint) {
          if (verbose_discard_reason) {
            std::cout << "DISCARD WAYPOINT NR " << (polyline_trajectory.shape.vertices.size() - 1)
                      << " REASON: intermediate waypoint - this step would overshoot waypoint"
                      << std::endl;
            std::cout << "  this_step_distance = " << this_step_distance
                      << " > distance_to_front_waypoint = " << distance_to_front_waypoint
                      << std::endl;
          }
          return discard_the_front_waypoint_and_recurse();
        }
      }
      /*
        If there is insufficient time left for the next calculation step.
        The value of step_time/2 is compared, as the remaining time is affected by floating point
        inaccuracy, sometimes it reaches values of 1e-7 (almost zero, but not zero) or (step_time -
        1e-7) (almost step_time). Because the step is fixed, it should be assumed that the value
        here is either equal to 0 or step_time. Value step_time/2 allows to return true if no next
        step is possible (remaining_time_to_front_waypoint is almost zero).
      */
    } else if (isDefinitelyLessThan(remaining_time_to_front_waypoint, step_time / 2.0)) {
      if (follow_waypoint_controller.areConditionsOfArrivalMet(
            acceleration, speed, distance_to_front_waypoint)) {
        if (verbose_discard_reason) {
          std::cout << "DISCARD WAYPOINT NR " << (polyline_trajectory.shape.vertices.size() - 1)
                    << " REASON: insufficient remaining time but arrival conditions met"
                    << std::endl;
          std::cout << "  remaining_time_to_front_waypoint = " << remaining_time_to_front_waypoint
                    << " < step_time/2 = " << (step_time / 2.0) << std::endl;
        }
        return discard_the_front_waypoint_and_recurse();
      } else {
        throw common::SimulationError(
          "Vehicle ", std::quoted(entity_status.name), " at time ", entity_status.time,
          "s (remaining time is ", remaining_time_to_front_waypoint,
          "s), has completed a trajectory to the nearest waypoint with",
          " specified time equal to ", polyline_trajectory.shape.vertices.front().time,
          "s at a distance equal to ", distance,
          " from that waypoint which is greater than the accepted accuracy.");
      }
    }

    /*
       Note: If obstacle avoidance is to be implemented, the steering behavior
       known by the name "collision avoidance" should be synthesized here into
       steering.
    */
    return update_entity_status(entity_status, desired_velocity, verbose_status_changes);
  }
}
}  // namespace follow_trajectory
}  // namespace traffic_simulator
