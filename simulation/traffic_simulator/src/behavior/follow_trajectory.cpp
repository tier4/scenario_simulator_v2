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
#include <geometry/vector3/scalar_to_direction_vector.hpp>
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
  using math::geometry::scalarToDirectionVector;
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

  const auto update_entity_status =
    [step_time, include_crosswalk](
      const traffic_simulator_msgs::msg::EntityStatus & entity_status,
      const geometry_msgs::msg::Vector3 & desired_velocity)
    -> traffic_simulator_msgs::msg::EntityStatus {
    auto updated_status = entity_status;

    const auto current_velocity = scalarToDirectionVector(
      entity_status.action_status.twist.linear.x, entity_status.pose.orientation);

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
        /// @note handle lanelet transition
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
    return updated_status;
  };

  const auto is_immobile = [&entity_status]() {
    return std::abs(entity_status.action_status.twist.linear.x) <
             FollowWaypointController::local_epsilon &&
           std::abs(entity_status.action_status.accel.linear.x) <
             FollowWaypointController::local_epsilon;
  };

  const auto constrained_brake_velocity = [&behavior_parameter, step_time](
                                            const double speed, const auto & orientation) {
    const auto controller = FollowWaypointController(behavior_parameter, step_time, true, 0.0);
    const auto deceleration = std::max(
      controller.accelerationWithJerkConstraint(
        speed, 0.0, behavior_parameter.dynamic_constraints.max_deceleration_rate),
      -behavior_parameter.dynamic_constraints.max_deceleration);
    return scalarToDirectionVector(speed + deceleration * step_time, orientation);
  };

  const auto is_desired_velocity_opposite =
    [](const auto & desired_velocity, const auto & current_velocity) {
      /*
       Check if desired velocity has opposite direction to current velocity.
       Desired velocity must have non-zero magnitude (norm > epsilon) to have a valid direction.
       If current_velocity is zero (0,0,0), inner product is 0.0 which is not < 0.0, so returns false.
       Only when both velocities are non-zero and point in opposite directions, inner product < 0.0.
    */
      return norm(desired_velocity) > std::numeric_limits<double>::epsilon() &&
             innerProduct(desired_velocity, current_velocity) < 0.0;
    };

  const auto is_waypoint_overshot_and_can_be_discarded =
    [&is_desired_velocity_opposite, &entity_status](
      const double distance_to_front_waypoint, const auto & desired_velocity,
      const double desired_acceleration) -> bool {
    /*
       Waypoint can be considered overshot only if within acceptable_overshoot_distance.
       Waypoint is overshot and should be discarded if:
       (1) moving backward (speed < 0) or stopped (speed == 0) with negative acceleration (will move backward)
       (2) desired velocity direction is reversed relative to current velocity (passed the waypoint)
    */
    const auto speed = entity_status.action_status.twist.linear.x;
    const auto current_velocity = scalarToDirectionVector(speed, entity_status.pose.orientation);
    return distance_to_front_waypoint <= FollowWaypointController::acceptable_overshoot_distance &&
           ((speed <= 0.0 && desired_acceleration < -std::numeric_limits<double>::epsilon()) ||
            is_desired_velocity_opposite(desired_velocity, current_velocity));
  };

  const auto is_waypoint_overshot_and_cannot_be_discarded =
    [&is_desired_velocity_opposite, &entity_status](
      const double distance_to_front_waypoint, const auto & desired_velocity,
      const double desired_acceleration) -> bool {
    /*
       Check if waypoint was already overshot beyond acceptable_overshoot_distance.
       This should NEVER be true - waypoint should have been discarded by is_waypoint_overshot_and_can_be_discarded()
       or step execution should have been prevented earlier.

       Returns true if:
       - Waypoint is overshot (desired_velocity is opposite to current_velocity, or moving backward with negative acceleration)
       - Distance is beyond acceptable tolerance (distance_to_front_waypoint > acceptable_overshoot_distance)

       This indicates inconsistent distance calculations - distance_along_lanelet likely returned
       different values across simulation steps due to lanelet projection issues.
    */
    const auto speed = entity_status.action_status.twist.linear.x;
    const auto current_velocity = scalarToDirectionVector(speed, entity_status.pose.orientation);
    return distance_to_front_waypoint > FollowWaypointController::acceptable_overshoot_distance &&
           ((speed <= 0.0 && desired_acceleration < -std::numeric_limits<double>::epsilon()) ||
            is_desired_velocity_opposite(desired_velocity, current_velocity));
  };

  /*
     The following code implements the steering behavior known as "seek". See
     "Steering Behaviors For Autonomous Characters" by Craig Reynolds for more
     information.

     See https://www.researchgate.net/publication/2495826_Steering_Behaviors_For_Autonomous_Characters
  */
  if (polyline_trajectory.shape.vertices.empty() and is_immobile()) {
    return std::nullopt;
  } else if (polyline_trajectory.shape.vertices.empty()) {
    /// @note if no waypoints and vehicle is still moving, apply constrained deceleration to bring it to immobile state
    return update_entity_status(
      entity_status, constrained_brake_velocity(
                       entity_status.action_status.twist.linear.x, entity_status.pose.orientation));
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
  } else if (is_waypoint_overshot_and_can_be_discarded(
               distance_to_front_waypoint, desired_velocity, desired_acceleration)) {
    /*
       Waypoint is overshot (within acceptable_overshoot_distance), desired_velocity now points in opposite direction to current velocity.
       Following this desired_velocity would cause unexpected 180-degree turn, so discard the waypoint.
    */
    return discard_the_front_waypoint_and_recurse();
  } else if (is_waypoint_overshot_and_cannot_be_discarded(
               distance_to_front_waypoint, desired_velocity, desired_acceleration)) {
    /*
       CRITICAL: Waypoint is overshot with distance >= acceptable_overshoot_distance but was not caught earlier.
       This should never happen - waypoint should have been discarded or exception thrown before reaching here.
       Indicates inconsistent distance_along_lanelet calculations across simulation steps.
    */
    throw common::Error(
      "CRITICAL: Waypoint overshoot beyond acceptable distance detected for vehicle ",
      std::quoted(entity_status.name), " at time ", entity_status.time,
      "s. Distance to waypoint: ", distance_to_front_waypoint,
      " m (>= acceptable overshoot tolerance: ",
      FollowWaypointController::acceptable_overshoot_distance,
      " m). This should never happen - waypoint should have been discarded by "
      "is_waypoint_overshot_and_can_be_discarded() or exception thrown earlier. "
      "This indicates a BUG: inconsistent distance_along_lanelet calculations across simulation "
      "steps. "
      "Report this to the developer with full scenario details.");
  } else {
    if (std::isnan(remaining_time_to_front_waypoint)) {
      /// @note if the nearest waypoint is arrived at in this step without a specific arrival time, it will
      /// be considered as achieved
      const auto current_velocity = scalarToDirectionVector(speed, entity_status.pose.orientation);
      const auto this_step_distance = norm(current_velocity + desired_velocity) * 0.5 * step_time;
      if (std::isinf(remaining_time) && polyline_trajectory.shape.vertices.size() == 1) {
        /*
           Last waypoint with unspecified time: check overshoot before executing the step.
           this_step_distance is predicted travel distance in this step.
           If predicted overshoot (this_step_distance - distance_to_front_waypoint) > acceptable_overshoot_distance:
             throw exception NOW to prevent excessive overshoot that cannot be handled
           If predicted overshoot <= acceptable_overshoot_distance:
             continue (step will be executed later, overshoot will be acceptable, waypoint will be discarded)
        */
        if (
          this_step_distance >
          distance_to_front_waypoint + FollowWaypointController::acceptable_overshoot_distance) {
          throw common::Error(
            "Prevented excessive overshoot of the last waypoint for vehicle ",
            std::quoted(entity_status.name), " at time ", entity_status.time,
            "s. Executing this step would cause overshoot of ",
            this_step_distance - distance_to_front_waypoint,
            " m (this step distance: ", this_step_distance,
            " m, remaining distance: ", distance_to_front_waypoint,
            " m), which exceeds acceptable overshoot tolerance of ",
            FollowWaypointController::acceptable_overshoot_distance,
            " m. Step execution blocked to prevent unacceptable overshoot. "
            "This likely indicates that the initial entity state at the start of "
            "FollowTrajectoryAction "
            "makes precise stopping at the last waypoint impossible (e.g., too high speed, too "
            "close distance). "
            "If the initial conditions appear correct, report the scenario details to the "
            "developer.");
        } else if (follow_waypoint_controller.areConditionsOfArrivalMet(
                     acceleration, speed, distance_to_front_waypoint)) {
          return discard_the_front_waypoint_and_recurse();
        }
      } else {
        /*
           Intermediate waypoint (not last): discard just before overshoot, regardless of overshoot magnitude.
           Accuracy is less important for intermediate waypoints, so large overshoot is acceptable.
           If this_step_distance > distance_to_front_waypoint: waypoint will be overshot, discard it now.
        */
        if (this_step_distance > distance_to_front_waypoint) {
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
    return update_entity_status(entity_status, desired_velocity);
  }
}
}  // namespace follow_trajectory
}  // namespace traffic_simulator
