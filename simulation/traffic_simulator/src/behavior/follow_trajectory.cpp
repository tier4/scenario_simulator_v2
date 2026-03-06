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
#include <geometry/vector3/is_finite.hpp>
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
#include <traffic_simulator/utils/lanelet_map.hpp>
#include <traffic_simulator/utils/pose.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{
/**
 * @brief Computes entity state after one simulation step along a polyline trajectory.
 *
 * Validates entity state and trajectory, computes desired velocity respecting dynamic constraints
 * (acceleration, deceleration, jerk limits), updates position/orientation/velocity, and advances
 * to next waypoint when reached (recursive). Handles both timed and untimed waypoints.
 *
 * @return EntityStatus with updated state, or nullopt when final waypoint reached and stopped
 * @throws common::Error Entity/trajectory has NaN/infinity values, acceleration constraints
 *                       violated, arrival time missed, final waypoint unreachable (stopped too
 *                       far from target), or intermediate waypoint arrival time reached but
 *                       entity has not yet reached waypoint position (impossible timing)
 *
 * FINAL WAYPOINT CASES:
 * ┌──────────────────────────────┬─────────────────────────────────┬──────────────────────┐
 * │ Entity State                 │ Timing Condition                │ Return               │
 * ├──────────────────────────────┼─────────────────────────────────┼──────────────────────┤
 * │ Distance ≤ threshold         │ No arrival time specified       │ nullopt              │
 * │ Velocity = 0                 │ (remaining_time = NaN)          │ (trajectory done)    │
 * │ Acceleration = 0             │                                 │                      │
 * ├──────────────────────────────┼─────────────────────────────────┼──────────────────────┤
 * │ Distance ≤ threshold         │ Arrival time reached            │ nullopt              │
 * │ Velocity = 0                 │ (remaining_time < step_time/2)  │ (trajectory done)    │
 * │ Acceleration = 0             │                                 │                      │
 * ├──────────────────────────────┼─────────────────────────────────┼──────────────────────┤
 * │ Distance ≤ threshold         │ Arrival time not reached        │ EntityStatus         │
 * │ Velocity = 0                 │ (remaining_time ≥ step_time/2)  │ velocity = 0 (wait)  │
 * │ Acceleration = 0             │                                 │                      │
 * ├──────────────────────────────┼─────────────────────────────────┼──────────────────────┤
 * │ Distance > threshold         │ Any                             │ throw                │
 * │ Velocity = 0                 │                                 │ Error                │
 * │ Acceleration = 0             │                                 │                      │
 * ├──────────────────────────────┼─────────────────────────────────┼──────────────────────┤
 * │ Distance ≤ threshold         │ Any                             │ EntityStatus         │
 * │ Velocity > 0                 │                                 │ brake to stop        │
 * ├──────────────────────────────┼─────────────────────────────────┼──────────────────────┤
 * │ Distance > threshold         │ Any                             │ EntityStatus         │
 * │ Velocity > 0                 │                                 │ move toward waypoint │
 * └──────────────────────────────┴─────────────────────────────────┴──────────────────────┘
 *
 * INTERMEDIATE WAYPOINT CASES:
 * ┌───────────────────────────────────┬──────────────────────────────┬──────────────────────┐
 * │ Distance Check                    │ Timing Condition             │ Return               │
 * ├───────────────────────────────────┼──────────────────────────────┼──────────────────────┤
 * │ step_distance < remaining_distance│ No arrival time specified    │ EntityStatus         │
 * │ (waypoint not reached yet)        │ (remaining_time = NaN)       │ move toward waypoint │
 * ├───────────────────────────────────┼──────────────────────────────┼──────────────────────┤
 * │ step_distance < remaining_distance│ Arrival time not reached     │ EntityStatus         │
 * │ (waypoint not reached yet)        │ (remaining_time ≥ step/2)    │ move toward waypoint │
 * ├───────────────────────────────────┼──────────────────────────────┼──────────────────────┤
 * │ step_distance ≥ remaining_distance│ No arrival time specified    │ Recurse              │
 * │ (waypoint reached in this step)   │ (remaining_time = NaN)       │ (advance waypoint)   │
 * ├───────────────────────────────────┼──────────────────────────────┼──────────────────────┤
 * │ step_distance ≥ remaining_distance│ Arrival time reached         │ Recurse              │
 * │ (waypoint reached in this step)   │ (remaining_time < step/2)    │ (advance waypoint)   │
 * ├───────────────────────────────────┼──────────────────────────────┼──────────────────────┤
 * │ step_distance < remaining_distance│ Arrival time reached         │ throw                │
 * │ (waypoint not reached yet)        │ (remaining_time < step/2)    │ Error                │
 * └───────────────────────────────────┴──────────────────────────────┴──────────────────────┘
 */
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
  using math::geometry::isfinite;

  using geometry_msgs::msg::Vector3;
  using math::geometry::convertDirectionToQuaternion;
  using math::geometry::convertQuaternionToEulerAngle;
  using math::geometry::getRotation;
  using math::geometry::hypot;
  using math::geometry::norm;
  using math::geometry::scalarToDirectionVector;

  constexpr bool include_adjacent_lanelet{false};
  constexpr bool include_opposite_direction{false};
  constexpr bool allow_lane_change{true};
  constexpr bool desired_velocity_is_global{true};
  /// @note debug
  constexpr bool verbose_input_state{false};
  constexpr bool verbose_action{false};
  constexpr bool verbose_move_action{false};

  const auto should_include_crosswalk =
    (entity_status.type.type == traffic_simulator_msgs::msg::EntityType::PEDESTRIAN) ||
    (entity_status.type.type == traffic_simulator_msgs::msg::EntityType::MISC_OBJECT);

  //  ==============================================
  //                 WAYPOINT QUERIES
  //  ==============================================

  const auto previous_waypoint = [&polyline_trajectory]() -> const auto &
  {
    return polyline_trajectory.shape.vertices[0];
  };

  const auto target_waypoint = [&polyline_trajectory]() -> const auto &
  {
    return polyline_trajectory.shape.vertices[1];
  };

  const auto nearest_timed_waypoint = [&polyline_trajectory]() {
    /// @note search starts from vertices[1], skipping previous_waypoint (vertices[0])
    return std::find_if(
      polyline_trajectory.shape.vertices.begin() + 1, polyline_trajectory.shape.vertices.end(),
      [](auto && vertex) { return isfinite(vertex.time); });
  };

  const auto is_nearest_timed_waypoint_final = [&nearest_timed_waypoint, &polyline_trajectory]() {
    /// @note returns true when no more timed waypoints exist or the nearest timed waypoint is the final waypoint
    return nearest_timed_waypoint() >= std::prev(polyline_trajectory.shape.vertices.end());
  };

  const auto is_target_waypoint_final = [&polyline_trajectory]() {
    return polyline_trajectory.shape.vertices.size() == 2;
  };

  //  ==============================================
  //                      DISTANCE
  //  ==============================================

  /// @note returns distance between two positions along lanelet
  /// @note falls back to Euclidean distance if lanelet matching fails
  const auto distance_along_lanelet =
    [&entity_status, &should_include_crosswalk, &matching_distance, &allow_lane_change,
     &include_adjacent_lanelet, &include_opposite_direction](const auto & from, const auto & to) {
      using geometry_msgs::msg::Pose;

      const RoutingConfiguration routing_configuration{allow_lane_change};
      const auto quaternion = convertDirectionToQuaternion(
        geometry_msgs::build<Vector3>().x(to.x - from.x).y(to.y - from.y).z(to.z - from.z));

      const auto from_pose = geometry_msgs::build<Pose>().position(from).orientation(quaternion);
      const auto from_canonicalized = pose::toCanonicalizedLaneletPose(
        from_pose, entity_status.bounding_box, should_include_crosswalk, matching_distance);
      if (not from_canonicalized) {
        return hypot(from, to);
      }

      const auto to_pose = geometry_msgs::build<Pose>().position(to).orientation(quaternion);
      const auto to_canonicalized = pose::toCanonicalizedLaneletPose(
        to_pose, entity_status.bounding_box, should_include_crosswalk, matching_distance);
      if (not to_canonicalized) {
        return hypot(from, to);
      }

      /**
       * DIRTY HACK!
       * Negative longitudinal distance (calculated along lanelet in opposite direction)
       * causes some scenarios to fail because of an unrelated issue with lanelet matching.
       * The issue is caused by wrongly matched lanelet poses and thus wrong distances.
       * When lanelet matching errors are fixed, this dirty hack can be removed.
       */
      const auto longitudinal_distance = distance::longitudinalDistance(
        from_canonicalized.value(), to_canonicalized.value(), include_adjacent_lanelet,
        include_opposite_direction, routing_configuration);
      if (not longitudinal_distance.has_value() or longitudinal_distance.value() < 0.0) {
        return hypot(from, to);
      }

      const auto lateral_distance = distance::lateralDistance(
        from_canonicalized.value(), to_canonicalized.value(), routing_configuration);
      if (not lateral_distance) {
        return hypot(from, to);
      }

      return std::hypot(longitudinal_distance.value(), lateral_distance.value());
    };

  /// @note returns distance from entity to specified waypoint
  const auto distance_to_waypoint =
    [&entity_status, &target_waypoint, &polyline_trajectory,
     &distance_along_lanelet](const auto waypoint_iterator) -> double {
    /// @note special case: distance from entity to previous waypoint (vertices[0])
    if (waypoint_iterator == std::begin(polyline_trajectory.shape.vertices)) {
      return distance_along_lanelet(
        entity_status.pose.position, waypoint_iterator->position.position);
    }
    /// @note distance from entity to target waypoint (vertices[1])
    auto accumulated_distance =
      distance_along_lanelet(entity_status.pose.position, target_waypoint().position.position);
    /// @note start from vertices[1] (target waypoint), accumulate distance to waypoint_iterator
    for (auto current_waypoint_iter = std::begin(polyline_trajectory.shape.vertices) + 1;
         0 < std::distance(current_waypoint_iter, waypoint_iterator); ++current_waypoint_iter) {
      const auto & current_position = current_waypoint_iter->position.position;
      const auto & next_position = std::next(current_waypoint_iter)->position.position;
      accumulated_distance += distance_along_lanelet(current_position, next_position);
    }
    return accumulated_distance;
  };

  /// @note returns distance from entity to target waypoint (vertices[1])
  const auto distance_to_target_waypoint = [&polyline_trajectory, &distance_to_waypoint]() {
    return distance_to_waypoint(polyline_trajectory.shape.vertices.begin() + 1);
  };

  /// @note returns distance from entity to nearest timed waypoint or final waypoint
  const auto distance_to_timed_or_final_waypoint = [&nearest_timed_waypoint, &polyline_trajectory,
                                                    &distance_to_waypoint]() -> double {
    if (const auto timed_waypoint = nearest_timed_waypoint();
        timed_waypoint != std::end(polyline_trajectory.shape.vertices)) {
      return distance_to_waypoint(timed_waypoint);
    } else {
      return distance_to_waypoint(std::end(polyline_trajectory.shape.vertices) - 1);
    }
  };

  //  ==============================================
  //                        TIME
  //  ==============================================

  /// @note returns remaining time to specified waypoint
  /// @note returns quiet_NaN if waypoint has no time constraint
  /// @note returns epsilon instead of 0.0 when exactly at arrival time
  const auto remaining_time_to_waypoint = [&polyline_trajectory,
                                           &entity_status](const auto & waypoint_vertex) {
    if (not isfinite(waypoint_vertex.time)) {
      return std::numeric_limits<double>::quiet_NaN();
    } else {
      const auto remaining_time =
        (isfinite(polyline_trajectory.base_time) ? polyline_trajectory.base_time : 0.0) +
        waypoint_vertex.time - entity_status.time;
      return remaining_time != 0.0 ? remaining_time : std::numeric_limits<double>::epsilon();
    }
  };

  /// @note returns remaining time to target waypoint (vertices[1])
  const auto remaining_time_to_target_waypoint = [&target_waypoint, &remaining_time_to_waypoint]() {
    return remaining_time_to_waypoint(target_waypoint());
  };

  /// @note returns remaining time to nearest timed waypoint or final waypoint
  /// @note returns quiet_NaN if no timed waypoints exist (indicates no time constraint)
  const auto remaining_time_to_nearest_timed_waypoint =
    [&nearest_timed_waypoint, &polyline_trajectory, &remaining_time_to_waypoint]() {
      if (const auto timed_waypoint = nearest_timed_waypoint();
          timed_waypoint != std::end(polyline_trajectory.shape.vertices)) {
        return remaining_time_to_waypoint(*timed_waypoint);
      } else {
        return std::numeric_limits<double>::quiet_NaN();
      }
    };

  //  ==============================================
  //                     VALIDATION
  //  ==============================================

  const auto validate_entity_status = [&entity_status]() {
    /// @note check entity position for NaN/infinity
    if (const auto & position = entity_status.pose.position; not isfinite(position)) {
      throw common::Error(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(entity_status.name),
        " coordinate value contains NaN or infinity. The value is [", position.x, ", ", position.y,
        ", ", position.z, "].");
      /// @note check linear acceleration for NaN/infinity
    } else if (const auto & linear_acceleration = entity_status.action_status.accel.linear;
               not isfinite(linear_acceleration)) {
      throw common::Error(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(entity_status.name),
        "'s linear acceleration value contains NaN or infinity. The value is [",
        linear_acceleration.x, ", ", linear_acceleration.y, ", ", linear_acceleration.z, "].");
      /// @note check angular acceleration for NaN/infinity
    } else if (const auto & angular_acceleration = entity_status.action_status.accel.angular;
               not isfinite(angular_acceleration)) {
      throw common::Error(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(entity_status.name),
        "'s angular acceleration value contains NaN or infinity. The value is [",
        angular_acceleration.x, ", ", angular_acceleration.y, ", ", angular_acceleration.z, "].");
      /// @note check linear velocity for NaN/infinity
    } else if (const auto & linear_velocity = entity_status.action_status.twist.linear;
               not isfinite(linear_velocity)) {
      throw common::Error(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(entity_status.name),
        "'s linear velocity value contains NaN or infinity. The value is [", linear_velocity.x,
        ", ", linear_velocity.y, ", ", linear_velocity.z, "].");
      /// @note check angular velocity for NaN/infinity
    } else if (const auto & angular_velocity = entity_status.action_status.twist.angular;
               not isfinite(angular_velocity)) {
      throw common::Error(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(entity_status.name),
        "'s angular velocity value contains NaN or infinity. The value is [", angular_velocity.x,
        ", ", angular_velocity.y, ", ", angular_velocity.z, "].");
    }
  };

  const auto validate_trajectory = [&entity_status, &polyline_trajectory]() {
    /// @note check trajectory has at least 2 waypoints (previous and target)
    if (polyline_trajectory.shape.vertices.size() < 2) {
      throw common::Error(
        "Trajectory must have at least 2 waypoints: previous and target. "
        "Case with single target waypoint is not supported.");
      /// @note check previous waypoint position for NaN/infinity
    } else if (const auto & previous_position =
                 polyline_trajectory.shape.vertices[0].position.position;
               not isfinite(previous_position)) {
      throw common::Error(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(entity_status.name),
        "'s previous waypoint position coordinate value contains NaN or infinity. The value is [",
        previous_position.x, ", ", previous_position.y, ", ", previous_position.z, "].");
      /// @note check target waypoint position for NaN/infinity
    } else if (const auto & target_position =
                 polyline_trajectory.shape.vertices[1].position.position;
               not isfinite(target_position)) {
      throw common::Error(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(entity_status.name),
        "'s target waypoint position coordinate value contains NaN or infinity. The value is [",
        target_position.x, ", ", target_position.y, ", ", target_position.z, "].");
    }
  };

  const auto validate_arrival_time = [&entity_status, &polyline_trajectory, &nearest_timed_waypoint,
                                      &remaining_time_to_waypoint, &step_time,
                                      &distance_to_waypoint]() {
    /// @note check if vehicle has not exceeded arrival time for timed waypoint
    if (const auto timed_waypoint = nearest_timed_waypoint();
        timed_waypoint != std::end(polyline_trajectory.shape.vertices)) {
      /// @note allow delay of 1 step time due to floating point accumulation
      if (const auto remaining_time = remaining_time_to_waypoint(*timed_waypoint);
          isfinite(remaining_time) and remaining_time < -step_time) {
        throw common::Error(
          "Vehicle ", std::quoted(entity_status.name), " with current time: ", entity_status.time,
          " failed to reach the trajectory waypoint at the specified time. The specified time is ",
          timed_waypoint->time, " (in ",
          (isfinite(polyline_trajectory.base_time) ? "absolute" : "relative"),
          " simulation time). Distance to this waypoint is: ", distance_to_waypoint(timed_waypoint),
          ". This may be due to unrealistic conditions of arrival time "
          "specification compared to vehicle parameters and dynamic constraints.");
      }
    }
  };

  const auto validate_acceleration_constraints = [&entity_status, &behavior_parameter,
                                                  &step_time]() {
    /// @note check max acceleration for NaN/infinity
    const auto max_acceleration = std::min(
      entity_status.action_status.accel.linear.x /* [m/s^2] */ +
        behavior_parameter.dynamic_constraints.max_acceleration_rate /* [m/s^3] */ *
          step_time /* [s] */,
      +behavior_parameter.dynamic_constraints.max_acceleration /* [m/s^2] */);
    if (not isfinite(max_acceleration)) {
      throw common::Error(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(entity_status.name),
        "'s maximum acceleration value is NaN or infinity. The value is ", max_acceleration, ". ");
    }
    /// @note check min acceleration (max deceleration) for NaN/infinity
    const auto min_acceleration = std::max(
      entity_status.action_status.accel.linear.x /* [m/s^2] */ -
        behavior_parameter.dynamic_constraints.max_deceleration_rate /* [m/s^3] */ *
          step_time /* [s] */,
      -behavior_parameter.dynamic_constraints.max_deceleration /* [m/s^2] */);
    if (not isfinite(min_acceleration)) {
      throw common::Error(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(entity_status.name),
        "'s minimum acceleration value is NaN or infinity. The value is ", min_acceleration, ". ");
    }
  };

  const auto validate_desired_motion = [&entity_status](
                                         const double desired_acceleration,
                                         const double desired_speed,
                                         const auto & desired_velocity) {
    /// @note check desired acceleration for NaN/infinity
    if (not isfinite(desired_acceleration)) {
      throw common::Error(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(entity_status.name),
        "'s desired acceleration value contains NaN or infinity. The value is ",
        desired_acceleration, ". ");
      /// @note check desired speed for NaN/infinity
    } else if (not isfinite(desired_speed)) {
      throw common::Error(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(entity_status.name), "'s desired speed value is NaN or infinity. The value is ",
        desired_speed, ". ");
      /// @note check desired velocity for NaN/infinity
    } else if (not isfinite(desired_velocity)) {
      throw common::Error(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(entity_status.name),
        "'s desired velocity contains NaN or infinity. The value is [", desired_velocity.x, ", ",
        desired_velocity.y, ", ", desired_velocity.z, "].");
    }
  };

  //  ==============================================
  //                  ENTITY UPDATE
  //  ==============================================

  const auto step_displacement_vector = [&entity_status,
                                         &step_time](const auto & desired_velocity) {
    const auto current_speed = norm(entity_status.action_status.twist.linear);
    const auto desired_speed = norm(desired_velocity);
    Vector3 step_direction;
    if (desired_speed > std::numeric_limits<double>::epsilon()) {
      step_direction = desired_velocity / desired_speed;
    } else {
      step_direction = scalarToDirectionVector(1.0, entity_status.pose.orientation);
    }
    return step_direction * (current_speed + desired_speed) * 0.5 * step_time;
  };

  const auto previous_target_waypoint_direction = [&previous_waypoint,
                                                   &target_waypoint](const auto & entity_status) {
    const auto waypoint_to_waypoint_direction =
      target_waypoint().position.position - previous_waypoint().position.position;
    if (norm(waypoint_to_waypoint_direction) > std::numeric_limits<double>::epsilon()) {
      return convertDirectionToQuaternion(waypoint_to_waypoint_direction);
    } else {
      return entity_status.pose.orientation;
    }
  };

  /// @note refine position when entity crosses lanelet boundary during a single simulation step
  /// @note detects transition by comparing current and updated lanelet_id, then adjusts position for precision
  /// @note returns refined position if transition occurs and refinement succeeds, otherwise nullopt
  const auto refine_position_for_lanelet_transition =
    [&should_include_crosswalk, &step_time, &desired_velocity_is_global](
      const auto & entity_status, const auto & updated_pose,
      const auto & desired_velocity) -> std::optional<geometry_msgs::msg::Point> {
    if (not entity_status.lanelet_pose_valid) {
      return std::nullopt;
    }

    const auto canonicalized_lanelet_pose =
      traffic_simulator::pose::toCanonicalizedLaneletPose(entity_status.lanelet_pose);
    if (not canonicalized_lanelet_pose) {
      return std::nullopt;
    }

    const auto estimated_next_canonicalized_lanelet_pose =
      traffic_simulator::pose::toCanonicalizedLaneletPose(updated_pose, should_include_crosswalk);
    if (not estimated_next_canonicalized_lanelet_pose) {
      return std::nullopt;
    }

    const auto next_lanelet_id =
      static_cast<LaneletPose>(estimated_next_canonicalized_lanelet_pose.value()).lanelet_id;
    return pose::updatePositionForLaneletTransition(
      canonicalized_lanelet_pose.value(), next_lanelet_id, desired_velocity,
      desired_velocity_is_global, step_time);
  };

  const auto update_entity_status = [&](const auto & entity_status, const auto & desired_velocity) {
    using math::geometry::operator+=;

    validate_entity_status();
    validate_trajectory();
    validate_acceleration_constraints();
    validate_arrival_time();
    if (not isfinite(desired_velocity)) {
      throw common::Error(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(entity_status.name),
        "'s desired velocity contains NaN or infinity. The value is [", desired_velocity.x, ", ",
        desired_velocity.y, ", ", desired_velocity.z, "].");
    }

    auto updated_status = entity_status;

    updated_status.pose.position += step_displacement_vector(desired_velocity);

    updated_status.pose.orientation = previous_target_waypoint_direction(entity_status);

    /// @note if entity crosses lanelet boundary during this step, refine position for improved precision
    if (
      const auto refined_position = refine_position_for_lanelet_transition(
        entity_status, updated_status.pose, desired_velocity)) {
      updated_status.pose.position = refined_position.value();
    }

    /// @note desired_velocity is in global frame, but twist.linear is in local (entity) frame
    /// @note in local frame, vehicle moves only forward (x-axis), so y and z components are zero
    updated_status.action_status.twist.linear.x = norm(desired_velocity);
    updated_status.action_status.twist.linear.y = 0;
    updated_status.action_status.twist.linear.z = 0;

    updated_status.action_status.twist.angular =
      convertQuaternionToEulerAngle(
        getRotation(entity_status.pose.orientation, updated_status.pose.orientation)) /
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

  //  ==============================================
  //                      VELOCITY
  //  ==============================================

  const auto constrained_brake_velocity = [&behavior_parameter, step_time](
                                            const double speed, const auto & orientation) {
    constexpr double target_braking_speed = 0.0;
    const auto controller =
      FollowWaypointController(behavior_parameter, step_time, true, target_braking_speed);
    const auto deceleration = std::max(
      controller.accelerationWithJerkConstraint(
        speed, target_braking_speed, behavior_parameter.dynamic_constraints.max_deceleration_rate),
      -behavior_parameter.dynamic_constraints.max_deceleration);
    return scalarToDirectionVector(speed + deceleration * step_time, orientation);
  };

  const auto velocity_from_speed = [&polyline_trajectory, &entity_status](
                                     const auto & position, const auto & target_position,
                                     const double speed) {
    const auto dx = target_position.x - position.x;
    const auto dy = target_position.y - position.y;
    /// @note use pitch from lanelet if entity is on lane, otherwise use pitch on target
    const auto pitch = entity_status.lanelet_pose_valid
                         ? -convertQuaternionToEulerAngle(entity_status.pose.orientation).y
                         : std::atan2(target_position.z - position.z, std::hypot(dy, dx));
    /// @note always use yaw on target
    const auto yaw = std::atan2(dy, dx);
    return geometry_msgs::build<geometry_msgs::msg::Vector3>()
      .x(std::cos(pitch) * std::cos(yaw) * speed)
      .y(std::cos(pitch) * std::sin(yaw) * speed)
      .z(std::sin(pitch) * speed);
  };

  const auto desired_velocity = [&]() {
    /*
       Note: The followingMode in OpenSCENARIO is passed as
       variable dynamic_constraints_ignorable. the value of the
       variable is `followingMode == position`.
    */
    if (not polyline_trajectory.dynamic_constraints_ignorable) {
      /*
         Note: The vector returned if
         dynamic_constraints_ignorable == true ignores parameters
         such as the maximum rudder angle of the vehicle entry. In
         this clause, such parameters must be respected and the
         rotation angle difference of the z-axis center of the
         vector must be kept below a certain value.
      */
      throw common::SimulationError("The followingMode is only supported for position.");
    }

    /*
      The controller provides the ability to calculate acceleration using constraints from the
      behavior_parameter. The value is_nearest_timed_waypoint_final() determines whether the calculated
      acceleration takes braking into account - it is true if the nearest waypoint with the
      specified time is the last waypoint or the nearest waypoint without the specified time is
      the last waypoint.

      If an arrival time was specified for any of the remaining waypoints, priority is given to
      meeting the arrival time, and the vehicle is driven at a speed at which the arrival time
      can be met.

      However, the controller allows passing target_speed as a speed which is followed by the
      controller. target_speed is passed only if no arrival time was specified for any of the
      remaining waypoints. If despite no arrival time in the remaining waypoints, target_speed is
      not set (it is std::nullopt), target_speed is assumed to be the same as max_speed from the
      behaviour_parameter.
    */
    const auto follow_waypoint_controller = FollowWaypointController(
      behavior_parameter, step_time, is_nearest_timed_waypoint_final(),
      not isfinite(remaining_time_to_nearest_timed_waypoint()) ? target_speed : std::nullopt);

    /*
      The desired acceleration is the acceleration at which the destination
      can be reached exactly at the specified time (= time remaining at zero).

      The desired acceleration is calculated to the nearest waypoint with a specified arrival
      time. It is calculated in such a way as to reach a constant linear speed as quickly as
      possible, ensuring arrival at a waypoint at the precise time and with the shortest possible
      distance. More precisely, the controller selects acceleration to minimize the distance to
      the waypoint that will be reached in a time step defined as the expected arrival time. In
      addition, the controller ensures a smooth stop at the last waypoint of the trajectory, with
      linear speed equal to zero and acceleration equal to zero.
    */
    double desired_acceleration = 0.0;
    try {
      desired_acceleration = follow_waypoint_controller.getAcceleration(
        remaining_time_to_nearest_timed_waypoint(), distance_to_timed_or_final_waypoint(),
        entity_status, update_entity_status, distance_along_lanelet);
    } catch (const ControllerError & e) {
      throw common::Error(
        "Vehicle ", std::quoted(entity_status.name),
        " - controller operation problem encountered. ",
        follow_waypoint_controller.getFollowedWaypointDetails(polyline_trajectory), e.what());
    }
    const auto desired_speed =
      entity_status.action_status.twist.linear.x + desired_acceleration * step_time;
    const auto desired_velocity = velocity_from_speed(
      entity_status.pose.position, target_waypoint().position.position, desired_speed);
    validate_desired_motion(desired_acceleration, desired_speed, desired_velocity);
    return desired_velocity;
  };

  //  ==============================================
  //                WAYPOINT HANDLING
  //  ==============================================

  const auto log_waypoint_action = [&entity_status, &target_waypoint, &distance_to_target_waypoint,
                                    &remaining_time_to_target_waypoint, &polyline_trajectory,
                                    &is_target_waypoint_final, verbose_action, verbose_move_action](
                                     std::string_view reason, std::string_view action,
                                     const double this_step_distance = 0.0) {
    if (not verbose_action) {
      return;
    } else if (not verbose_move_action and action == "Move") {
      return;
    } else {
      const auto distance = distance_to_target_waypoint();
      const auto remaining_time = remaining_time_to_target_waypoint();
      const auto & target_pos = target_waypoint().position.position;
      const auto waypoints_left = polyline_trajectory.shape.vertices.size() - 2;
      const auto is_final = is_target_waypoint_final();

      /// @note this_step_distance does not affect final waypoint handling logic, so it's not displayed for final waypoints
      std::cout << std::fixed << std::setprecision(2)
                << "[FTA-RESULT] Entity: " << entity_status.name << ", time: " << entity_status.time
                << "s, step distance: "
                << (is_final ? "N/A" : std::to_string(this_step_distance) + "m")
                << " | Waypoint: type: "
                << (is_final ? "final"
                             : "intermediate (" + std::to_string(waypoints_left) + " left)")
                << ", position: [" << target_pos.x << ", " << target_pos.y << ", " << target_pos.z
                << "] | State: distance: " << distance << "m, remaining time: "
                << (isfinite(remaining_time) ? std::to_string(remaining_time) + "s" : "N/A")
                << " | Result: action: " << action << ", reason: " << reason << std::endl;
    }
  };

  const auto discard_previous_waypoint_and_recurse =
    [&entity_status, &polyline_trajectory, &behavior_parameter, &step_time, &matching_distance,
     &target_speed, &target_waypoint]() {
      /*
         The OpenSCENARIO standard does not define the behavior when the value of
         Timing.domainAbsoluteRelative is "relative". The standard only states
         "Definition of time value context as either absolute or relative", and
         it is completely unclear when the relative time starts.

         This implementation has interpreted the specification as follows:
         Relative time starts from the start of FollowTrajectoryAction or from
         the time of reaching the previous "waypoint with arrival time".

         Note: isfinite(polyline_trajectory.base_time) means
         "Timing.domainAbsoluteRelative is relative".

         Note: isfinite(previous_waypoint_vertex().time)
         means "The waypoint about to be popped is the waypoint with the
         specified arrival time".
      */
      if (isfinite(polyline_trajectory.base_time) and isfinite(target_waypoint().time)) {
        polyline_trajectory.base_time = entity_status.time;
      }

      std::rotate(
        std::begin(polyline_trajectory.shape.vertices),
        std::begin(polyline_trajectory.shape.vertices) + 1,
        std::end(polyline_trajectory.shape.vertices));

      if (not polyline_trajectory.closed) {
        polyline_trajectory.shape.vertices.pop_back();
      }

      return makeUpdatedStatus(
        entity_status, polyline_trajectory, behavior_parameter, step_time, matching_distance,
        target_speed);
    };

  const auto handle_intermediate_waypoint =
    [&](const double distance, const double remaining_time) -> std::optional<EntityStatus> {
    const auto next_velocity = desired_velocity();
    const auto this_step_distance = norm(step_displacement_vector(next_velocity));

    /// @note handle intermediate waypoint without arrival time defined
    if (not isfinite(remaining_time)) {
      if (this_step_distance >= distance or distance < FollowWaypointController::local_epsilon) {
        log_waypoint_action("Reached (no time)", "Recurse", this_step_distance);
        return discard_previous_waypoint_and_recurse();
      } else {
        log_waypoint_action("Not reached (no time)", "Move", this_step_distance);
        return update_entity_status(entity_status, next_velocity);
      }
    }

    /// @note handle intermediate waypoint with arrival time defined
    /// @note use step_time/2 threshold to account for floating point accumulation errors
    if (isDefinitelyLessThan(remaining_time, step_time / 2.0)) {
      if (this_step_distance >= distance or distance < FollowWaypointController::local_epsilon) {
        log_waypoint_action("Reached on time", "Recurse", this_step_distance);
        return discard_previous_waypoint_and_recurse();
      } else {
        log_waypoint_action("Time reached but distance too far", "Error", this_step_distance);
        throw common::Error(
          "Vehicle ", std::quoted(entity_status.name), " at time ", entity_status.time,
          "s (remaining time is ", remaining_time,
          "s), has completed a trajectory to the nearest waypoint with specified time equal to ",
          target_waypoint().time, "s at a distance equal to ", distance,
          " from that waypoint which is greater than this step distance: ", this_step_distance);
      }
    }

    log_waypoint_action("Time not reached", "Move", this_step_distance);
    return update_entity_status(entity_status, next_velocity);
  };

  const auto is_entity_immobile = [&entity_status]() {
    return std::abs(entity_status.action_status.twist.linear.x) <
             FollowWaypointController::local_epsilon &&
           std::abs(entity_status.action_status.accel.linear.x) <
             FollowWaypointController::local_epsilon;
  };

  const auto handle_final_waypoint =
    [&](const double distance, const double remaining_time) -> std::optional<EntityStatus> {
    constexpr double distance_threshold = FollowWaypointController::remaining_distance_tolerance;

    if (is_entity_immobile() and distance <= distance_threshold) {
      if (not isfinite(remaining_time)) {
        log_waypoint_action("Reached and immobile (no time)", "Complete");
        return std::nullopt;
      } else if (isDefinitelyLessThan(remaining_time, step_time / 2.0)) {
        log_waypoint_action("Reached and immobile on time", "Complete");
        return std::nullopt;
      } else {
        log_waypoint_action("Reached but time not reached", "Wait");
        /// @note distance sufficient and vehicle immobile, but arrival time not yet reached - wait with zero velocity
        return update_entity_status(
          entity_status, geometry_msgs::build<Vector3>().x(0.0).y(0.0).z(0.0));
      }
    }

    if (is_entity_immobile() and distance > distance_threshold) {
      /// @note only report error if entity progressed past previous waypoint but stopped too far from target
      /// @note entities waiting near previous waypoint (e.g., target_speed=0.0 at trajectory start) are allowed to be immobile
      if (distance < distance_to_waypoint(polyline_trajectory.shape.vertices.begin())) {
        log_waypoint_action("Immobile but distance exceeds threshold", "Error");
        throw common::Error(
          "Vehicle ", std::quoted(entity_status.name),
          " is immobile and only final waypoint left, but distance to final waypoint is ", distance,
          " m, which exceeds the threshold of ", distance_threshold, " m.");
      }
    }

    if (distance <= distance_threshold) {
      log_waypoint_action("Within threshold but moving", "Brake");
      /// @note distance within threshold but vehicle still moving - apply constrained braking to reach immobile state
      return update_entity_status(
        entity_status,
        constrained_brake_velocity(
          entity_status.action_status.twist.linear.x, entity_status.pose.orientation));
    }

    log_waypoint_action("Not reached yet", "Move");
    return update_entity_status(entity_status, desired_velocity());
  };

  //  ==============================================
  //                    EXECUTION
  //  ==============================================

  if (verbose_input_state) {
    auto const orientation = convertQuaternionToEulerAngle(entity_status.pose.orientation);
    auto const previous = previous_waypoint().position.position;
    auto const target = target_waypoint().position.position;

    std::cout << "  --------------------   " << std::endl;
    std::cout << "name: " << entity_status.name << std::endl;
    std::cout << "time: " << entity_status.time << std::endl;
    std::cout << "lanelet_pose_valid: " << (entity_status.lanelet_pose_valid ? "true" : "false")
              << std::endl;
    std::cout << "position: " << entity_status.pose.position.x << ", "
              << entity_status.pose.position.y << ", " << entity_status.pose.position.z
              << std::endl;
    std::cout << "orientation: " << orientation.x << ", " << orientation.y << ", " << orientation.z
              << std::endl;
    std::cout << "speed: " << entity_status.action_status.twist.linear.x << std::endl;
    std::cout << "acceleration: " << entity_status.action_status.accel.linear.x << std::endl;
    std::cout << "is_entity_immobile: " << is_entity_immobile() << std::endl;
    std::cout << "  ---  " << std::endl;
    std::cout << "target_speed: " << (target_speed ? std::to_string(target_speed.value()) : "N/A")
              << std::endl;
    std::cout << "target_time: "
              << (isfinite(target_waypoint().time) ? std::to_string(target_waypoint().time) : "N/A")
              << std::endl;
    std::cout << "previous_waypoint: " << previous.x << ", " << previous.y << ", " << previous.z
              << std::endl;
    std::cout << "target_waypoint: " << target.x << ", " << target.y << ", " << target.z
              << std::endl;
    std::cout << "is_target_waypoint_final: " << is_target_waypoint_final() << std::endl;
    std::cout << "distance_to_previous: "
              << distance_to_waypoint(polyline_trajectory.shape.vertices.begin()) << std::endl;
    std::cout << "distance_to_target: " << distance_to_target_waypoint() << std::endl;
    std::cout << "distance_to_timed_or_final: " << distance_to_timed_or_final_waypoint()
              << std::endl;
    std::cout << "time_to_target: " << remaining_time_to_target_waypoint() << std::endl;
    std::cout << "  ---  " << std::endl;
    std::cout << "trajectory_base_time: "
              << (isfinite(polyline_trajectory.base_time)
                    ? std::to_string(polyline_trajectory.base_time)
                    : "N/A")
              << std::endl;
    std::cout << "trajectory_closed: " << (polyline_trajectory.closed ? "true" : "false")
              << std::endl;
  }

  if (is_target_waypoint_final()) {
    return handle_final_waypoint(
      distance_to_target_waypoint(), remaining_time_to_target_waypoint());
  } else {
    return handle_intermediate_waypoint(
      distance_to_target_waypoint(), remaining_time_to_target_waypoint());
  }
}
}  // namespace follow_trajectory
}  // namespace traffic_simulator
