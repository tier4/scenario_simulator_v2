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
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/inner_product.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/operator.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <traffic_simulator/behavior/polyline_trajectory_follower.hpp>
#include <traffic_simulator_msgs/msg/action_status.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{
/// @todo  move to some geometry lib
template <typename T>
auto isFiniteVector3(const T & vec) -> bool
{
  static_assert(math::geometry::IsLikeVector3<std::decay_t<decltype(vec)>>::value);
  return std::isfinite(vec.x) and std::isfinite(vec.y) and std::isfinite(vec.z);
}

/// @todo move to ValidatedEntityStatus cpp - each method below

ValidatedEntityStatus::ValidatedEntityStatus(
  const EntityStatus & entity_status, const BehaviorParameter & behavior_parameter,
  const double step_time)
: time(entity_status.time),
  name(entity_status.name),
  lanelet_pose_valid(entity_status.lanelet_pose_valid),
  pose(validatedPose()),
  linear_speed(validatedLinearSpeed()),
  linear_acceleration(validatedLinearAcceleration(step_time)),
  velocity(validatedVelocity()),
  bounding_box(entity_status.bounding_box),
  behavior_parameter(behavior_parameter),
  entity_status(entity_status)
{
}

auto ValidatedEntityStatus::validatedPose() const noexcept(false) -> const Pose &
{
  if (not isFiniteVector3(entity_status.pose.position)) {
    THROW_SIMULATION_ERROR(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Entity ",
      std::quoted(name), " coordinate value contains NaN or infinity. The value is [",
      entity_status.pose.position.x, ", ", entity_status.pose.position.y, ", ",
      entity_status.pose.position.z, "].");
  }
  /// @todo add validate orientation
  else {
    return entity_status.pose;
  }
}

auto ValidatedEntityStatus::validatedLinearSpeed() const noexcept(false) -> double
{
  if (not std::isfinite(entity_status.action_status.twist.linear.x)) {  // [m/s]
    THROW_SIMULATION_ERROR(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Entity ",
      std::quoted(name), "'s speed value is NaN or infinity. The value is ",
      entity_status.action_status.twist.linear.x, ". ");
  } else {
    return entity_status.action_status.twist.linear.x;
  }
}

auto ValidatedEntityStatus::validatedLinearAcceleration(const double step_time) const
  noexcept(false) -> double
{
  const auto throwDetailedException = [this](const std::string & text, const double value) {
    std::stringstream ss;
    ss << "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
          "following information to the developer: Entity "
       << std::quoted(name) << "'s";
    ss << text << " value is NaN or infinity. The value is " << value << ".";
    // THROW_SIMULATION_ERROR(ss.str());
  };

  const double acceleration = entity_status.action_status.accel.linear.x;
  const double max_acceleration = std::min(
    acceleration + behavior_parameter.dynamic_constraints.max_acceleration_rate * step_time,
    +behavior_parameter.dynamic_constraints.max_acceleration);
  const double min_acceleration = std::max(
    acceleration - behavior_parameter.dynamic_constraints.max_deceleration_rate * step_time,
    -behavior_parameter.dynamic_constraints.max_deceleration);
  if (not std::isfinite(acceleration)) {
    throwDetailedException("acceleration", acceleration);
  } else if (not std::isfinite(max_acceleration)) {
    throwDetailedException("maximum acceleration", max_acceleration);
  } else if (not std::isfinite(min_acceleration)) {
    throwDetailedException("minimum acceleration", min_acceleration);
  } else {
    return acceleration;
  }
}

auto ValidatedEntityStatus::validatedVelocity() const -> Vector3
{
  /// @todo add validation
  const auto euler_angles = math::geometry::convertQuaternionToEulerAngle(pose.orientation);
  const double pitch = -euler_angles.y;
  const double yaw = euler_angles.z;
  return geometry_msgs::build<Vector3>()
    .x(std::cos(pitch) * std::cos(yaw) * linear_speed)
    .y(std::cos(pitch) * std::sin(yaw) * linear_speed)
    .z(std::sin(pitch) * linear_speed);
}

auto ValidatedEntityStatus::updatedEntityStatus(
  const Vector3 & desired_velocity, const double step_time) const -> EntityStatus
{
  /// @todo increase readability
  using math::geometry::operator+;
  using math::geometry::operator-;
  using math::geometry::operator*;
  using math::geometry::operator/;

  const auto updated_pose_orientation = [this](const Vector3 & desired_velocity) -> Quaternion {
    if (desired_velocity.y == 0.0 && desired_velocity.x == 0.0 && desired_velocity.z == 0.0) {
      // do not change orientation if there is no designed_velocity vector
      return pose.orientation;
    } else {
      // if there is a designed_velocity vector, set the orientation in the direction of it
      const Vector3 direction =
        geometry_msgs::build<Vector3>()
          .x(0.0)
          .y(std::atan2(-desired_velocity.z, std::hypot(desired_velocity.x, desired_velocity.y)))
          .z(std::atan2(desired_velocity.y, desired_velocity.x));
      return math::geometry::convertEulerAngleToQuaternion(direction);
    }
  }(desired_velocity);

  const auto updated_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                              .position(entity_status.pose.position + desired_velocity * step_time)
                              .orientation(updated_pose_orientation);

  const auto updated_action_status_twist_linear =
    geometry_msgs::build<Vector3>().x(math::geometry::norm(desired_velocity)).y(0.0).z(0.0);
  const auto updated_action_status_twist_angular =
    math::geometry::convertQuaternionToEulerAngle(
      math::geometry::getRotation(entity_status.pose.orientation, updated_pose_orientation)) /
    step_time;
  const auto updated_action_status_twist = geometry_msgs::build<geometry_msgs::msg::Twist>()
                                             .linear(updated_action_status_twist_linear)
                                             .angular(updated_action_status_twist_angular);
  const auto updated_action_status_accel =
    geometry_msgs::build<geometry_msgs::msg::Accel>()
      .linear(
        (updated_action_status_twist_linear - entity_status.action_status.twist.linear) / step_time)
      .angular(
        (updated_action_status_twist_angular - entity_status.action_status.twist.angular) /
        step_time);
  const auto updated_action_status =
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::ActionStatus>()
      .current_action(entity_status.action_status.current_action)
      .twist(updated_action_status_twist)
      .accel(updated_action_status_accel)
      .linear_jerk(entity_status.action_status.linear_jerk);

  const auto updated_time = entity_status.time + step_time;
  const auto updated_lanelet_pose_valid = false;

  return traffic_simulator_msgs::build<traffic_simulator_msgs::msg::EntityStatus>()
    .type(entity_status.type)
    .subtype(entity_status.subtype)
    .time(updated_time)
    .name(entity_status.name)
    .bounding_box(entity_status.bounding_box)
    .action_status(updated_action_status)
    .pose(updated_pose)
    .lanelet_pose(entity_status.lanelet_pose)
    .lanelet_pose_valid(updated_lanelet_pose_valid);
}

/// @todo move to PolylineTrajectoryFollower cpp - each method below

auto PolylineTrajectoryFollower::discardTheFrontWaypoint(
  PolylineTrajectory & polyline_trajectory, const double current_time) noexcept(false) -> void
{
  if (polyline_trajectory.shape.vertices.empty()) {
    THROW_SIMULATION_ERROR(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: ",
      "Attempted to access an element of an empty vector");
  } else {
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
      polyline_trajectory.base_time = current_time;
    }

    std::rotate(
      std::begin(polyline_trajectory.shape.vertices),
      std::begin(polyline_trajectory.shape.vertices) + 1,
      std::end(polyline_trajectory.shape.vertices));

    if (not polyline_trajectory.closed) {
      polyline_trajectory.shape.vertices.pop_back();
    }
  }
}

auto PolylineTrajectoryFollower::updatedEntityStatus(
  const ValidatedEntityStatus & validated_status, PolylineTrajectory & polyline_trajectory,
  const std::optional<double> target_speed, const double step_time, const double matching_distance,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) noexcept(false)
  -> std::optional<EntityStatus>
{
  /*
    The following code implements the steering behavior known as "seek". See
    "Steering Behaviors For Autonomous Characters" by Craig Reynolds for more
    information.

    See
    https://www.researchgate.net/publication/2495826_Steering_Behaviors_For_Autonomous_Characters
  */

  if (step_time <= 0.0) {
    THROW_SIMULATION_ERROR(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: ",
      "non-positive step time provided");
    /// @todo rethink - for empty - when nullopt is returned and when an exception is thrown
  } else if (polyline_trajectory.shape.vertices.empty()) {
    return std::nullopt;
  } else if (
    const auto updated_entity_opt = PolylineTrajectoryFollowerStep(
                                      validated_status, polyline_trajectory, target_speed,
                                      step_time, matching_distance, hdmap_utils_ptr)
                                      .updatedEntityStatus()) {
    return updated_entity_opt.value();
  } else {
    discardTheFrontWaypoint(polyline_trajectory, validated_status.time);
    return updatedEntityStatus(
      validated_status, polyline_trajectory, target_speed, step_time, matching_distance,
      hdmap_utils_ptr);
  }
}

////////////////////

auto PolylineTrajectoryFollowerStep::nearestWaypointWithSpecifiedTimeIterator() const
  -> WaypointIterator
{
  return std::find_if(
    polyline_trajectory.shape.vertices.cbegin(), polyline_trajectory.shape.vertices.cend(),
    [](const auto & vertex) { return not std::isnan(vertex.time); });
}

/*
  The controller provides the ability to calculate acceleration using constraints from the
  behavior_parameter. The value isNearestWaypointWithSpecifiedTimeSameAsLastWaypoint()
  determines whether the calculated acceleration takes braking into account - it is true
  if the nearest waypoint with the specified time is the last waypoint or
  there is no waypoint with a specified time.

  If an arrival time was specified for any of the remaining waypoints, priority is given to
  meeting the arrival time, and the vehicle is driven at a speed at which the arrival time can
  be met.

  However, the controller allows passing target_speed as a speed which is followed by the
  controller. target_speed is passed only if no arrival time was specified for any of the
  remaining waypoints. If despite no arrival time in the remaining waypoints, target_speed is
  not set (it is std::nullopt), target_speed is assumed to be the same as max_speed from the
  behaviour_parameter.
*/
auto PolylineTrajectoryFollowerStep::isNearestWaypointWithSpecifiedTimeSameAsLastWaypoint() const
  -> bool
{
  return nearest_waypoint_with_specified_time_it >=
         std::prev(polyline_trajectory.shape.vertices.cend());
}

auto PolylineTrajectoryFollowerStep::validatedNearestWaypointPosition() const noexcept(false)
  -> const Point &
{
  if (polyline_trajectory.shape.vertices.empty()) {
    THROW_SIMULATION_ERROR(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "attempted to dereference an element of an empty PolylineTrajectory");
  } else if (not isFiniteVector3(polyline_trajectory.shape.vertices.front().position.position)) {
    THROW_SIMULATION_ERROR(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Entity ",
      std::quoted(validated_status.name),
      "'s target position coordinate value contains NaN or infinity. The value is [",
      polyline_trajectory.shape.vertices.front().position.position.x, ", ",
      polyline_trajectory.shape.vertices.front().position.position.y, ", ",
      polyline_trajectory.shape.vertices.front().position.position.z, "].");
  } else {
    return polyline_trajectory.shape.vertices.front().position.position;
  }
}

auto PolylineTrajectoryFollowerStep::totalRemainingDistance(
  const double matching_distance,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) const -> double
{
  /*
    Note for anyone working on adding support for followingMode follow
    to this function (FollowPolylineTrajectoryAction::tick) in the
    future: if followingMode is follow, this distance calculation may be
    inappropriate.
  */
  const auto total_distance_to = [this, &matching_distance,
                                  &hdmap_utils_ptr](const WaypointIterator last) {
    return std::accumulate(
      polyline_trajectory.shape.vertices.cbegin(), last, 0.0,
      [this, &matching_distance, &hdmap_utils_ptr](
        const double total_distance, const auto & vertex) {
        const auto next = std::next(&vertex);
        return total_distance + distanceAlongLanelet(
                                  vertex.position.position, next->position.position,
                                  matching_distance, hdmap_utils_ptr);
      });
  };

  if (nearest_waypoint_with_specified_time_it == std::cend(polyline_trajectory.shape.vertices)) {
    return distance_to_nearest_waypoint +
           total_distance_to(std::cend(polyline_trajectory.shape.vertices) - 1);
  } else {
    return distance_to_nearest_waypoint +
           total_distance_to(nearest_waypoint_with_specified_time_it);
  }
}

auto PolylineTrajectoryFollowerStep::totalRemainingTime() const -> double
{
  if (nearest_waypoint_with_specified_time_it == std::cend(polyline_trajectory.shape.vertices)) {
    return std::numeric_limits<double>::infinity();
  } else {
    const double remaining_time =
      (std::isnan(polyline_trajectory.base_time) ? 0.0 : polyline_trajectory.base_time) +
      nearest_waypoint_with_specified_time_it->time - validated_status.time;

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
    if (remaining_time < -step_time) {
      THROW_SIMULATION_ERROR(
        "Vehicle ", std::quoted(validated_status.name),
        " failed to reach the trajectory waypoint at the specified time. The specified time "
        "is ",
        nearest_waypoint_with_specified_time_it->time, " (in ",
        (not std::isnan(polyline_trajectory.base_time) ? "absolute" : "relative"),
        " simulation time). This may be due to unrealistic conditions of arrival time "
        "specification compared to vehicle parameters and dynamic constraints.");
    } else {
      return remaining_time == 0.0 ? std::numeric_limits<double>::epsilon() : remaining_time;
    }
  }
}

auto PolylineTrajectoryFollowerStep::updatedEntityStatus() const -> std::optional<EntityStatus>
{
  /// @todo increase readability
  /*
    The following code implements the steering behavior known as "seek". See
    "Steering Behaviors For Autonomous Characters" by Craig Reynolds for more
    information.

    See
    https://www.researchgate.net/publication/2495826_Steering_Behaviors_For_Autonomous_Characters
  */

  using math::geometry::operator+;
  using math::geometry::operator-;
  using math::geometry::operator*;
  using math::geometry::operator/;
  using math::geometry::operator+=;

  /*
    This clause is to avoid division-by-zero errors in later clauses with
    distance_to_front_waypoint as the denominator if the distance
    miraculously becomes zero.
  */
  const auto epsilon = std::numeric_limits<double>::epsilon();
  if (math::arithmetic::isDefinitelyLessThan(distance_to_nearest_waypoint, epsilon)) {
    return std::nullopt;
  } else if (math::arithmetic::isDefinitelyLessThan(total_remining_distance, epsilon)) {
    return std::nullopt;
  }

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
  const auto desired_acceleration = validatedDesiredAcceleration();
  const auto desired_speed = validatedDesiredSpeed(desired_acceleration);
  const auto desired_velocity = validatedDesiredVelocity(desired_speed);

  if (
    validated_status.linear_speed * step_time > distance_to_nearest_waypoint and
    math::geometry::innerProduct(desired_velocity, validated_status.velocity) < 0.0) {
    return std::nullopt;
  }

  validatePredictedState(desired_acceleration);
  if (std::isnan(time_to_nearest_waypoint)) {
    /*
      If the nearest waypoint is arrived at in this step without a specific arrival time, it will
      be considered as achieved
    */
    if (std::isinf(total_remining_distance) and polyline_trajectory.shape.vertices.size() == 1UL) {
      /*
        If the trajectory has only waypoints with unspecified time, the last one is followed using
        maximum speed including braking - in this case accuracy of arrival is checked
      */
      if (follow_waypoint_controller.areConditionsOfArrivalMet(
            validated_status.linear_acceleration, validated_status.linear_speed,
            distance_to_nearest_waypoint)) {
        return std::nullopt;
      } else {
        return validated_status.updatedEntityStatus(desired_velocity, step_time);
      }
    } else {
      /*
        If it is an intermediate waypoint with an unspecified time, the accuracy of the arrival is
        irrelevant
      */
      if (const double this_step_distance =
            (validated_status.linear_speed + desired_acceleration * step_time) * step_time;
          this_step_distance > distance_to_nearest_waypoint) {
        return std::nullopt;
      } else {
        return validated_status.updatedEntityStatus(desired_velocity, step_time);
      }
    }
    /*
      If there is insufficient time left for the next calculation step.
      The value of step_time/2 is compared, as the remaining time is affected by floating point
      inaccuracy, sometimes it reaches values of 1e-7 (almost zero, but not zero) or (step_time -
      1e-7) (almost step_time). Because the step is fixed, it should be assumed that the value
      here is either equal to 0 or step_time. Value step_time/2 allows to return true if no next
      step is possible (time_to_nearest_waypoint is almost zero).
    */
  } else if (math::arithmetic::isDefinitelyLessThan(time_to_nearest_waypoint, step_time / 2.0)) {
    if (follow_waypoint_controller.areConditionsOfArrivalMet(
          validated_status.linear_acceleration, validated_status.linear_speed,
          distance_to_nearest_waypoint)) {
      return std::nullopt;
    } else {
      THROW_SIMULATION_ERROR(
        "Vehicle ", std::quoted(validated_status.name), " at time ", validated_status.time,
        "s (remaining time is ", time_to_nearest_waypoint,
        "s), has completed a trajectory to the nearest waypoint with", " specified time equal to ",
        polyline_trajectory.shape.vertices.front().time, "s at a distance equal to ",
        distance_to_nearest_waypoint,
        " from that waypoint which is greater than the accepted accuracy.");
    }
  } else {
    return validated_status.updatedEntityStatus(desired_velocity, step_time);
  }

  /*
    Note: If obstacle avoidance is to be implemented, the steering behavior
    known by the name "collision avoidance" should be synthesized here into
    steering.
  */
}

auto PolylineTrajectoryFollowerStep::validatedDesiredAcceleration() const noexcept(false) -> double
{
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

  try {
    const double desired_acceleration = follow_waypoint_controller.getAcceleration(
      total_remaining_time, total_remining_distance, validated_status.linear_acceleration,
      validated_status.linear_speed);

    if (not std::isfinite(desired_acceleration)) {
      THROW_SIMULATION_ERROR(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Entity ",
        std::quoted(validated_status.name),
        "'s desired acceleration value contains NaN or infinity. The value is ",
        desired_acceleration, ". ");
    }
    return desired_acceleration;
  } catch (const ControllerError & e) {
    THROW_SIMULATION_ERROR(
      "Vehicle", std::quoted(validated_status.name), " - controller operation problem encountered.",
      follow_waypoint_controller.getFollowedWaypointDetails(polyline_trajectory), e.what());
  }
}

auto PolylineTrajectoryFollowerStep::validatedDesiredSpeed(const double desired_acceleration) const
  noexcept(false) -> double
{
  const double desired_speed = validated_status.linear_speed + desired_acceleration * step_time;
  if (not std::isfinite(desired_speed)) {
    THROW_SIMULATION_ERROR(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Entity ",
      std::quoted(validated_status.name),
      "'s desired speed value is NaN or infinity. The value is ", desired_speed, ". ");
  } else {
    return desired_speed;
  }
}

auto PolylineTrajectoryFollowerStep::validatedDesiredVelocity(const double desired_speed) const
  noexcept(false) -> Vector3
{
  /// @todo increase readability
  /*
    If not dynamic_constraints_ignorable, the linear distance should cause
    problems.
  */

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
    THROW_SIMULATION_ERROR("The followingMode is only supported for position.");
  }

  const double dx = nearest_waypoint_position.x - validated_status.pose.position.x;
  const double dy = nearest_waypoint_position.y - validated_status.pose.position.y;
  // if entity is on lane use pitch from lanelet, otherwise use pitch on target
  const double pitch =
    validated_status.lanelet_pose_valid
      ? -math::geometry::convertQuaternionToEulerAngle(validated_status.pose.orientation).y
      : std::atan2(
          nearest_waypoint_position.z - validated_status.pose.position.z, std::hypot(dy, dx));
  const double yaw = std::atan2(dy, dx);  // Use yaw on target

  const auto desired_velocity = geometry_msgs::build<Vector3>()
                                  .x(std::cos(pitch) * std::cos(yaw) * desired_speed)
                                  .y(std::cos(pitch) * std::sin(yaw) * desired_speed)
                                  .z(std::sin(pitch) * desired_speed);
  if (not isFiniteVector3(desired_velocity)) {
    THROW_SIMULATION_ERROR(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Entity ",
      std::quoted(validated_status.name),
      "'s desired velocity contains NaN or infinity. The value is [", desired_velocity.x, ", ",
      desired_velocity.y, ", ", desired_velocity.z, "].");
  }
  return desired_velocity;
}

auto PolylineTrajectoryFollowerStep::validatePredictedState(const double desired_acceleration) const
  noexcept(false) -> void
{
  const auto predicted_state_opt = follow_waypoint_controller.getPredictedWaypointArrivalState(
    desired_acceleration, total_remaining_time, total_remining_distance,
    validated_status.linear_acceleration, validated_status.linear_speed);
  if (not std::isinf(total_remaining_time) and not predicted_state_opt.has_value()) {
    THROW_SIMULATION_ERROR(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: FollowWaypointController for vehicle ",
      std::quoted(validated_status.name),
      " calculated invalid acceleration:", " desired_acceleration: ", desired_acceleration,
      ", total_remaining_time: ", total_remaining_time,
      ", total_remining_distance: ", total_remining_distance,
      ", acceleration: ", validated_status.linear_acceleration,
      ", speed: ", validated_status.linear_speed, ". ", follow_waypoint_controller);
  }
}

auto PolylineTrajectoryFollowerStep::distanceAlongLanelet(
  const Point & from, const Point & to, const double matching_distance,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr) const -> double
{
  if (const auto from_lanelet_pose = hdmap_utils_ptr->toLaneletPose(
        from, validated_status.bounding_box, false, matching_distance);
      from_lanelet_pose.has_value()) {
    if (const auto to_lanelet_pose = hdmap_utils_ptr->toLaneletPose(
          to, validated_status.bounding_box, false, matching_distance);
        to_lanelet_pose.has_value()) {
      if (const auto distance = hdmap_utils_ptr->getLongitudinalDistance(
            from_lanelet_pose.value(), to_lanelet_pose.value());
          distance.has_value()) {
        return distance.value();
      }
    }
  }
  return math::geometry::hypot(from, to);
}
}  // namespace follow_trajectory
}  // namespace traffic_simulator
