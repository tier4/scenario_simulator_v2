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
#include <geometry/vector3/is_finite.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/operator.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <traffic_simulator/behavior/follow_trajectory/polyline_trajectory_positioner.hpp>
#include <traffic_simulator/utils/distance.hpp>
#include <traffic_simulator_msgs/msg/action_status.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{

PolylineTrajectoryPositioner::PolylineTrajectoryPositioner(
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr,
  const ValidatedEntityStatus & validated_entity_status,
  const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
  const std::optional<double> target_speed, const double matching_distance, const double step_time)
: hdmap_utils_ptr_(hdmap_utils_ptr),
  validated_entity_status_(validated_entity_status),
  polyline_trajectory_(polyline_trajectory),
  step_time_(step_time),
  matching_distance_(matching_distance),
  nearest_waypoint_with_specified_time_it_(
    nearestWaypointWithSpecifiedTimeIterator()),  // implicitly requires: polyline_trajectory_
  nearest_waypoint_position_(
    validatedEntityTargetPosition()),  // implicitly requires: polyline_trajectory_
  distance_to_nearest_waypoint_(distanceAlongLanelet(
    hdmap_utils_ptr_, validated_entity_status_.position(), validated_entity_status_.boundingBox(),
    nearest_waypoint_position_, validated_entity_status_.boundingBox(), matching_distance_)),
  total_remaining_distance_(
    totalRemainingDistance()),  // implicitly requires: polyline_trajectory_, hdmap_utils_ptr_, matching_distance_
  time_to_nearest_waypoint_(
    (std::isnan(polyline_trajectory_.base_time) ? 0.0 : polyline_trajectory_.base_time) +
    polyline_trajectory_.shape.vertices.front().time - validated_entity_status_.time()),
  total_remaining_time_(
    totalRemainingTime()),  // implicitly requires: nearest_waypoint_with_specified_time_it_, polyline_trajectory_, validated_entity_status_, step_time_
  follow_waypoint_controller_(
    validated_entity_status_.behaviorParameter(), step_time_,
    isNearestWaypointWithSpecifiedTimeSameAsLastWaypoint(),  // implicitly requires: nearest_waypoint_with_specified_time_it_, polyline_trajectory_
    std::isinf(total_remaining_time_) ? target_speed : std::nullopt)
{
}

auto PolylineTrajectoryPositioner::nearestWaypointWithSpecifiedTimeIterator() const
  -> std::vector<traffic_simulator_msgs::msg::Vertex>::const_iterator
{
  return std::find_if(
    polyline_trajectory_.shape.vertices.cbegin(), polyline_trajectory_.shape.vertices.cend(),
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
auto PolylineTrajectoryPositioner::isNearestWaypointWithSpecifiedTimeSameAsLastWaypoint() const
  -> bool
{
  return nearest_waypoint_with_specified_time_it_ >=
         std::prev(polyline_trajectory_.shape.vertices.cend());
}

auto PolylineTrajectoryPositioner::totalRemainingDistance() const -> double
{
  /*
    Note for anyone working on adding support for followingMode follow
    to this function (FollowPolylineTrajectoryAction::tick) in the
    future: if followingMode is follow, this distance calculation may be
    inappropriate.
  */
  const auto total_distance_to =
    [this](const std::vector<traffic_simulator_msgs::msg::Vertex>::const_iterator last) {
      return std::accumulate(
        polyline_trajectory_.shape.vertices.cbegin(), last, 0.0,
        [this](const double total_distance, const auto & vertex) {
          const auto next_vertex = std::next(&vertex);
          return total_distance + distanceAlongLanelet(
                                    hdmap_utils_ptr_, vertex.position.position,
                                    validated_entity_status_.boundingBox(),
                                    next_vertex->position.position,
                                    validated_entity_status_.boundingBox(), matching_distance_);
        });
    };

  if (nearest_waypoint_with_specified_time_it_ == polyline_trajectory_.shape.vertices.cend()) {
    return distance_to_nearest_waypoint_ +
           total_distance_to(std::prev(polyline_trajectory_.shape.vertices.cend()));
  } else {
    return distance_to_nearest_waypoint_ +
           total_distance_to(nearest_waypoint_with_specified_time_it_);
  }
}

auto PolylineTrajectoryPositioner::totalRemainingTime() const noexcept(false) -> double
{
  if (nearest_waypoint_with_specified_time_it_ == polyline_trajectory_.shape.vertices.cend()) {
    return std::numeric_limits<double>::infinity();
  } else {
    const double remaining_time =
      (std::isnan(polyline_trajectory_.base_time) ? 0.0 : polyline_trajectory_.base_time) +
      nearest_waypoint_with_specified_time_it_->time - validated_entity_status_.time();

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
    if (remaining_time < -step_time_) {
      THROW_SIMULATION_ERROR(
        "Vehicle ", std::quoted(validated_entity_status_.name()),
        " failed to reach the trajectory waypoint at the specified time. The specified time "
        "is ",
        nearest_waypoint_with_specified_time_it_->time, " (in ",
        (not std::isnan(polyline_trajectory_.base_time) ? "absolute" : "relative"),
        " simulation time). This may be due to unrealistic conditions of arrival time "
        "specification compared to vehicle parameters and dynamic constraints.");
    } else {
      return remaining_time == 0.0 ? std::numeric_limits<double>::epsilon() : remaining_time;
    }
  }
}

auto PolylineTrajectoryPositioner::validatedEntityDesiredVelocity(const double desired_speed) const
  noexcept(false) -> geometry_msgs::msg::Vector3
{
  /*
    If not dynamic_constraints_ignorable, the linear distance should cause
    problems.
  */

  /*
    Note: The followingMode in OpenSCENARIO is passed as
    variable dynamic_constraints_ignorable. the value of the
    variable is `followingMode == position`.
  */
  if (not polyline_trajectory_.dynamic_constraints_ignorable) {
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

  const double dx = nearest_waypoint_position_.x - validated_entity_status_.position().x;
  const double dy = nearest_waypoint_position_.y - validated_entity_status_.position().y;
  // if entity is on lane use pitch from lanelet, otherwise use pitch on target
  const double pitch =
    validated_entity_status_.isLaneletPoseValid()
      ? -math::geometry::convertQuaternionToEulerAngle(validated_entity_status_.orientation()).y
      : std::atan2(
          nearest_waypoint_position_.z - validated_entity_status_.position().z, std::hypot(dy, dx));
  const double yaw = std::atan2(dy, dx);  // Use yaw on target

  const auto desired_velocity = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                                  .x(std::cos(pitch) * std::cos(yaw) * desired_speed)
                                  .y(std::cos(pitch) * std::sin(yaw) * desired_speed)
                                  .z(std::sin(pitch) * desired_speed);
  if (not math::geometry::isFinite(desired_velocity)) {
    THROW_SIMULATION_ERROR(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(validated_entity_status_.name()),
      "'s desired velocity contains NaN or infinity. The value is [", desired_velocity.x, ", ",
      desired_velocity.y, ", ", desired_velocity.z, "].");
  }
  return desired_velocity;
}

auto PolylineTrajectoryPositioner::validatedEntityDesiredAcceleration() const noexcept(false)
  -> double
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
    const double desired_acceleration = follow_waypoint_controller_.getAcceleration(
      total_remaining_time_, total_remaining_distance_,
      validated_entity_status_.linearAcceleration(), validated_entity_status_.linearSpeed());

    if (not std::isfinite(desired_acceleration)) {
      THROW_SIMULATION_ERROR(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(validated_entity_status_.name()),
        "'s desired acceleration value contains NaN or infinity. The value is ",
        desired_acceleration, ". ");
    }
    return desired_acceleration;
  } catch (const ControllerError & e) {
    THROW_SIMULATION_ERROR(
      "Vehicle ", std::quoted(validated_entity_status_.name()),
      " - controller operation problem encountered. ",
      follow_waypoint_controller_.getFollowedWaypointDetails(polyline_trajectory_), e.what());
  }
}

auto PolylineTrajectoryPositioner::makeUpdatedEntityStatus() const -> std::optional<EntityStatus>
{
  /*
    The following code implements the steering behavior known as "seek". See
    "Steering Behaviors For Autonomous Characters" by Craig Reynolds for more
    information.

    See https://www.researchgate.net/publication/2495826_Steering_Behaviors_For_Autonomous_Characters
  */

  using math::geometry::operator+;
  using math::geometry::operator-;
  using math::geometry::operator*;
  using math::geometry::operator/;
  using math::geometry::operator+=;

  /*
    This clause is to avoid division-by-zero errors in later clauses with
    distance_to_nearest_waypoint as the denominator if the distance
    miraculously becomes zero.
  */
  if (distance_to_nearest_waypoint_ <= 0.0) {
    return std::nullopt;
  }

  if (total_remaining_distance_ <= 0) {
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
  const double desired_acceleration = validatedEntityDesiredAcceleration();
  const double desired_speed = validatedEntityDesiredSpeed(desired_acceleration);
  const auto desired_velocity = validatedEntityDesiredVelocity(desired_speed);

  if (const bool target_passed =
        validated_entity_status_.linearSpeed() * step_time_ > distance_to_nearest_waypoint_ and
        math::geometry::innerProduct(desired_velocity, validated_entity_status_.velocity()) < 0.0;
      target_passed) {
    return std::nullopt;
  }

  validatePredictedState(desired_acceleration);
  if (not std::isfinite(time_to_nearest_waypoint_)) {
    /*
      If the nearest waypoint is arrived at in this step without a specific arrival time, it will
      be considered as achieved
    */
    if (
      not std::isfinite(total_remaining_time_) and
      polyline_trajectory_.shape.vertices.size() == 1UL) {
      /*
        If the trajectory has only waypoints with unspecified time, the last one is followed using
        maximum speed including braking - in this case accuracy of arrival is checked
      */
      if (follow_waypoint_controller_.areConditionsOfArrivalMet(
            validated_entity_status_.linearAcceleration(), validated_entity_status_.linearSpeed(),
            distance_to_nearest_waypoint_)) {
        return std::nullopt;
      } else {
        return validated_entity_status_.buildUpdatedEntityStatus(desired_velocity);
      }
    } else {
      /*
        If it is an intermediate waypoint with an unspecified time, the accuracy of the arrival is
        irrelevant
      */
      if (const double this_step_distance =
            (validated_entity_status_.linearSpeed() + desired_acceleration * step_time_) *
            step_time_;
          this_step_distance > distance_to_nearest_waypoint_) {
        return std::nullopt;
      } else {
        return validated_entity_status_.buildUpdatedEntityStatus(desired_velocity);
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
  } else if (math::arithmetic::isDefinitelyLessThan(time_to_nearest_waypoint_, step_time_ / 2.0)) {
    if (follow_waypoint_controller_.areConditionsOfArrivalMet(
          validated_entity_status_.linearAcceleration(), validated_entity_status_.linearSpeed(),
          distance_to_nearest_waypoint_)) {
      return std::nullopt;
    } else {
      THROW_SIMULATION_ERROR(
        "Vehicle ", std::quoted(validated_entity_status_.name()), " at time ",
        validated_entity_status_.time(), "s (remaining time is ", time_to_nearest_waypoint_,
        "s), has completed a trajectory to the nearest waypoint with", " specified time equal to ",
        polyline_trajectory_.shape.vertices.front().time, "s at a distance equal to ",
        distance_to_nearest_waypoint_,
        " from that waypoint which is greater than the accepted accuracy.");
    }
  } else {
    return validated_entity_status_.buildUpdatedEntityStatus(desired_velocity);
  }

  /*
    Note: If obstacle avoidance is to be implemented, the steering behavior
    known by the name "collision avoidance" should be synthesized here into
    steering.
  */
}

auto PolylineTrajectoryPositioner::validatedEntityTargetPosition() const noexcept(false)
  -> geometry_msgs::msg::Point
{
  if (polyline_trajectory_.shape.vertices.empty()) {
    THROW_SIMULATION_ERROR(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "attempted to dereference an element of an empty PolylineTrajectory for vehicle ",
      std::quoted(validated_entity_status_.name()));
  }
  const auto & target_position = polyline_trajectory_.shape.vertices.front().position.position;
  if (not math::geometry::isFinite(target_position)) {
    THROW_SIMULATION_ERROR(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(validated_entity_status_.name()),
      "'s target position coordinate value contains NaN or infinity. The value is [",
      target_position.x, ", ", target_position.y, ", ", target_position.z, "].");
  }
  return target_position;
}

auto PolylineTrajectoryPositioner::validatedEntityDesiredSpeed(
  const double desired_acceleration) const noexcept(false) -> double
{
  const double desired_speed =
    validated_entity_status_.linearSpeed() + desired_acceleration * step_time_;

  if (not std::isfinite(desired_speed)) {
    THROW_SIMULATION_ERROR(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(validated_entity_status_.name()),
      "'s desired speed value is NaN or infinity. The value is ", desired_speed, ". ");
  }
  return desired_speed;
}

auto PolylineTrajectoryPositioner::validatePredictedState(const double desired_acceleration) const
  noexcept(false) -> void
{
  const auto predicted_state_opt = follow_waypoint_controller_.getPredictedWaypointArrivalState(
    desired_acceleration, total_remaining_time_, total_remaining_distance_,
    validated_entity_status_.linearAcceleration(), validated_entity_status_.linearSpeed());
  if (not std::isinf(total_remaining_time_) and not predicted_state_opt.has_value()) {
    THROW_SIMULATION_ERROR(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: FollowWaypointController for vehicle ",
      std::quoted(validated_entity_status_.name()),
      " calculated invalid acceleration:", " desired_acceleration: ", desired_acceleration,
      ", total_remaining_time: ", total_remaining_time_,
      ", total_remaining_distance: ", total_remaining_distance_,
      ", acceleration: ", validated_entity_status_.linearAcceleration(),
      ", speed: ", validated_entity_status_.linearSpeed(), ". ", follow_waypoint_controller_);
  }
}

}  // namespace follow_trajectory
}  // namespace traffic_simulator
