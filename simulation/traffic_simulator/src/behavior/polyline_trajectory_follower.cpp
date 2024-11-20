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

PolylineTrajectoryFollower::PolylineTrajectoryFollower(
  const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter,
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr, const double step_time)
: entity_status(entity_status),
  behavior_parameter(behavior_parameter),
  hdmap_utils_ptr(hdmap_utils_ptr),
  step_time(step_time)
{
}

template <typename T>
auto isfinite_vec3(const T & vec) -> bool
{
  static_assert(math::geometry::IsLikeVector3<std::decay_t<decltype(vec)>>::value);
  return std::isfinite(vec.x) and std::isfinite(vec.y) and std::isfinite(vec.z);
}

auto distance_along_lanelet(
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr,
  const traffic_simulator_msgs::msg::EntityStatus & entity_status, const double matching_distance,
  const geometry_msgs::msg::Point & from, const geometry_msgs::msg::Point & to) -> double
{
  if (const auto from_lanelet_pose =
        hdmap_utils_ptr->toLaneletPose(from, entity_status.bounding_box, false, matching_distance);
      from_lanelet_pose.has_value()) {
    if (const auto to_lanelet_pose =
          hdmap_utils_ptr->toLaneletPose(to, entity_status.bounding_box, false, matching_distance);
        to_lanelet_pose.has_value()) {
      if (const auto distance = hdmap_utils_ptr->getLongitudinalDistance(
            from_lanelet_pose.value(), to_lanelet_pose.value());
          distance.has_value()) {
        return distance.value();
      }
    }
  }
  return math::geometry::hypot(from, to);
};

auto first_waypoint_with_arrival_time_specified(
  const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory)
  -> std::vector<traffic_simulator_msgs::msg::Vertex>::const_iterator
{
  return std::find_if(
    polyline_trajectory.shape.vertices.cbegin(), polyline_trajectory.shape.vertices.cend(),
    [](const auto & vertex) { return not std::isnan(vertex.time); });
};

auto make_updated_pose_orientation(
  const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  const geometry_msgs::msg::Vector3 & desired_velocity) -> geometry_msgs::msg::Quaternion
{
  if (desired_velocity.y == 0.0 && desired_velocity.x == 0.0 && desired_velocity.z == 0.0) {
    // do not change orientation if there is no designed_velocity vector
    return entity_status.pose.orientation;
  } else {
    // if there is a designed_velocity vector, set the orientation in the direction of it
    const geometry_msgs::msg::Vector3 direction =
      geometry_msgs::build<geometry_msgs::msg::Vector3>()
        .x(0.0)
        .y(std::atan2(-desired_velocity.z, std::hypot(desired_velocity.x, desired_velocity.y)))
        .z(std::atan2(desired_velocity.y, desired_velocity.x));
    return math::geometry::convertEulerAngleToQuaternion(direction);
  }
}

auto calculate_current_velocity(
  const traffic_simulator_msgs::msg::EntityStatus & entity_status, const double speed)
  -> geometry_msgs::msg::Vector3
{
  const auto euler_angles =
    math::geometry::convertQuaternionToEulerAngle(entity_status.pose.orientation);
  const double pitch = -euler_angles.y;
  const double yaw = euler_angles.z;
  return geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(std::cos(pitch) * std::cos(yaw) * speed)
    .y(std::cos(pitch) * std::sin(yaw) * speed)
    .z(std::sin(pitch) * speed);
}

auto calculate_distance_and_remaining_time(
  const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr,
  const traffic_simulator_msgs::msg::EntityStatus & entity_status, const double matching_distance,
  const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
  const double distance_to_front_waypoint, const double step_time) -> std::tuple<double, double>
{
  /*
    Note for anyone working on adding support for followingMode follow
    to this function (FollowPolylineTrajectoryAction::tick) in the
    future: if followingMode is follow, this distance calculation may be
    inappropriate.
  */
  const auto total_distance_to =
    [&hdmap_utils_ptr, &entity_status, &matching_distance, &polyline_trajectory](
      const std::vector<traffic_simulator_msgs::msg::Vertex>::const_iterator last) {
      return std::accumulate(
        polyline_trajectory.shape.vertices.cbegin(), last, 0.0,
        [&hdmap_utils_ptr, &entity_status, &matching_distance](
          const double total_distance, const auto & vertex) {
          const auto next = std::next(&vertex);
          return total_distance + distance_along_lanelet(
                                    hdmap_utils_ptr, entity_status, matching_distance,
                                    vertex.position.position, next->position.position);
        });
    };

  const auto waypoint_ptr = first_waypoint_with_arrival_time_specified(polyline_trajectory);
  if (waypoint_ptr == std::cend(polyline_trajectory.shape.vertices)) {
    return std::make_tuple(
      distance_to_front_waypoint +
        total_distance_to(std::cend(polyline_trajectory.shape.vertices) - 1),
      std::numeric_limits<double>::infinity());
  }

  const auto remaining_time =
    (not std::isnan(polyline_trajectory.base_time) ? polyline_trajectory.base_time : 0.0) +
    waypoint_ptr->time - entity_status.time;

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
    throw common::Error(
      "Vehicle ", std::quoted(entity_status.name),
      " failed to reach the trajectory waypoint at the specified time. The specified time "
      "is ",
      waypoint_ptr->time, " (in ",
      (not std::isnan(polyline_trajectory.base_time) ? "absolute" : "relative"),
      " simulation time). This may be due to unrealistic conditions of arrival time "
      "specification compared to vehicle parameters and dynamic constraints.");

  } else {
    return std::make_tuple(
      distance_to_front_waypoint + total_distance_to(waypoint_ptr),
      remaining_time == 0.0 ? std::numeric_limits<double>::epsilon() : remaining_time);
  }
}

auto PolylineTrajectoryFollower::getValidatedEntityDesiredVelocity(
  const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
  const geometry_msgs::msg::Point & target_position, const geometry_msgs::msg::Point & position,
  const double desired_speed) const noexcept(false) -> geometry_msgs::msg::Vector3
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

  const double dx = target_position.x - position.x;
  const double dy = target_position.y - position.y;
  // if entity is on lane use pitch from lanelet, otherwise use pitch on target
  const double pitch =
    entity_status.lanelet_pose_valid
      ? -math::geometry::convertQuaternionToEulerAngle(entity_status.pose.orientation).y
      : std::atan2(target_position.z - position.z, std::hypot(dy, dx));
  const double yaw = std::atan2(dy, dx);  // Use yaw on target

  const auto desired_velocity = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                                  .x(std::cos(pitch) * std::cos(yaw) * desired_speed)
                                  .y(std::cos(pitch) * std::sin(yaw) * desired_speed)
                                  .z(std::sin(pitch) * desired_speed);
  if (not isfinite_vec3(desired_velocity)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name),
      "'s desired velocity contains NaN or infinity. The value is [", desired_velocity.x, ", ",
      desired_velocity.y, ", ", desired_velocity.z, "].");
  }
  return desired_velocity;
}

auto PolylineTrajectoryFollower::getValidatedEntityDesiredAcceleration(
  const traffic_simulator::follow_trajectory::FollowWaypointController & follow_waypoint_controller,
  const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
  const double remaining_time, const double distance, const double acceleration,
  const double speed) const noexcept(false) -> double
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
    const double desired_acceleration =
      follow_waypoint_controller.getAcceleration(remaining_time, distance, acceleration, speed);

    if (not std::isfinite(desired_acceleration)) {
      throw common::Error(
        "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
        "following information to the developer: Vehicle ",
        std::quoted(entity_status.name),
        "'s desired acceleration value contains NaN or infinity. The value is ",
        desired_acceleration, ". ");
    }
    return desired_acceleration;
  } catch (const ControllerError & e) {
    throw common::Error(
      "Vehicle ", std::quoted(entity_status.name), " - controller operation problem encountered. ",
      follow_waypoint_controller.getFollowedWaypointDetails(polyline_trajectory), e.what());
  }
}

auto PolylineTrajectoryFollower::discardTheFrontWaypointAndRecurse(
  const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
  const double matching_distance, const std::optional<double> target_speed) const
  -> std::optional<EntityStatus>
{
  if (polyline_trajectory.shape.vertices.empty()) {
    throw common::SimulationError(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: ",
      "Attempted to access an element of an empty vector");
  }
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
  auto polyline_trajectory_copy = polyline_trajectory;
  if (
    not std::isnan(polyline_trajectory_copy.base_time) and
    not std::isnan(polyline_trajectory_copy.shape.vertices.front().time)) {
    polyline_trajectory_copy.base_time = entity_status.time;
  }

  std::rotate(
    std::begin(polyline_trajectory_copy.shape.vertices),
    std::begin(polyline_trajectory_copy.shape.vertices) + 1,
    std::end(polyline_trajectory_copy.shape.vertices));

  if (not polyline_trajectory_copy.closed) {
    polyline_trajectory_copy.shape.vertices.pop_back();
  }

  return makeUpdatedEntityStatus(polyline_trajectory_copy, matching_distance, target_speed);
};

auto PolylineTrajectoryFollower::buildUpdatedEntityStatus(
  const geometry_msgs::msg::Vector3 & desired_velocity) const noexcept(true) -> EntityStatus
{
  using math::geometry::operator+;
  using math::geometry::operator-;
  using math::geometry::operator*;
  using math::geometry::operator/;

  const auto updated_pose_orientation =
    make_updated_pose_orientation(entity_status, desired_velocity);
  const auto updated_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                              .position(entity_status.pose.position + desired_velocity * step_time)
                              .orientation(updated_pose_orientation);

  const auto updated_action_status_twist_linear =
    geometry_msgs::build<geometry_msgs::msg::Vector3>()
      .x(math::geometry::norm(desired_velocity))
      .y(0.0)
      .z(0.0);
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

auto PolylineTrajectoryFollower::makeUpdatedEntityStatus(
  const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory,
  const double matching_distance, const std::optional<double> target_speed /*= std::nullopt*/) const
  -> std::optional<EntityStatus>
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

  if (step_time <= 0.0) {
    throw common::SimulationError(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: ",
      "non-positive step time provided");
  }

  if (polyline_trajectory.shape.vertices.empty()) {
    return std::nullopt;
  }

  const auto entity_position = getValidatedEntityPosition();
  const auto target_position = getValidatedEntityTargetPosition(polyline_trajectory);

  const double distance_to_front_waypoint = distance_along_lanelet(
    hdmap_utils_ptr, entity_status, matching_distance, entity_position, target_position);
  const double remaining_time_to_front_waypoint =
    (std::isnan(polyline_trajectory.base_time) ? 0.0 : polyline_trajectory.base_time) +
    polyline_trajectory.shape.vertices.front().time - entity_status.time;

  /*
    This clause is to avoid division-by-zero errors in later clauses with
    distance_to_front_waypoint as the denominator if the distance
    miraculously becomes zero.
  */
  if (math::arithmetic::isDefinitelyLessThan(
        distance_to_front_waypoint, std::numeric_limits<double>::epsilon())) {
    return discardTheFrontWaypointAndRecurse(polyline_trajectory, matching_distance, target_speed);
  }
  const auto [distance, remaining_time] = calculate_distance_and_remaining_time(
    hdmap_utils_ptr, entity_status, matching_distance, polyline_trajectory,
    distance_to_front_waypoint, step_time);

  if (math::arithmetic::isDefinitelyLessThan(distance, std::numeric_limits<double>::epsilon())) {
    return discardTheFrontWaypointAndRecurse(polyline_trajectory, matching_distance, target_speed);
  }

  const double acceleration = getValidatedEntityAcceleration();
  [[maybe_unused]] const double max_acceleration = getValidatedEntityMaxAcceleration(acceleration);
  [[maybe_unused]] const double min_acceleration = getValidatedEntityMinAcceleration(acceleration);
  const double entity_speed = getValidatedEntitySpeed();

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
  const bool is_breaking_waypoint =
    first_waypoint_with_arrival_time_specified(polyline_trajectory) >=
    std::prev(polyline_trajectory.shape.vertices.cend());
  const auto follow_waypoint_controller = FollowWaypointController(
    behavior_parameter, step_time, is_breaking_waypoint,
    std::isinf(remaining_time) ? target_speed : std::nullopt);

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
  const double desired_acceleration = getValidatedEntityDesiredAcceleration(
    follow_waypoint_controller, polyline_trajectory, remaining_time, distance, acceleration,
    entity_speed);
  const double desired_speed = getValidatedEntityDesiredSpeed(entity_speed, desired_acceleration);
  const auto desired_velocity = getValidatedEntityDesiredVelocity(
    polyline_trajectory, target_position, entity_position, desired_speed);
  const auto current_velocity = calculate_current_velocity(entity_status, entity_speed);

  if (
    entity_speed * step_time > distance_to_front_waypoint &&
    math::geometry::innerProduct(desired_velocity, current_velocity) < 0.0) {
    return discardTheFrontWaypointAndRecurse(polyline_trajectory, matching_distance, target_speed);
  }

  const auto predicted_state_opt = follow_waypoint_controller.getPredictedWaypointArrivalState(
    desired_acceleration, remaining_time, distance, acceleration, entity_speed);
  if constexpr (true) {
    auto remaining_time_to_arrival_to_front_waypoint = predicted_state_opt->travel_time;
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
                << "== 2 * " << distance << " / " << std::pow(remaining_time, 2) << " - 2 * " << entity_speed << " / " << remaining_time << " "
                << "== " << (2 * distance / std::pow(remaining_time, 2)) << " - " << (2 * entity_speed / remaining_time) << " "
                << "== " << desired_acceleration << " "
                << "(acceleration < desired_acceleration == " << (acceleration < desired_acceleration) << " == need to " <<(acceleration < desired_acceleration ? "accelerate" : "decelerate") << ")"
                << std::endl;

      std::cout << "desired_speed "
                << "== speed + std::clamp(desired_acceleration, min_acceleration, max_acceleration) * step_time "
                << "== " << entity_speed << " + std::clamp(" << desired_acceleration << ", " << min_acceleration << ", " << max_acceleration << ") * " << step_time << " "
                << "== " << entity_speed << " + " << std::clamp(desired_acceleration, min_acceleration, max_acceleration) << " * " << step_time << " "
                << "== " << entity_speed << " + " << std::clamp(desired_acceleration, min_acceleration, max_acceleration) * step_time << " "
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
                << "== " << math::arithmetic::isDefinitelyLessThan(remaining_time_to_arrival_to_front_waypoint, step_time)
                << std::endl;

      std::cout << "not too early? "
                << "== std::isnan(remaining_time_to_front_waypoint) or remaining_time_to_front_waypoint < remaining_time_to_arrival_to_front_waypoint + step_time "
                << "== std::isnan(" << remaining_time_to_front_waypoint << ") or " << remaining_time_to_front_waypoint << " < " << remaining_time_to_arrival_to_front_waypoint << " + " << step_time << " "
                << "== " << std::isnan(remaining_time_to_front_waypoint) << " or " << math::arithmetic::isDefinitelyLessThan(remaining_time_to_front_waypoint, remaining_time_to_arrival_to_front_waypoint + step_time) << " "
                << "== " << (std::isnan(remaining_time_to_front_waypoint) or math::arithmetic::isDefinitelyLessThan(remaining_time_to_front_waypoint, remaining_time_to_arrival_to_front_waypoint + step_time))
                << std::endl;
    // clang-format on
  }
  if (not std::isinf(remaining_time) and not predicted_state_opt.has_value()) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: FollowWaypointController for vehicle ",
      std::quoted(entity_status.name),
      " calculated invalid acceleration:", " desired_acceleration: ", desired_acceleration,
      ", remaining_time_to_front_waypoint: ", remaining_time_to_front_waypoint,
      ", distance: ", distance, ", acceleration: ", acceleration, ", speed: ", entity_speed, ". ",
      follow_waypoint_controller);
  }

  if (std::isnan(remaining_time_to_front_waypoint)) {
    /*
      If the nearest waypoint is arrived at in this step without a specific arrival time, it will
      be considered as achieved
    */
    if (std::isinf(remaining_time) && polyline_trajectory.shape.vertices.size() == 1UL) {
      /*
        If the trajectory has only waypoints with unspecified time, the last one is followed using
        maximum speed including braking - in this case accuracy of arrival is checked
      */
      if (follow_waypoint_controller.areConditionsOfArrivalMet(
            acceleration, entity_speed, distance_to_front_waypoint)) {
        return discardTheFrontWaypointAndRecurse(
          polyline_trajectory, matching_distance, target_speed);
      } else {
        return buildUpdatedEntityStatus(desired_velocity);
      }
    } else {
      /*
        If it is an intermediate waypoint with an unspecified time, the accuracy of the arrival is
        irrelevant
      */
      if (const double this_step_distance =
            (entity_speed + desired_acceleration * step_time) * step_time;
          this_step_distance > distance_to_front_waypoint) {
        return discardTheFrontWaypointAndRecurse(
          polyline_trajectory, matching_distance, target_speed);
      } else {
        return buildUpdatedEntityStatus(desired_velocity);
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
  } else if (math::arithmetic::isDefinitelyLessThan(
               remaining_time_to_front_waypoint, step_time / 2.0)) {
    if (follow_waypoint_controller.areConditionsOfArrivalMet(
          acceleration, entity_speed, distance_to_front_waypoint)) {
      return discardTheFrontWaypointAndRecurse(
        polyline_trajectory, matching_distance, target_speed);
    } else {
      throw common::SimulationError(
        "Vehicle ", std::quoted(entity_status.name), " at time ", entity_status.time,
        "s (remaining time is ", remaining_time_to_front_waypoint,
        "s), has completed a trajectory to the nearest waypoint with", " specified time equal to ",
        polyline_trajectory.shape.vertices.front().time, "s at a distance equal to ", distance,
        " from that waypoint which is greater than the accepted accuracy.");
    }
  } else {
    return buildUpdatedEntityStatus(desired_velocity);
  }

  /*
    Note: If obstacle avoidance is to be implemented, the steering behavior
    known by the name "collision avoidance" should be synthesized here into
    steering.
  */
}

auto PolylineTrajectoryFollower::getValidatedEntityAcceleration() const noexcept(false) -> double
{
  const double acceleration = entity_status.action_status.accel.linear.x;
  if (not std::isfinite(acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name), "'s acceleration value is NaN or infinity. The value is ",
      acceleration, ". ");
  }
  return acceleration;
}

auto PolylineTrajectoryFollower::getValidatedEntitySpeed() const noexcept(false) -> double
{
  const double entity_speed = entity_status.action_status.twist.linear.x;  // [m/s]

  if (not std::isfinite(entity_speed)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name), "'s speed value is NaN or infinity. The value is ",
      entity_speed, ". ");
  }
  return entity_speed;
}
auto PolylineTrajectoryFollower::getValidatedEntityMaxAcceleration(const double acceleration) const
  noexcept(false) -> double
{
  const double max_acceleration = std::min(
    acceleration /* [m/s^2] */ +
      behavior_parameter.dynamic_constraints.max_acceleration_rate /* [m/s^3] */ *
        step_time /* [s] */,
    +behavior_parameter.dynamic_constraints.max_acceleration /* [m/s^2] */);

  if (not std::isfinite(max_acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name),
      "'s maximum acceleration value is NaN or infinity. The value is ", max_acceleration, ". ");
  }
  return max_acceleration;
}
auto PolylineTrajectoryFollower::getValidatedEntityMinAcceleration(const double acceleration) const
  noexcept(false) -> double
{
  const double min_acceleration = std::max(
    acceleration /* [m/s^2] */ -
      behavior_parameter.dynamic_constraints.max_deceleration_rate /* [m/s^3] */ *
        step_time /* [s] */,
    -behavior_parameter.dynamic_constraints.max_deceleration /* [m/s^2] */);

  if (not std::isfinite(min_acceleration)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name),
      "'s minimum acceleration value is NaN or infinity. The value is ", min_acceleration, ". ");
  }
  return min_acceleration;
}
auto PolylineTrajectoryFollower::getValidatedEntityPosition() const noexcept(false)
  -> geometry_msgs::msg::Point
{
  const auto entity_position = entity_status.pose.position;
  if (not isfinite_vec3(entity_position)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name), " coordinate value contains NaN or infinity. The value is [",
      entity_position.x, ", ", entity_position.y, ", ", entity_position.z, "].");
  }
  return entity_position;
}
auto PolylineTrajectoryFollower::getValidatedEntityTargetPosition(
  const traffic_simulator_msgs::msg::PolylineTrajectory & polyline_trajectory) const noexcept(false)
  -> geometry_msgs::msg::Point
{
  if (polyline_trajectory.shape.vertices.empty()) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "attempted to dereference an element of an empty PolylineTrajectory");
  }
  const auto target_position = polyline_trajectory.shape.vertices.front().position.position;
  if (not isfinite_vec3(target_position)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name),
      "'s target position coordinate value contains NaN or infinity. The value is [",
      target_position.x, ", ", target_position.y, ", ", target_position.z, "].");
  }
  return target_position;
}
auto PolylineTrajectoryFollower::getValidatedEntityDesiredSpeed(
  const double entity_speed, const double desired_acceleration) const noexcept(false) -> double
{
  const double desired_speed = entity_speed + desired_acceleration * step_time;

  if (not std::isfinite(desired_speed)) {
    throw common::Error(
      "An error occurred in the internal state of FollowTrajectoryAction. Please report the "
      "following information to the developer: Vehicle ",
      std::quoted(entity_status.name), "'s desired speed value is NaN or infinity. The value is ",
      desired_speed, ". ");
  }
  return desired_speed;
}

}  // namespace follow_trajectory
}  // namespace traffic_simulator
