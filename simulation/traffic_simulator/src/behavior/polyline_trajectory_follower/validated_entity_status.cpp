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

#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/inner_product.hpp>
#include <geometry/vector3/is_finite.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/operator.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/behavior/polyline_trajectory_follower/validated_entity_status.hpp>
#include <traffic_simulator_msgs/msg/action_status.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{

ValidatedEntityStatus::ValidatedEntityStatus(
  const traffic_simulator_msgs::msg::EntityStatus & entity_status,
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter,
  const double step_time) noexcept(false)
: step_time_(step_time),
  entity_status_(entity_status),
  behavior_parameter_(behavior_parameter),
  current_velocity_(buildValidatedCurrentVelocity(linearSpeed(), orientation()))
{
  validateBehaviorParameter(behaviorParameter());
  validatePosition(position());
  validateLinearSpeed(linearSpeed());
  validateLinearAcceleration(linearAcceleration(), behaviorParameter(), step_time_);
}

ValidatedEntityStatus::ValidatedEntityStatus(const ValidatedEntityStatus & other)
: ValidatedEntityStatus(other.entity_status_, other.behavior_parameter_, other.step_time_)
{
}

auto ValidatedEntityStatus::buildUpdatedPoseOrientation(
  const geometry_msgs::msg::Vector3 & desired_velocity) const noexcept(true)
  -> geometry_msgs::msg::Quaternion
{
  if (desired_velocity.x == 0.0 and desired_velocity.y == 0.0 and desired_velocity.z == 0.0) {
    // do not change orientation if there is no designed_velocity vector
    return orientation();
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

auto ValidatedEntityStatus::buildUpdatedEntityStatus(
  const geometry_msgs::msg::Vector3 & desired_velocity) const
  -> traffic_simulator_msgs::msg::EntityStatus
{
  using math::geometry::operator+;
  using math::geometry::operator-;
  using math::geometry::operator*;
  using math::geometry::operator/;

  const auto updated_time = entity_status_.time + step_time_;
  const auto updated_pose_orientation = buildUpdatedPoseOrientation(desired_velocity);
  const auto updated_action_status_twist_linear =
    geometry_msgs::build<geometry_msgs::msg::Vector3>()
      .x(math::geometry::norm(desired_velocity))
      .y(0.0)
      .z(0.0);
  const auto updated_action_status_twist_angular =
    math::geometry::convertQuaternionToEulerAngle(
      math::geometry::getRotation(orientation(), updated_pose_orientation)) /
    step_time_;
  const auto updated_action_status_twist = geometry_msgs::build<geometry_msgs::msg::Twist>()
                                             .linear(updated_action_status_twist_linear)
                                             .angular(updated_action_status_twist_angular);
  const auto updated_action_status_accel =
    geometry_msgs::build<geometry_msgs::msg::Accel>()
      .linear(
        (updated_action_status_twist_linear - entity_status_.action_status.twist.linear) /
        step_time_)
      .angular(
        (updated_action_status_twist_angular - entity_status_.action_status.twist.angular) /
        step_time_);
  const auto updated_action_status =
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::ActionStatus>()
      .current_action(entity_status_.action_status.current_action)
      .twist(updated_action_status_twist)
      .accel(updated_action_status_accel)
      .linear_jerk(entity_status_.action_status.linear_jerk);
  const auto updated_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                              .position(position() + desired_velocity * step_time_)
                              .orientation(updated_pose_orientation);
  constexpr bool updated_lanelet_pose_valid = false;

  return traffic_simulator_msgs::build<traffic_simulator_msgs::msg::EntityStatus>()
    .type(entity_status_.type)
    .subtype(entity_status_.subtype)
    .time(updated_time)
    .name(entity_status_.name)
    .bounding_box(entity_status_.bounding_box)
    .action_status(updated_action_status)
    .pose(updated_pose)
    .lanelet_pose(entity_status_.lanelet_pose)
    .lanelet_pose_valid(updated_lanelet_pose_valid);
}

auto ValidatedEntityStatus::validatePosition(
  const geometry_msgs::msg::Point & entity_position) const noexcept(false) -> void
{
  if (not math::geometry::isFinite(entity_position)) {
    throwDetailedValidationError("entity_position", entity_position);
  }
}

auto ValidatedEntityStatus::validateLinearSpeed(const double entity_speed) const noexcept(false)
  -> void
{
  if (not std::isfinite(entity_speed)) {
    throwDetailedValidationError("entity_speed", entity_speed);
  }
}

auto ValidatedEntityStatus::validateLinearAcceleration(
  const double acceleration,
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter,
  const double step_time) const noexcept(false) -> void
{
  const double max_acceleration = std::min(
    acceleration + behavior_parameter.dynamic_constraints.max_acceleration_rate * step_time,
    +behavior_parameter.dynamic_constraints.max_acceleration);
  const double min_acceleration = std::max(
    acceleration - behavior_parameter.dynamic_constraints.max_deceleration_rate * step_time,
    -behavior_parameter.dynamic_constraints.max_deceleration);
  if (not std::isfinite(acceleration)) {
    throwDetailedValidationError("acceleration", acceleration);
  } else if (not std::isfinite(max_acceleration)) {
    throwDetailedValidationError("maximum acceleration", max_acceleration);
  } else if (not std::isfinite(min_acceleration)) {
    throwDetailedValidationError("minimum acceleration", min_acceleration);
  }
}

auto ValidatedEntityStatus::buildValidatedCurrentVelocity(
  const double speed, const geometry_msgs::msg::Quaternion & entity_orientation) const
  noexcept(false) -> geometry_msgs::msg::Vector3
{
  const auto euler_angles = math::geometry::convertQuaternionToEulerAngle(entity_orientation);
  const double pitch = -euler_angles.y;
  const double yaw = euler_angles.z;
  const auto entity_velocity = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                                 .x(std::cos(pitch) * std::cos(yaw) * speed)
                                 .y(std::cos(pitch) * std::sin(yaw) * speed)
                                 .z(std::sin(pitch) * speed);
  if (not math::geometry::isFinite(entity_velocity)) {
    throwDetailedValidationError("entity_velocity", entity_velocity);
  }
  return entity_velocity;
}

auto ValidatedEntityStatus::validateBehaviorParameter(
  const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter) const noexcept(false)
  -> void
{
  if (not std::isfinite(behavior_parameter.dynamic_constraints.max_acceleration_rate)) {
    throwDetailedValidationError(
      "behavior_parameter.dynamic_constraints.max_acceleration_rate",
      behavior_parameter.dynamic_constraints.max_acceleration_rate);
  }
  if (not std::isfinite(behavior_parameter.dynamic_constraints.max_deceleration_rate)) {
    throwDetailedValidationError(
      "behavior_parameter.dynamic_constraints.max_acceleration_rate",
      behavior_parameter.dynamic_constraints.max_acceleration_rate);
  }
  if (not std::isfinite(behavior_parameter.dynamic_constraints.max_acceleration)) {
    throwDetailedValidationError(
      "behavior_parameter.dynamic_constraints.max_acceleration_rate",
      behavior_parameter.dynamic_constraints.max_acceleration_rate);
  }
  if (not std::isfinite(behavior_parameter.dynamic_constraints.max_deceleration)) {
    throwDetailedValidationError(
      "behavior_parameter.dynamic_constraints.max_acceleration_rate",
      behavior_parameter.dynamic_constraints.max_acceleration_rate);
  }
}
}  // namespace follow_trajectory
}  // namespace traffic_simulator
