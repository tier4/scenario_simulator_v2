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

#include <geometry/quaternion/direction_to_quaternion.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/quaternion/get_rotation.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <geometry/vector3/hypot.hpp>
#include <geometry/vector3/inner_product.hpp>
#include <geometry/vector3/is_finite.hpp>
#include <geometry/vector3/norm.hpp>
#include <geometry/vector3/operator.hpp>
#include <geometry/vector3/rotate.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/behavior/follow_trajectory/validated_entity_status.hpp>
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
  velocity_(math::geometry::rotate(
    entity_status_.action_status.twist.linear, entity_status_.pose.orientation))
{
  /// @todo add validate time, add validate orientation
  validateBehaviorParameter(behaviorParameter());
  validatePosition(position());
  validateLinearSpeed(linearSpeed());
  validateVelocity(velocity());
  validateLinearAcceleration(linearAcceleration(), behaviorParameter(), step_time_);
}

ValidatedEntityStatus::ValidatedEntityStatus(const ValidatedEntityStatus & other)
: ValidatedEntityStatus(other.entity_status_, other.behavior_parameter_, other.step_time_)
{
}

/// @brief this method updates the EntityStatus by applying a local velocity vector,
/// during the update the position on the lanelet is lost (it is set as invalid)
auto ValidatedEntityStatus::buildUpdatedEntityStatus(
  const geometry_msgs::msg::Vector3 & desired_local_velocity) const
  -> traffic_simulator_msgs::msg::EntityStatus
{
  using math::geometry::operator+;
  using math::geometry::operator-;
  using math::geometry::operator*;
  using math::geometry::operator/;

  const auto updated_time = entity_status_.time + step_time_;

  const auto updated_pose_position =
    entity_status_.pose.position + desired_local_velocity * step_time_;
  const auto updated_pose_orientation =
    desired_local_velocity == geometry_msgs::msg::Vector3()
      ? entity_status_.pose.orientation
      : math::geometry::convertDirectionToQuaternion(desired_local_velocity);
  const auto updated_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                              .position(updated_pose_position)
                              .orientation(updated_pose_orientation);

  const auto updated_twist_linear = geometry_msgs::build<geometry_msgs::msg::Vector3>()
                                      .x(math::geometry::norm(desired_local_velocity))
                                      .y(0.0)
                                      .z(0.0);
  const auto updated_twist_angular =
    math::geometry::convertQuaternionToEulerAngle(
      math::geometry::getRotation(entity_status_.pose.orientation, updated_pose_orientation)) /
    step_time_;
  const auto updated_twist = geometry_msgs::build<geometry_msgs::msg::Twist>()
                               .linear(updated_twist_linear)
                               .angular(updated_twist_angular);

  const auto updated_accel =
    geometry_msgs::build<geometry_msgs::msg::Accel>()
      .linear((updated_twist_linear - entity_status_.action_status.twist.linear) / step_time_)
      .angular((updated_twist_angular - entity_status_.action_status.twist.angular) / step_time_);

  const auto updated_linear_jerk =
    (updated_accel.linear.x - entity_status_.action_status.accel.linear.x) / step_time_;

  const auto updated_action_status =
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::ActionStatus>()
      .current_action(entity_status_.action_status.current_action)
      .twist(updated_twist)
      .accel(updated_accel)
      .linear_jerk(updated_linear_jerk);

  return traffic_simulator_msgs::build<traffic_simulator_msgs::msg::EntityStatus>()
    .type(entity_status_.type)
    .subtype(entity_status_.subtype)
    .time(updated_time)
    .name(entity_status_.name)
    .bounding_box(entity_status_.bounding_box)
    .action_status(updated_action_status)
    .pose(updated_pose)
    .lanelet_pose(traffic_simulator_msgs::msg::LaneletPose())
    .lanelet_pose_valid(false);
}

auto ValidatedEntityStatus::validatePosition(const geometry_msgs::msg::Point & position) const
  noexcept(false) -> void
{
  if (not math::geometry::isFinite(position)) {
    throwDetailedValidationError("position", position);
  }
}

auto ValidatedEntityStatus::validateLinearSpeed(const double speed) const noexcept(false) -> void
{
  if (not std::isfinite(speed)) {
    throwDetailedValidationError("speed", speed);
  }
}

auto ValidatedEntityStatus::validateVelocity(const geometry_msgs::msg::Vector3 & velocity) const
  noexcept(false) -> void
{
  if (not math::geometry::isFinite(velocity)) {
    throwDetailedValidationError("velocity", velocity);
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
