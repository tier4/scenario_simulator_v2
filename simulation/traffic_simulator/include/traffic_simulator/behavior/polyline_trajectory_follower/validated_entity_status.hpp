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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER__VALIDATED_ENTITY_STATUS_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER__VALIDATED_ENTITY_STATUS_HPP_

#include <geometry/vector3/is_like_vector3.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>

namespace traffic_simulator
{
namespace follow_trajectory
{
struct ValidatedEntityStatus
{
public:
  explicit ValidatedEntityStatus(
    const traffic_simulator_msgs::msg::EntityStatus & entity_status,
    const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter,
    const double step_time);

  auto buildUpdatedEntityStatus(const geometry_msgs::msg::Vector3 & desired_velocity) const
    -> traffic_simulator_msgs::msg::EntityStatus;

  const traffic_simulator_msgs::msg::EntityStatus entity_status_;
  const std::string & name;
  const double time;
  const double step_time;
  const geometry_msgs::msg::Point position;
  const double linear_speed;
  const double linear_acceleration;
  const bool lanelet_pose_valid;
  const geometry_msgs::msg::Vector3 current_velocity;
  const traffic_simulator_msgs::msg::BoundingBox & bounding_box;
  const traffic_simulator_msgs::msg::BehaviorParameter behavior_parameter;

  ValidatedEntityStatus() = delete;
  ValidatedEntityStatus(const ValidatedEntityStatus & other);
  ValidatedEntityStatus & operator=(const ValidatedEntityStatus & other) = delete;
  ValidatedEntityStatus(ValidatedEntityStatus && other) noexcept(true) = delete;
  ValidatedEntityStatus & operator=(ValidatedEntityStatus && other) noexcept(true) = delete;
  ~ValidatedEntityStatus() = default;

private:
  auto validatedPosition() const noexcept(false) -> geometry_msgs::msg::Point;

  auto validatedLinearSpeed() const noexcept(false) -> double;

  auto validatedLinearAcceleration() const noexcept(false) -> double;

  auto buildUpdatedPoseOrientation(const geometry_msgs::msg::Vector3 & desired_velocity) const
    noexcept(true) -> geometry_msgs::msg::Quaternion;

  auto buildValidatedCurrentVelocity(const double speed) const -> geometry_msgs::msg::Vector3;

  template <
    typename T, std::enable_if_t<math::geometry::IsLikeVector3<T>::value, std::nullptr_t> = nullptr>
  auto throwDetailedError(const std::string & variable_name, const T variable) const noexcept(false)
    -> void
  {
    THROW_SIMULATION_ERROR(
      "Error in ValidatedEntityStatus. Entity name: ", std::quoted(entity_status_.name),
      ", Variable: ", std::quoted(variable_name), ", variable contains NaN or inf value, ",
      "Values: [", variable.x, ", ", variable.y, ", ", variable.z, "].");
  }

  template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, std::nullptr_t> = nullptr>
  auto throwDetailedError(const std::string & variable_name, const T variable) const noexcept(false)
    -> void
  {
    THROW_SIMULATION_ERROR(
      "Error in ValidatedEntityStatus. Entity name: ", std::quoted(entity_status_.name),
      ", Variable: ", std::quoted(variable_name), ", variable contains NaN or inf value, ",
      "Value: ", variable);
  }
};

}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER__VALIDATED_ENTITY_STATUS_HPP_
