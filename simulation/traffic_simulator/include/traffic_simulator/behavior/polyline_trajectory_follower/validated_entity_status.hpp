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
    const double step_time) noexcept(false);

  auto buildUpdatedEntityStatus(const geometry_msgs::msg::Vector3 & desired_velocity) const
    -> traffic_simulator_msgs::msg::EntityStatus;

  ValidatedEntityStatus() = delete;
  ValidatedEntityStatus(const ValidatedEntityStatus & other);
  ValidatedEntityStatus & operator=(const ValidatedEntityStatus & other) = delete;
  ValidatedEntityStatus(ValidatedEntityStatus && other) noexcept(true) = delete;
  ValidatedEntityStatus & operator=(ValidatedEntityStatus && other) noexcept(true) = delete;
  ~ValidatedEntityStatus() = default;

  // clang-format off
  auto name()               const noexcept(true) -> const std::string &                                    { return entity_status_.name;                         }
  auto time()               const noexcept(true) -> double                                                 { return entity_status_.time;                         }
  auto boundingBox()        const noexcept(true) -> const traffic_simulator_msgs::msg::BoundingBox &       { return entity_status_.bounding_box;                 }
  auto laneletPoseValid()   const noexcept(true) -> bool                                                   { return entity_status_.lanelet_pose_valid;           }
  auto position()           const noexcept(true) -> const geometry_msgs::msg::Point &                      { return entity_status_.pose.position;                }
  auto orientation()        const noexcept(true) -> const geometry_msgs::msg::Quaternion &                 { return entity_status_.pose.orientation;             }
  auto linearSpeed()        const noexcept(true) -> double                                                 { return entity_status_.action_status.twist.linear.x; }
  auto linearAcceleration() const noexcept(true) -> double                                                 { return entity_status_.action_status.accel.linear.x; }
  auto behaviorParameter()  const noexcept(true) -> const traffic_simulator_msgs::msg::BehaviorParameter & { return behavior_parameter_;                         }
  auto currentVelocity()    const noexcept(true) -> const geometry_msgs::msg::Vector3 &                    { return current_velocity_;                           }
  // clang-format on

private:
  auto validatePosition(const geometry_msgs::msg::Point & entity_position) const noexcept(false)
    -> void;

  auto validateLinearSpeed(const double entity_speed) const noexcept(false) -> void;

  auto validateLinearAcceleration(
    const double acceleration,
    const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter,
    const double step_time) const noexcept(false) -> void;

  auto validateBehaviorParameter(
    const traffic_simulator_msgs::msg::BehaviorParameter & behavior_parameter) const noexcept(false)
    -> void;

  auto buildUpdatedPoseOrientation(const geometry_msgs::msg::Vector3 & desired_velocity) const
    noexcept(true) -> geometry_msgs::msg::Quaternion;

  auto buildValidatedCurrentVelocity(
    const double speed, const geometry_msgs::msg::Quaternion & entity_orientation) const
    noexcept(false) -> geometry_msgs::msg::Vector3;

  template <
    typename T, std::enable_if_t<math::geometry::IsLikeVector3<T>::value, std::nullptr_t> = nullptr>
  auto throwDetailedValidationError(const std::string & variable_name, const T variable) const
    noexcept(false) -> void
  {
    THROW_SIMULATION_ERROR(
      "Error in ValidatedEntityStatus. Entity name: ", std::quoted(entity_status_.name),
      ", Variable: ", std::quoted(variable_name), ", variable contains NaN or inf value, ",
      "Values: [", variable.x, ", ", variable.y, ", ", variable.z, "].");
  }

  template <typename T, std::enable_if_t<std::is_arithmetic_v<T>, std::nullptr_t> = nullptr>
  auto throwDetailedValidationError(const std::string & variable_name, const T variable) const
    noexcept(false) -> void
  {
    THROW_SIMULATION_ERROR(
      "Error in ValidatedEntityStatus. Entity name: ", std::quoted(entity_status_.name),
      ", Variable: ", std::quoted(variable_name), ", variable contains NaN or inf value, ",
      "Value: ", variable);
  }

  const double step_time_;
  const traffic_simulator_msgs::msg::EntityStatus entity_status_;
  const traffic_simulator_msgs::msg::BehaviorParameter behavior_parameter_;
  const geometry_msgs::msg::Vector3 current_velocity_;
};

}  // namespace follow_trajectory
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__POLYLINE_TRAJECTORY_FOLLOWER__VALIDATED_ENTITY_STATUS_HPP_
