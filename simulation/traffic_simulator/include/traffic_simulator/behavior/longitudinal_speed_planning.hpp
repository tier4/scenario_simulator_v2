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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__LONGITUDINAL_SPEED_PLANNING_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__LONGITUDINAL_SPEED_PLANNING_HPP_

#include <traffic_simulator_msgs/msg/action_status.hpp>
#include <traffic_simulator_msgs/msg/dynamic_constraints.hpp>
#include <tuple>

namespace traffic_simulator
{
namespace longitudinal_speed_planning
{
class LongitudinalSpeedPlanner
{
public:
  explicit LongitudinalSpeedPlanner(const double step_time, const std::string & entity);
  auto getDynamicStates(
    double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints &,
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel) const
    -> std::tuple<geometry_msgs::msg::Twist, geometry_msgs::msg::Accel, double>;
  auto getAccelerationDuration(
    double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints &,
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel) const -> double;
  auto planConstraintsFromJerkAndTimeConstraint(
    double target_speed, const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel, double acceleration_duration,
    const traffic_simulator_msgs::msg::DynamicConstraints & constraints)
    -> traffic_simulator_msgs::msg::DynamicConstraints;
  auto getRunningDistance(
    double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints &,
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel, double current_linear_jerk) const -> double;
  auto isAccelerating(double target_speed, const geometry_msgs::msg::Twist & current_twist) const
    -> bool;
  auto isDecelerating(double target_speed, const geometry_msgs::msg::Twist & current_twist) const
    -> bool;
  auto isTargetSpeedReached(
    double target_speed, const geometry_msgs::msg::Twist & current_twist,
    double tolerance = 0.01) const noexcept -> bool;
  const double step_time;
  const std::string entity;

private:
  auto isReachedToTargetSpeedWithConstantJerk(
    double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints &,
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel, double duration, double tolerance = 0.01) const
    -> bool;
  auto getVelocityWithConstantJerk(
    double target_speed, const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel,
    const traffic_simulator_msgs::msg::DynamicConstraints &, double duration) const -> double;
  auto getVelocityWithConstantJerk(
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel, double linear_jerk, double duration) const
    -> double;
  auto getQuadraticAccelerationDuration(
    double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints &,
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel) const -> double;
  auto getLinearAccelerationDuration(
    double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints &,
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel) const -> double;
  auto getQuadraticAccelerationDurationWithConstantJerk(
    double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints &,
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel) const -> double;
  auto planLinearJerk(
    double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints &,
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel) const -> double;
  auto forward(
    double linear_jerk, const geometry_msgs::msg::Accel &,
    const traffic_simulator_msgs::msg::DynamicConstraints &) const -> geometry_msgs::msg::Accel;
  auto forward(
    const geometry_msgs::msg::Accel &, const geometry_msgs::msg::Twist &,
    const traffic_simulator_msgs::msg::DynamicConstraints &) const -> geometry_msgs::msg::Twist;
  auto timeDerivative(
    const geometry_msgs::msg::Twist & before, const geometry_msgs::msg::Twist & after) const
    -> geometry_msgs::msg::Accel;
  auto timeDerivative(
    const geometry_msgs::msg::Accel & before, const geometry_msgs::msg::Accel & after) const
    -> double;
};

}  // namespace longitudinal_speed_planning
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__LONGITUDINAL_SPEED_PLANNING_HPP_
