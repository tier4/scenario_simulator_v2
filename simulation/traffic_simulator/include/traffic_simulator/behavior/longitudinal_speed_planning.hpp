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
  explicit LongitudinalSpeedPlanner(double step_time, const std::string & entity);
  std::tuple<geometry_msgs::msg::Twist, geometry_msgs::msg::Accel, double> getDynamicStates(
    double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints &,
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel) const;
  double getAccelerationDuration(
    double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints &,
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel) const;
  auto planConstraintsFromJerkAndTimeConstraint(
    double target_speed, const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel, double acceleration_duration,
    const traffic_simulator_msgs::msg::DynamicConstraints & constraints)
    -> traffic_simulator_msgs::msg::DynamicConstraints;
  bool isAccelerating(double target_speed, const geometry_msgs::msg::Twist & current_twist) const;
  bool isDecelerating(double target_speed, const geometry_msgs::msg::Twist & current_twist) const;
  bool isTargetSpeedReached(
    double target_speed, const geometry_msgs::msg::Twist & current_twist,
    double torelance = 0.01) const;
  const double step_time;
  const std::string entity;

private:
  bool isReachedToTargetSpeedWithConstantJerk(
    double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints &,
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel, double duration,
    double torelance = 0.01) const;
  double getVelocityWithConstantJerk(
    double target_speed, const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel,
    const traffic_simulator_msgs::msg::DynamicConstraints &, double duration) const;
  double getVelocityWithConstantJerk(
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel, double linear_jerk, double duration) const;
  double getQuadraticAccelerationDuration(
    double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints &,
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel) const;
  double getLinearAccelerationDuration(
    double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints &,
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel) const;
  double getQuadraticAccelerationDurationWithConstantJerk(
    double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints &,
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel) const;
  double planLinearJerk(
    double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints &,
    const geometry_msgs::msg::Twist & current_twist,
    const geometry_msgs::msg::Accel & current_accel) const;
  geometry_msgs::msg::Accel forward(
    double linear_jerk, const geometry_msgs::msg::Accel &,
    const traffic_simulator_msgs::msg::DynamicConstraints &) const;
  geometry_msgs::msg::Twist forward(
    const geometry_msgs::msg::Accel &, const geometry_msgs::msg::Twist &,
    const traffic_simulator_msgs::msg::DynamicConstraints &) const;
  geometry_msgs::msg::Accel timeDerivative(
    const geometry_msgs::msg::Twist & before, const geometry_msgs::msg::Twist & after) const;
  double timeDerivative(
    const geometry_msgs::msg::Accel & before, const geometry_msgs::msg::Accel & after) const;
};

}  // namespace longitudinal_speed_planning
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__LONGITUDINAL_SPEED_PLANNING_HPP_
