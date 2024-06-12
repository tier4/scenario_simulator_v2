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

#include <algorithm>
#include <geometry/vector3/operator.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/behavior/longitudinal_speed_planning.hpp>

namespace traffic_simulator
{
namespace longitudinal_speed_planning
{
LongitudinalSpeedPlanner::LongitudinalSpeedPlanner(
  const double step_time, const std::string & entity)
: step_time(step_time), entity(entity)
{
}

auto LongitudinalSpeedPlanner::getAccelerationDuration(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints,
  const geometry_msgs::msg::Twist & current_twist,
  const geometry_msgs::msg::Accel & current_accel) const -> double
{
  return getQuadraticAccelerationDuration(target_speed, constraints, current_twist, current_accel) +
         getLinearAccelerationDuration(target_speed, constraints, current_twist, current_accel);
}

auto LongitudinalSpeedPlanner::planConstraintsFromJerkAndTimeConstraint(
  double target_speed, const geometry_msgs::msg::Twist & current_twist,
  const geometry_msgs::msg::Accel & current_accel, double acceleration_duration,
  const traffic_simulator_msgs::msg::DynamicConstraints & constraints)
  -> traffic_simulator_msgs::msg::DynamicConstraints
{
  traffic_simulator_msgs::msg::DynamicConstraints ret = constraints;
  const auto quad_duration =
    getQuadraticAccelerationDuration(target_speed, constraints, current_twist, current_accel);
  if (quad_duration >= acceleration_duration) {
    ret.max_acceleration_rate =
      2 * (target_speed - current_twist.linear.x - current_accel.linear.x * acceleration_duration) /
      (acceleration_duration * acceleration_duration);
    ret.max_deceleration_rate =
      2 * (target_speed - current_twist.linear.x - current_accel.linear.x * acceleration_duration) /
      (acceleration_duration * acceleration_duration);
  } else {
    if (isAccelerating(target_speed, current_twist)) {
      ret.max_acceleration = constraints.max_acceleration_rate * acceleration_duration -
                             std::sqrt(
                               (constraints.max_acceleration_rate * acceleration_duration *
                                constraints.max_acceleration_rate * acceleration_duration) +
                               2 * constraints.max_acceleration_rate *
                                 (current_twist.linear.x +
                                  current_accel.linear.x * acceleration_duration - target_speed)) +
                             current_accel.linear.x;
    } else {
      ret.max_deceleration = -constraints.max_deceleration_rate * acceleration_duration +
                             std::sqrt(
                               (constraints.max_deceleration_rate * acceleration_duration *
                                constraints.max_deceleration_rate * acceleration_duration) -
                               2 * constraints.max_deceleration_rate *
                                 (current_twist.linear.x -
                                  current_accel.linear.x * acceleration_duration - target_speed)) +
                             current_accel.linear.x;
    }
  }
  return ret;
}

auto LongitudinalSpeedPlanner::getDynamicStates(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints,
  const geometry_msgs::msg::Twist & current_twist,
  const geometry_msgs::msg::Accel & current_accel) const
  -> std::tuple<geometry_msgs::msg::Twist, geometry_msgs::msg::Accel, double>
{
  if (target_speed > constraints.max_speed) {
    target_speed = constraints.max_speed;
  }
  double linear_jerk = planLinearJerk(target_speed, constraints, current_twist, current_accel);
  auto accel = forward(linear_jerk, current_accel, constraints);
  auto twist = forward(accel, current_twist, constraints);
  accel = timeDerivative(current_twist, twist);
  linear_jerk = timeDerivative(current_accel, accel);
  return std::make_tuple(twist, accel, linear_jerk);
}

auto LongitudinalSpeedPlanner::getRunningDistance(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints,
  const geometry_msgs::msg::Twist & current_twist, const geometry_msgs::msg::Accel & current_accel,
  double current_linear_jerk) const -> double
{
  /**
   * @brief A value of 0.01 is the allowable range for determination of target
   * speed attainment. This value was determined heuristically rather than for
   * technical reasons.
   */
  constexpr double twist_tolerance = 0.01;
  if (isTargetSpeedReached(target_speed, current_twist, twist_tolerance)) {
    return 0;
  }
  double ret = 0;
  std::tuple<geometry_msgs::msg::Twist, geometry_msgs::msg::Accel, double> next_state =
    std::make_tuple(current_twist, current_accel, current_linear_jerk);
  do {
    next_state =
      getDynamicStates(target_speed, constraints, std::get<0>(next_state), std::get<1>(next_state));
    ret = ret + std::get<0>(next_state).linear.x * step_time +
          std::get<1>(next_state).linear.x * step_time * step_time / 2.0 +
          std::get<2>(next_state) * step_time * step_time * step_time / 6.0;
  } while (!isTargetSpeedReached(target_speed, std::get<0>(next_state), twist_tolerance));
  return ret;
}

auto LongitudinalSpeedPlanner::isTargetSpeedReached(
  double target_speed, const geometry_msgs::msg::Twist & current_twist,
  double tolerance) const noexcept -> bool
{
  return std::abs(target_speed - current_twist.linear.x) <= tolerance;
}

auto LongitudinalSpeedPlanner::getVelocityWithConstantJerk(
  double target_speed, const geometry_msgs::msg::Twist & current_twist,
  const geometry_msgs::msg::Accel & current_accel,
  const traffic_simulator_msgs::msg::DynamicConstraints & constraints, double duration) const
  -> double
{
  if (isAccelerating(target_speed, current_twist)) {
    return getVelocityWithConstantJerk(
      current_twist, current_accel, constraints.max_acceleration_rate, duration);
  } else {
    return getVelocityWithConstantJerk(
      current_twist, current_accel, constraints.max_deceleration_rate, duration);
  }
}

auto LongitudinalSpeedPlanner::getVelocityWithConstantJerk(
  const geometry_msgs::msg::Twist & current_twist, const geometry_msgs::msg::Accel & current_accel,
  double linear_jerk, double duration) const -> double
{
  return current_twist.linear.x + 0.5 * linear_jerk * duration * duration +
         current_accel.linear.x * duration;
}

auto LongitudinalSpeedPlanner::getQuadraticAccelerationDurationWithConstantJerk(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints,
  const geometry_msgs::msg::Twist & current_twist,
  const geometry_msgs::msg::Accel & current_accel) const -> double
{
  if (isAccelerating(target_speed, current_twist)) {
    return (-current_accel.linear.x + std::sqrt(
                                        current_accel.linear.x * current_accel.linear.x +
                                        2 * constraints.max_acceleration_rate *
                                          (constraints.max_speed - current_twist.linear.x))) /
           constraints.max_acceleration_rate;
  } else {
    return (-current_accel.linear.x + std::sqrt(
                                        current_accel.linear.x * current_accel.linear.x +
                                        2 * constraints.max_deceleration_rate *
                                          (-constraints.max_speed - current_twist.linear.x))) /
           constraints.max_deceleration_rate;
  }
}

auto LongitudinalSpeedPlanner::getQuadraticAccelerationDuration(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints,
  const geometry_msgs::msg::Twist & current_twist,
  const geometry_msgs::msg::Accel & current_accel) const -> double
{
  if (isAccelerating(target_speed, current_twist)) {
    double duration =
      (constraints.max_acceleration - current_accel.linear.x) / constraints.max_acceleration_rate;
    double v = getVelocityWithConstantJerk(
      current_twist, current_accel, constraints.max_acceleration_rate, duration);
    // While quadratic acceleration, the entity does not reached the target speed.
    if (std::abs(v - target_speed) >= 0.01) {
      return duration;
    }
    // While quadratic acceleration, the entity reached the target speed.
    else {
      return getQuadraticAccelerationDurationWithConstantJerk(
        target_speed, constraints, current_twist, current_accel);
    }
  } else {
    double duration =
      (current_accel.linear.x - constraints.max_deceleration) / constraints.max_deceleration_rate;
    double v = getVelocityWithConstantJerk(
      current_twist, current_accel, -constraints.max_deceleration_rate, duration);
    // While quadratic acceleration, the entity does not reached the target speed.
    if (std::abs(v - target_speed) >= 0.01) {
      return duration;
    }
    // While quadratic acceleration, the entity reached the target speed.
    else {
      return getQuadraticAccelerationDurationWithConstantJerk(
        target_speed, constraints, current_twist, current_accel);
    }
  }
}

auto LongitudinalSpeedPlanner::getLinearAccelerationDuration(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints,
  const geometry_msgs::msg::Twist & current_twist,
  const geometry_msgs::msg::Accel & current_accel) const -> double
{
  const double quad_duration =
    getQuadraticAccelerationDuration(target_speed, constraints, current_twist, current_accel);
  if (isReachedToTargetSpeedWithConstantJerk(
        target_speed, constraints, current_twist, current_accel, quad_duration)) {
    return 0;
  } else {
    geometry_msgs::msg::Twist twist = current_twist;
    twist.linear.x = getVelocityWithConstantJerk(
      target_speed, current_twist, current_accel, constraints, quad_duration);
    if (isAccelerating(target_speed, current_twist)) {
      return (target_speed - twist.linear.x) / constraints.max_acceleration;
    } else {
      return (twist.linear.x - target_speed) / constraints.max_deceleration;
    }
  }
}

auto LongitudinalSpeedPlanner::isReachedToTargetSpeedWithConstantJerk(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints,
  const geometry_msgs::msg::Twist & current_twist, const geometry_msgs::msg::Accel & current_accel,
  double duration, double tolerance) const -> bool
{
  return (
    std::abs(
      getVelocityWithConstantJerk(
        target_speed, current_twist, current_accel, constraints, duration) -
      target_speed) <= std::abs(tolerance));
}

auto LongitudinalSpeedPlanner::isAccelerating(
  double target_speed, const geometry_msgs::msg::Twist & current_twist) const -> bool
{
  return (current_twist.linear.x <= target_speed);
}

auto LongitudinalSpeedPlanner::isDecelerating(
  double target_speed, const geometry_msgs::msg::Twist & current_twist) const -> bool
{
  return (current_twist.linear.x >= target_speed);
}

auto LongitudinalSpeedPlanner::planLinearJerk(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints,
  const geometry_msgs::msg::Twist & current_twist,
  const geometry_msgs::msg::Accel & current_accel) const -> double
{
  double accel_x_new = 0;
  if (isAccelerating(target_speed, current_twist)) {
    accel_x_new = std::clamp(
      current_accel.linear.x + step_time * constraints.max_acceleration_rate, 0.0,
      std::min(constraints.max_acceleration, (target_speed - current_twist.linear.x) / step_time));
  } else {
    accel_x_new = std::clamp(
      current_accel.linear.x - step_time * constraints.max_deceleration_rate,
      std::max(
        constraints.max_deceleration * -1, (target_speed - current_twist.linear.x) / step_time),
      0.0);
  }
  return (accel_x_new - current_accel.linear.x) / step_time;
}

auto LongitudinalSpeedPlanner::forward(
  double linear_jerk, const geometry_msgs::msg::Accel & accel,
  const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const
  -> geometry_msgs::msg::Accel
{
  geometry_msgs::msg::Accel ret = accel;
  ret.linear.x = accel.linear.x + step_time * linear_jerk;
  ret.linear.x =
    std::clamp(ret.linear.x, constraints.max_deceleration * -1, constraints.max_acceleration);
  return ret;
}

auto LongitudinalSpeedPlanner::forward(
  const geometry_msgs::msg::Accel & accel, const geometry_msgs::msg::Twist & twist,
  const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const
  -> geometry_msgs::msg::Twist
{
  using math::geometry::operator*;
  using math::geometry::operator+;

  geometry_msgs::msg::Twist ret = twist;
  ret.linear = ret.linear + accel.linear * step_time;
  ret.linear.x = std::clamp(ret.linear.x, -1 * constraints.max_speed, constraints.max_speed);
  ret.angular = ret.angular + accel.angular * step_time;
  return ret;
}

auto LongitudinalSpeedPlanner::timeDerivative(
  const geometry_msgs::msg::Twist & before, const geometry_msgs::msg::Twist & after) const
  -> geometry_msgs::msg::Accel
{
  using math::geometry::operator-;
  using math::geometry::operator/;

  geometry_msgs::msg::Accel ret;
  ret.linear = (after.linear - before.linear) / step_time;
  ret.angular = (after.angular - before.angular) / step_time;
  return ret;
}

auto LongitudinalSpeedPlanner::timeDerivative(
  const geometry_msgs::msg::Accel & before, const geometry_msgs::msg::Accel & after) const -> double
{
  return (after.linear.x - before.linear.x) / step_time;
}
}  // namespace longitudinal_speed_planning
}  // namespace traffic_simulator
