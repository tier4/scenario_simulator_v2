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

#include <boost/algorithm/clamp.hpp>
#include <geometry/linear_algebra.hpp>
#include <traffic_simulator/behavior/longitudinal_speed_planning.hpp>

namespace traffic_simulator
{
namespace longitudinal_speed_planning
{
LongitudinalSpeedPlanner::LongitudinalSpeedPlanner(double step_time) : step_time(step_time) {}

std::tuple<geometry_msgs::msg::Twist, geometry_msgs::msg::Accel, double>
LongitudinalSpeedPlanner::getDynamicState(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints,
  const geometry_msgs::msg::Twist & current_twist,
  const geometry_msgs::msg::Accel & current_accel) const
{
  double linear_jerk = planLinearJerk(target_speed, constraints, current_twist, current_accel);
  auto accel = planAccel(linear_jerk, current_accel, constraints);
  auto twist = planTwist(accel, current_twist, constraints);
  accel = timeDerivative(twist, current_twist);
  linear_jerk = timeDerivative(accel, current_accel);
  return std::make_tuple(twist, accel, linear_jerk);
}

double LongitudinalSpeedPlanner::planLinearJerk(
  double target_speed, const traffic_simulator_msgs::msg::DynamicConstraints & constraints,
  const geometry_msgs::msg::Twist & current_twist,
  const geometry_msgs::msg::Accel & current_accel) const
{
  double accel_x_new = 0;
  if (current_twist.linear.x > target_speed) {
    accel_x_new = boost::algorithm::clamp(
      current_twist.linear.x - step_time * constraints.max_deceleration_rate,
      std::max(
        constraints.max_deceleration * -1, (target_speed - current_twist.linear.x) / step_time),
      0);
  } else {
    accel_x_new = boost::algorithm::clamp(
      current_accel.linear.x + step_time * constraints.max_acceleration_rate, 0,
      std::min(constraints.max_acceleration, (target_speed - current_twist.linear.x) / step_time));
  }
  return (accel_x_new - current_accel.linear.x) / step_time;
}

geometry_msgs::msg::Accel LongitudinalSpeedPlanner::planAccel(
  double linear_jerk, const geometry_msgs::msg::Accel & accel,
  const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const
{
  geometry_msgs::msg::Accel ret = accel;
  ret.linear.x = accel.linear.x + step_time * linear_jerk;
  ret.linear.x = boost::algorithm::clamp(
    ret.linear.x, constraints.max_deceleration * -1, constraints.max_acceleration);
  return ret;
}

geometry_msgs::msg::Twist LongitudinalSpeedPlanner::planTwist(
  const geometry_msgs::msg::Accel & accel, const geometry_msgs::msg::Twist & twist,
  const traffic_simulator_msgs::msg::DynamicConstraints & constraints) const
{
  geometry_msgs::msg::Twist ret = twist;
  ret.linear = ret.linear + accel.linear * step_time;
  ret.linear.x = boost::algorithm::clamp(ret.linear.x, -10, constraints.max_speed);
  ret.angular = ret.angular + accel.angular * step_time;
  return ret;
}

geometry_msgs::msg::Accel LongitudinalSpeedPlanner::timeDerivative(
  const geometry_msgs::msg::Twist & before, const geometry_msgs::msg::Twist & after) const
{
  geometry_msgs::msg::Accel ret;
  ret.linear = (after.linear - before.linear) / step_time;
  ret.angular = (after.angular - before.angular) / step_time;
  return ret;
}

double LongitudinalSpeedPlanner::timeDerivative(
  const geometry_msgs::msg::Accel & before, const geometry_msgs::msg::Accel & after) const
{
  return (after.linear.x - before.linear.x) / step_time;
}
}  // namespace longitudinal_speed_planning
}  // namespace traffic_simulator
