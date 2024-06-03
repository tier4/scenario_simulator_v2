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

#include <gtest/gtest.h>

#include <cmath>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <traffic_simulator/behavior/longitudinal_speed_planning.hpp>

#include "../expect_eq_macros.hpp"

#define EXPECT_CONSTRAINTS_BOUNDED(DATA, lower, upper) \
  EXPECT_GE(DATA.max_speed, lower);                    \
  EXPECT_GE(DATA.max_acceleration, lower);             \
  EXPECT_GE(DATA.max_deceleration, lower);             \
  EXPECT_GE(DATA.max_acceleration_rate, lower);        \
  EXPECT_GE(DATA.max_deceleration_rate, lower);        \
  EXPECT_LE(DATA.max_speed, upper);                    \
  EXPECT_LE(DATA.max_acceleration, upper);             \
  EXPECT_LE(DATA.max_deceleration, upper);             \
  EXPECT_LE(DATA.max_acceleration_rate, upper);        \
  EXPECT_LE(DATA.max_deceleration_rate, upper);

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

auto makeLongitudinalSpeedPlanner()
  -> traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner
{
  const double step_time = 0.001;
  const std::string entity_name = "entity";
  return traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner{
    step_time, entity_name};
}

auto makeTwistWithLinearX(double x) -> geometry_msgs::msg::Twist
{
  return geometry_msgs::build<geometry_msgs::msg::Twist>()
    .linear(geometry_msgs::build<geometry_msgs::msg::Vector3>().x(x).y(0.0).z(0.0))
    .angular(geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0));
}

auto makeAccelWithLinearX(double x) -> geometry_msgs::msg::Accel
{
  return geometry_msgs::build<geometry_msgs::msg::Accel>()
    .linear(geometry_msgs::build<geometry_msgs::msg::Vector3>().x(x).y(0.0).z(0.0))
    .angular(geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0));
}

/**
 * @note Test basic functionality. Test with an accelerating situation.
 */
TEST(LongitudinalSpeedPlanner, isAccelerating_true)
{
  const auto planner = makeLongitudinalSpeedPlanner();
  const auto current_twist = makeTwistWithLinearX(1.0);

  const double target_speed = 2.0;
  EXPECT_TRUE(planner.isAccelerating(target_speed, current_twist));
}

/**
 * @note Test basic functionality. Test with non accelerating situation.
 */
TEST(LongitudinalSpeedPlanner, isAccelerating_false)
{
  const auto planner = makeLongitudinalSpeedPlanner();
  const auto current_twist = makeTwistWithLinearX(3.0);

  const double target_speed = 2.0;
  EXPECT_FALSE(planner.isAccelerating(target_speed, current_twist));
}

/**
 * @note Test basic functionality. Test with a decelerating situation.
 */
TEST(LongitudinalSpeedPlanner, isDecelerating_true)
{
  const auto planner = makeLongitudinalSpeedPlanner();
  const auto current_twist = makeTwistWithLinearX(3.0);

  const double target_speed = 2.0;
  EXPECT_TRUE(planner.isDecelerating(target_speed, current_twist));
}

/**
 * @note Test basic functionality. Test with non decelerating situation.
 */
TEST(LongitudinalSpeedPlanner, isDecelerating_false)
{
  const auto planner = makeLongitudinalSpeedPlanner();
  const auto current_twist = makeTwistWithLinearX(1.0);

  const double target_speed = 2.0;
  EXPECT_FALSE(planner.isDecelerating(target_speed, current_twist));
}

/**
 * @note Test calculations correctness of acceleration
 * duration with some positive acceleration and speed substantially below target_speed.
 */
TEST(LongitudinalSpeedPlanner, getAccelerationDuration_acceleration)
{
  const auto planner = makeLongitudinalSpeedPlanner();
  const auto current_twist = makeTwistWithLinearX(1.0);
  const auto current_accel = makeAccelWithLinearX(1.0);
  const auto constraints =
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::DynamicConstraints>()
      .max_acceleration(2.0)
      .max_acceleration_rate(1.0)
      .max_deceleration(0.0)
      .max_deceleration_rate(0.0)
      .max_speed(10.0);

  const double tolerance = 1e-5;

  const double target_speed = 8.5;
  const double expected_duration = 4.0;

  const double result_duration =
    planner.getAccelerationDuration(target_speed, constraints, current_twist, current_accel);
  EXPECT_NEAR(result_duration, expected_duration, tolerance);
}

/**
 * @note Test functionality aggregation used in other classes; test function behavior
 * when target_speed is bigger than max speed.
 */
TEST(LongitudinalSpeedPlanner, getDynamicStates_targetSpeedOverLimit)
{
  const auto planner = makeLongitudinalSpeedPlanner();
  const auto current_twist = makeTwistWithLinearX(10.0);
  const auto current_accel = makeAccelWithLinearX(1.0);
  const auto constraints =
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::DynamicConstraints>()
      .max_acceleration(1.0)
      .max_acceleration_rate(1.0)
      .max_deceleration(1.0)
      .max_deceleration_rate(1.0)
      .max_speed(20.0);

  double target_speed = 100.0;
  const auto [result0_twist, result0_accel, result0_jerk] =
    planner.getDynamicStates(target_speed, constraints, current_twist, current_accel);

  target_speed = constraints.max_speed;
  const auto [result1_twist, result1_accel, result1_jerk] =
    planner.getDynamicStates(target_speed, constraints, current_twist, current_accel);

  EXPECT_ACCEL_EQ(result0_accel, result1_accel);
  EXPECT_TWIST_EQ(result0_twist, result1_twist);
  EXPECT_DOUBLE_EQ(result0_jerk, result1_jerk);
}

/**
 * @note Test functionality aggregation used in other classes; test calculations correctness
 * with target_speed = max speed and current speed = 0 - goal is to test the situation
 * when acceleration is so long that max jerk is reached.
 */
TEST(LongitudinalSpeedPlanner, getDynamicStates_maxJerk)
{
  const auto planner = makeLongitudinalSpeedPlanner();
  const auto current_twist = makeTwistWithLinearX(0.0);
  const auto current_accel = makeAccelWithLinearX(0.0);
  auto constraints =
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::DynamicConstraints>()
      .max_acceleration(5.0)
      .max_acceleration_rate(1.0)
      .max_deceleration(5.0)
      .max_deceleration_rate(1.0)
      .max_speed(20.0);

  const double target_speed = constraints.max_speed;

  const double result0_jerk =
    std::get<2>(planner.getDynamicStates(target_speed, constraints, current_twist, current_accel));
  EXPECT_DOUBLE_EQ(result0_jerk, constraints.max_acceleration_rate);

  constraints.max_acceleration_rate = 2.0;
  const double result1_jerk =
    std::get<2>(planner.getDynamicStates(target_speed, constraints, current_twist, current_accel));
  EXPECT_DOUBLE_EQ(result1_jerk, constraints.max_acceleration_rate);
}

/**
 * @note Test functionality aggregation used in other classes; test calculations correctness
 * with target_speed slightly smaller than current speed - goal is to test the situation
 * when the max jerk is not reached because the target_speed is reached first.
 */
TEST(LongitudinalSpeedPlanner, getDynamicStates_shortAccel)
{
  const auto planner = makeLongitudinalSpeedPlanner();
  auto current_twist = makeTwistWithLinearX(0.0);
  const auto current_accel = makeAccelWithLinearX(0.0);
  const auto constraints =
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::DynamicConstraints>()
      .max_acceleration(5.0)
      .max_acceleration_rate(5.0)
      .max_deceleration(5.0)
      .max_deceleration_rate(5.0)
      .max_speed(20.0);

  const double speed_difference = 1e-8;
  double target_speed, result_jerk;
  {
    current_twist.linear.x = 10.0;
    target_speed = current_twist.linear.x - speed_difference;
    result_jerk = std::get<2>(
      planner.getDynamicStates(target_speed, constraints, current_twist, current_accel));
    EXPECT_NE(result_jerk, -constraints.max_deceleration_rate);
  }
  {
    current_twist.linear.x = 0.0;
    target_speed = current_twist.linear.x - speed_difference;
    result_jerk = std::get<2>(
      planner.getDynamicStates(target_speed, constraints, current_twist, current_accel));
    EXPECT_NE(result_jerk, -constraints.max_deceleration_rate);
  }
}

/**
 * @note Test functionality used in other classes; test calculations correctness with target_speed
 * differing from current speed by several units.
 */
TEST(LongitudinalSpeedPlanner, isTargetSpeedReached_different)
{
  const auto planner = makeLongitudinalSpeedPlanner();

  const auto current_twist = makeTwistWithLinearX(13.0);
  const double target_speed = 15.0;
  const double tolerance = 0.1;

  EXPECT_FALSE(planner.isTargetSpeedReached(target_speed, current_twist, tolerance));
}

/**
 * @note Test functionality used in other classes; test calculations correctness
 * with target_speed differing from current speed by very small amount
 * - goal is to simulate equal speed but with numerical noise.
 */
TEST(LongitudinalSpeedPlanner, isTargetSpeedReached_same)
{
  const auto planner = makeLongitudinalSpeedPlanner();
  const auto current_twist = makeTwistWithLinearX(10.0);

  const double target_speed = 10.09;
  const double tolerance = 0.1;

  EXPECT_TRUE(planner.isTargetSpeedReached(target_speed, current_twist, tolerance));
}

/**
 * @note Test functionality aggregation used in other classes; test calculations
 * correctness with target_speed slightly bigger than current speed
 * - goal is to test the situation when the target speed is reached in little time,
 * so the loop executes only once.
 */
TEST(LongitudinalSpeedPlanner, getRunningDistance_shortTime)
{
  const auto planner = makeLongitudinalSpeedPlanner();
  const auto current_twist = makeTwistWithLinearX(10.0);
  const auto current_accel = makeAccelWithLinearX(0.0);
  const auto constraints =
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::DynamicConstraints>()
      .max_acceleration(18.0)
      .max_acceleration_rate(6.0)
      .max_deceleration(18.0)
      .max_deceleration_rate(6.0)
      .max_speed(70.0);

  const double speed_difference = 0.1;
  const double target_speed = current_twist.linear.x - speed_difference;
  const double current_linear_jerk = 0.0;
  const double distance = planner.getRunningDistance(
    target_speed, constraints, current_twist, current_accel, current_linear_jerk);

  EXPECT_GE(distance, 0.0);
  const double quad_time = constraints.max_deceleration / constraints.max_deceleration_rate;
  const double lin_time = (current_twist.linear.x - target_speed) / constraints.max_deceleration;
  const double distance_upper_bound = current_twist.linear.x * std::max(quad_time, lin_time);
  EXPECT_LE(distance, distance_upper_bound);
}

/**
 * @note Test functionality aggregation used in other classes; test calculations correctness
 * with target_speed = 0 and current speed = 50 (or other cruising speed) - goal is to test
 * the situation when the target speed is reached takes longer to reach
 * so the loop has to run miltiple times.
 */
TEST(LongitudinalSpeedPlanner, getRunningDistance_longTime)
{
  const auto planner = makeLongitudinalSpeedPlanner();
  const auto current_twist = makeTwistWithLinearX(60.0);
  const auto current_accel = makeAccelWithLinearX(0.0);
  const auto constraints =
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::DynamicConstraints>()
      .max_acceleration(1.0)
      .max_acceleration_rate(5.0)
      .max_deceleration(1.0)
      .max_deceleration_rate(5.0)
      .max_speed(70.0);

  const double target_speed = 0.0;
  const double current_linear_jerk = 0.0;
  const double distance = planner.getRunningDistance(
    target_speed, constraints, current_twist, current_accel, current_linear_jerk);

  EXPECT_GE(distance, 0.0);
  const double quad_time = constraints.max_deceleration / constraints.max_deceleration_rate;
  const double lin_time = (current_twist.linear.x - target_speed) / constraints.max_deceleration;
  const double distance_upper_bound = current_twist.linear.x * std::max(quad_time, lin_time);
  EXPECT_LE(distance, distance_upper_bound);
}

/**
 * @note Test functionality aggregation used in other classes; test calculations correctness
 * with target_speed identical to current speed so that the time is 0.
 */
TEST(LongitudinalSpeedPlanner, getRunningDistance_zero)
{
  const auto planner = makeLongitudinalSpeedPlanner();
  const auto current_twist = makeTwistWithLinearX(10.0);
  const auto current_accel = makeAccelWithLinearX(0.0);
  const auto constraints =
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::DynamicConstraints>()
      .max_acceleration(5.0)
      .max_acceleration_rate(5.0)
      .max_deceleration(5.0)
      .max_deceleration_rate(5.0)
      .max_speed(20.0);

  const double target_speed = current_twist.linear.x;
  const double current_linear_jerk = 1.0;
  const double distance = planner.getRunningDistance(
    target_speed, constraints, current_twist, current_accel, current_linear_jerk);

  EXPECT_EQ(distance, 0.0);
}
