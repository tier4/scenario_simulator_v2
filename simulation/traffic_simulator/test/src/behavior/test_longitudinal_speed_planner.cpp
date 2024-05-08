#include <gtest/gtest.h>

#include <cmath>
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
  const std::string entity = "entity";
  return traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner{
    step_time, entity};
}

TEST(LongitudinalSpeedPlanner, isAccelerating_true)
{
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  current_twist.linear.x = 1.0;

  const double target_speed = 2.0;
  EXPECT_TRUE(planner.isAccelerating(target_speed, current_twist));
}

TEST(LongitudinalSpeedPlanner, isAccelerating_false)
{
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  current_twist.linear.x = 3.0;

  const double target_speed = 2.0;
  EXPECT_FALSE(planner.isAccelerating(target_speed, current_twist));
}

TEST(LongitudinalSpeedPlanner, isDecelerating_true)
{
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  current_twist.linear.x = 3.0;

  const double target_speed = 2.0;
  EXPECT_TRUE(planner.isDecelerating(target_speed, current_twist));
}

TEST(LongitudinalSpeedPlanner, isDecelerating_false)
{
  auto planner = makeLongitudinalSpeedPlanner();
  geometry_msgs::msg::Twist current_twist{};
  current_twist.linear.x = 1.0;

  const double target_speed = 2.0;
  EXPECT_FALSE(planner.isDecelerating(target_speed, current_twist));
}

TEST(LongitudinalSpeedPlanner, getAccelerationDuration_acceleration)
{
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  geometry_msgs::msg::Accel current_accel{};
  traffic_simulator_msgs::msg::DynamicConstraints constraints{};
  current_twist.linear.x = 1.0;
  current_accel.linear.x = 1.0;
  constraints.max_speed = 10.0;
  constraints.max_acceleration = 2.0;
  constraints.max_acceleration_rate = 1.0;

  const double epsilon = 1e-5;

  const double target_speed = 8.5f;
  const double expected_duration = 4.0;

  const double result_duration =
    planner.getAccelerationDuration(target_speed, constraints, current_twist, current_accel);
  EXPECT_NEAR(result_duration, expected_duration, epsilon);
}

TEST(LongitudinalSpeedPlanner, getAccelerationDuration_zero)
{
  // possible negative duration, 1st issue
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  geometry_msgs::msg::Accel current_accel{};
  traffic_simulator_msgs::msg::DynamicConstraints constraints{};
  current_twist.linear.x = 1.0;
  current_accel.linear.x = 1.0;
  constraints.max_speed = 10.0;
  constraints.max_acceleration = 1.0;
  constraints.max_deceleration = 1.0;
  constraints.max_acceleration_rate = 1.0;
  constraints.max_deceleration_rate = 1.0;

  const double epsilon = 1e-5;

  double target_speed, result_duration;
  {
    target_speed = current_twist.linear.x + epsilon;
    result_duration =
      planner.getAccelerationDuration(target_speed, constraints, current_twist, current_accel);
    EXPECT_GE(result_duration, 0.0);
    EXPECT_LE(result_duration, epsilon);
  }
  {
    target_speed = current_twist.linear.x + 0.0100;
    result_duration =
      planner.getAccelerationDuration(target_speed, constraints, current_twist, current_accel);
    EXPECT_GE(result_duration, 0.0);
    EXPECT_LE(result_duration, 0.0100 + epsilon);
  }
  {
    target_speed = current_twist.linear.x + 0.0099;
    result_duration =
      planner.getAccelerationDuration(target_speed, constraints, current_twist, current_accel);
    EXPECT_GE(result_duration, 0.0);
    EXPECT_LE(result_duration, 0.0099 + epsilon);
  }
}

TEST(LongitudinalSpeedPlanner, planConstraintsFromJerkAndTimeConstraint_jerk)
{
  // possible sqrt of a negative number, results in a nan, 2nd issue
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  geometry_msgs::msg::Accel current_accel{};
  traffic_simulator_msgs::msg::DynamicConstraints constraints{};
  current_accel.linear.x = 1.0;
  constraints.max_speed = 10.0;
  constraints.max_acceleration = 1.0;
  constraints.max_deceleration = 1.0;
  constraints.max_acceleration_rate = 1.0;
  constraints.max_deceleration_rate = 1.0;

  const double target_speed = 5.0;
  const double acceleration_duration = 1.0;
  const double plausible_lower_bound = 0.0;
  const double plausible_upper_bound = 1e2;

  traffic_simulator_msgs::msg::DynamicConstraints new_constraints;

  {
    current_twist.linear.x = 10.0;
    new_constraints = planner.planConstraintsFromJerkAndTimeConstraint(
      target_speed, current_twist, current_accel, acceleration_duration, constraints);
    EXPECT_CONSTRAINTS_BOUNDED(new_constraints, plausible_lower_bound, plausible_upper_bound);
  }
  {
    current_twist.linear.x = 1.0;
    new_constraints = planner.planConstraintsFromJerkAndTimeConstraint(
      target_speed, current_twist, current_accel, acceleration_duration, constraints);
    EXPECT_CONSTRAINTS_BOUNDED(new_constraints, plausible_lower_bound, plausible_upper_bound);
  }
}

TEST(LongitudinalSpeedPlanner, planConstraintsFromJerkAndTimeConstraint_acceleration)
{
  // possible sqrt of a negative number, results in a nan, 2nd issue
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  geometry_msgs::msg::Accel current_accel{};
  traffic_simulator_msgs::msg::DynamicConstraints constraints{};
  current_accel.linear.x = 1.0;
  constraints.max_speed = 10.0;
  constraints.max_acceleration = 1.0;
  constraints.max_deceleration = 1.0;
  constraints.max_acceleration_rate = 1.0;
  constraints.max_deceleration_rate = 1.0;

  const double epsilon = 1e-5;
  const double acceleration_duration = 1.0;
  const double plausible_lower_bound = 0.0;
  const double plausible_upper_bound = 1e2;

  double target_speed;
  traffic_simulator_msgs::msg::DynamicConstraints new_constraints;

  {
    current_twist.linear.x = 10.0;
    target_speed = current_twist.linear.x + epsilon;
    new_constraints = planner.planConstraintsFromJerkAndTimeConstraint(
      target_speed, current_twist, current_accel, acceleration_duration, constraints);
    EXPECT_CONSTRAINTS_BOUNDED(new_constraints, plausible_lower_bound, plausible_upper_bound);
  }
  {
    current_twist.linear.x = 0.0;
    target_speed = current_twist.linear.x + epsilon;
    new_constraints = planner.planConstraintsFromJerkAndTimeConstraint(
      target_speed, current_twist, current_accel, acceleration_duration, constraints);
    EXPECT_CONSTRAINTS_BOUNDED(new_constraints, plausible_lower_bound, plausible_upper_bound);
  }
}

TEST(LongitudinalSpeedPlanner, planConstraintsFromJerkAndTimeConstraint_deceleration)
{
  // possible sqrt of a negative number, results in a nan, 2nd issue
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  geometry_msgs::msg::Accel current_accel{};
  traffic_simulator_msgs::msg::DynamicConstraints constraints{};
  current_accel.linear.x = -1.0;
  constraints.max_speed = 10.0;
  constraints.max_acceleration = 1.0;
  constraints.max_deceleration = 1.0;
  constraints.max_acceleration_rate = 1.0;
  constraints.max_deceleration_rate = 1.0;

  const double epsilon = 1e-5;
  const double acceleration_duration = 1.0;
  const double plausible_lower_bound = 0.0;
  const double plausible_upper_bound = 1e2;

  double target_speed;
  traffic_simulator_msgs::msg::DynamicConstraints new_constraints;
  {
    current_twist.linear.x = 10.0;
    target_speed = current_twist.linear.x - epsilon;
    new_constraints = planner.planConstraintsFromJerkAndTimeConstraint(
      target_speed, current_twist, current_accel, acceleration_duration, constraints);
    EXPECT_CONSTRAINTS_BOUNDED(new_constraints, plausible_lower_bound, plausible_upper_bound);
  }
  {
    current_twist.linear.x = 0.0;
    target_speed = current_twist.linear.x - epsilon;
    new_constraints = planner.planConstraintsFromJerkAndTimeConstraint(
      target_speed, current_twist, current_accel, acceleration_duration, constraints);
    EXPECT_CONSTRAINTS_BOUNDED(new_constraints, plausible_lower_bound, plausible_upper_bound);
  }
}

TEST(LongitudinalSpeedPlanner, getDynamicStates_targetSpeedOverLimit)
{
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  geometry_msgs::msg::Accel current_accel{};
  traffic_simulator_msgs::msg::DynamicConstraints constraints{};
  current_twist.linear.x = 10.0;
  current_accel.linear.x = 1.0;
  constraints.max_speed = 20.0;
  constraints.max_acceleration = 1.0;
  constraints.max_deceleration = 1.0;
  constraints.max_acceleration_rate = 1.0;
  constraints.max_deceleration_rate = 1.0;

  double target_speed = 100.0;
  auto [result0_twist, result0_accel, result0_jerk] =
    planner.getDynamicStates(target_speed, constraints, current_twist, current_accel);

  target_speed = constraints.max_speed;
  auto [result1_twist, result1_accel, result1_jerk] =
    planner.getDynamicStates(target_speed, constraints, current_twist, current_accel);

  EXPECT_ACCEL_EQ(result0_accel, result1_accel);
  EXPECT_TWIST_EQ(result0_twist, result1_twist);
  EXPECT_DOUBLE_EQ(result0_jerk, result1_jerk);
}

TEST(LongitudinalSpeedPlanner, getDynamicStates_maxJerk)
{
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  geometry_msgs::msg::Accel current_accel{};
  traffic_simulator_msgs::msg::DynamicConstraints constraints{};
  current_twist.linear.x = 0.0;
  current_accel.linear.x = 0.0;
  constraints.max_speed = 20.0;
  constraints.max_acceleration = 5.0;
  constraints.max_deceleration = 5.0;
  constraints.max_acceleration_rate = 1.0;
  constraints.max_deceleration_rate = 1.0;

  const double target_speed = constraints.max_speed;

  const double result0_jerk =
    std::get<2>(planner.getDynamicStates(target_speed, constraints, current_twist, current_accel));
  EXPECT_DOUBLE_EQ(result0_jerk, constraints.max_acceleration_rate);

  constraints.max_acceleration_rate = 2.0;
  const double result1_jerk =
    std::get<2>(planner.getDynamicStates(target_speed, constraints, current_twist, current_accel));
  EXPECT_DOUBLE_EQ(result1_jerk, constraints.max_acceleration_rate);
}

TEST(LongitudinalSpeedPlanner, getDynamicStates_shortAccel)
{
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  geometry_msgs::msg::Accel current_accel{};
  traffic_simulator_msgs::msg::DynamicConstraints constraints{};
  current_accel.linear.x = 0.0;
  constraints.max_speed = 20.0;
  constraints.max_acceleration = 5.0;
  constraints.max_deceleration = 5.0;
  constraints.max_acceleration_rate = 5.0;
  constraints.max_deceleration_rate = 5.0;

  const double epsilon = 1e-8;
  double target_speed, result_jerk;

  {
    current_twist.linear.x = 10.0;
    target_speed = current_twist.linear.x - epsilon;
    result_jerk = std::get<2>(
      planner.getDynamicStates(target_speed, constraints, current_twist, current_accel));
    EXPECT_NE(result_jerk, -constraints.max_deceleration_rate);
  }
  {
    current_twist.linear.x = 0.0;
    target_speed = current_twist.linear.x - epsilon;
    result_jerk = std::get<2>(
      planner.getDynamicStates(target_speed, constraints, current_twist, current_accel));
    EXPECT_NE(result_jerk, -constraints.max_deceleration_rate);
  }
}

TEST(LongitudinalSpeedPlanner, isTargetSpeedReached_different)
{
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  current_twist.linear.x = 13.0;
  const double target_speed = 15.0;
  const double tolerance = 0.1;

  EXPECT_FALSE(planner.isTargetSpeedReached(target_speed, current_twist, tolerance));
}

TEST(LongitudinalSpeedPlanner, isTargetSpeedReached_same)
{
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  current_twist.linear.x = 10.0;
  const double target_speed = 10.09;
  const double tolerance = 0.1;

  EXPECT_TRUE(planner.isTargetSpeedReached(target_speed, current_twist, tolerance));
}

TEST(LongitudinalSpeedPlanner, getRunningDistance_shortTime)
{
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  geometry_msgs::msg::Accel current_accel{};
  traffic_simulator_msgs::msg::DynamicConstraints constraints{};
  current_twist.linear.x = 10.0;
  current_accel.linear.x = 0.0;
  constraints.max_speed = 70.0;
  constraints.max_acceleration = 18.0;
  constraints.max_deceleration = 18.0;
  constraints.max_acceleration_rate = 6.0;
  constraints.max_deceleration_rate = 6.0;

  const double epsilon = 0.1;
  const double target_speed = current_twist.linear.x - epsilon;
  const double current_linear_jerk = 0.0;
  const double distance = planner.getRunningDistance(
    target_speed, constraints, current_twist, current_accel, current_linear_jerk);

  EXPECT_GE(distance, 0.0);
  const double quad_time = constraints.max_deceleration / constraints.max_deceleration_rate;
  const double lin_time = (current_twist.linear.x - target_speed) / constraints.max_deceleration;
  const double distance_upper_bound = current_twist.linear.x * std::max(quad_time, lin_time);
  EXPECT_LE(distance, distance_upper_bound);
}

TEST(LongitudinalSpeedPlanner, getRunningDistance_longTime)
{
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  geometry_msgs::msg::Accel current_accel{};
  traffic_simulator_msgs::msg::DynamicConstraints constraints{};
  current_twist.linear.x = 60.0;
  current_accel.linear.x = 0.0;
  constraints.max_speed = 70.0;
  constraints.max_acceleration = 1.0;
  constraints.max_deceleration = 1.0;
  constraints.max_acceleration_rate = 5.0;
  constraints.max_deceleration_rate = 5.0;

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

TEST(LongitudinalSpeedPlanner, getRunningDistance_zero)
{
  auto planner = makeLongitudinalSpeedPlanner();

  geometry_msgs::msg::Twist current_twist{};
  geometry_msgs::msg::Accel current_accel{};
  traffic_simulator_msgs::msg::DynamicConstraints constraints{};
  current_twist.linear.x = 10.0;
  current_accel.linear.x = 0.0;
  constraints.max_speed = 20.0;
  constraints.max_acceleration = 5.0;
  constraints.max_deceleration = 5.0;
  constraints.max_acceleration_rate = 5.0;
  constraints.max_deceleration_rate = 5.0;

  const double target_speed = current_twist.linear.x;
  const double current_linear_jerk = 1.0;
  const double distance = planner.getRunningDistance(
    target_speed, constraints, current_twist, current_accel, current_linear_jerk);

  EXPECT_EQ(distance, 0.0);
}

/*
ISSUES:
1: 185, 199: negative duration can be returned, possibly because
  "getQuadraticAccelerationDurationWithConstantJerk" is being invoked if-and-only-if
  "v" and "target_speed" is almost equal, which does not seem correct.
  This part of the code also seems incongruent with the comments in 184, 188, 198 and 202.
2: 59, 68: possible sqrt of a negative number.
  "target_speed" value can overwhelm the equation. The equation itself
  (its derivation) could not be understood.
*/
