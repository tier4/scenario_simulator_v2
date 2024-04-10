#include <gtest/gtest.h>

#include <cmath>
#include <traffic_simulator/behavior/longitudinal_speed_planning.hpp>

#include "../expect_eq_macros.hpp"

#define EXPECT_CONSTRAINTS_BOUNDED(DATA, lower, upper)         \
  EXPECT_TRUE(new_constraints.max_speed >= lower);             \
  EXPECT_TRUE(new_constraints.max_acceleration >= lower);      \
  EXPECT_TRUE(new_constraints.max_deceleration >= lower);      \
  EXPECT_TRUE(new_constraints.max_acceleration_rate >= lower); \
  EXPECT_TRUE(new_constraints.max_deceleration_rate >= lower); \
  EXPECT_TRUE(new_constraints.max_speed < upper);              \
  EXPECT_TRUE(new_constraints.max_acceleration < upper);       \
  EXPECT_TRUE(new_constraints.max_deceleration < upper);       \
  EXPECT_TRUE(new_constraints.max_acceleration_rate < upper);  \
  EXPECT_TRUE(new_constraints.max_deceleration_rate < upper);

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(LongitudinalSpeedPlanner, isAccelerating_true)
{
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

  geometry_msgs::msg::Twist current_twist{};
  current_twist.linear.x = 1.0;

  const double target_speed = 2.0;
  EXPECT_TRUE(planner.isAccelerating(target_speed, current_twist));
}

TEST(LongitudinalSpeedPlanner, isAccelerating_false)
{
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

  geometry_msgs::msg::Twist current_twist{};
  current_twist.linear.x = 3.0;

  const double target_speed = 2.0;
  EXPECT_FALSE(planner.isAccelerating(target_speed, current_twist));
}

TEST(LongitudinalSpeedPlanner, isDecelerating_true)
{
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

  geometry_msgs::msg::Twist current_twist{};
  current_twist.linear.x = 3.0;

  const double target_speed = 2.0;
  EXPECT_TRUE(planner.isDecelerating(target_speed, current_twist));
}

TEST(LongitudinalSpeedPlanner, isDecelerating_false)
{
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

  geometry_msgs::msg::Twist current_twist{};
  current_twist.linear.x = 1.0;

  const double target_speed = 2.0;
  EXPECT_FALSE(planner.isDecelerating(target_speed, current_twist));
}

TEST(LongitudinalSpeedPlanner, getAccelerationDuration_acceleration)
{
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

  geometry_msgs::msg::Twist current_twist{};
  geometry_msgs::msg::Accel current_accel{};
  traffic_simulator_msgs::msg::DynamicConstraints constraints{};
  current_twist.linear.x = 1.0;
  current_accel.linear.x = 1.0;
  constraints.max_speed = 10.0;
  constraints.max_acceleration = 2.0;
  constraints.max_acceleration_rate = 1.0;

  const double epsilon = 1e-5;

  const double expected_duration = 4.0;
  const double target_speed = 8.5f;

  const double result_duration =
    planner.getAccelerationDuration(target_speed, constraints, current_twist, current_accel);
  EXPECT_TRUE(std::abs(expected_duration - result_duration) < epsilon);
}

TEST(LongitudinalSpeedPlanner, getAccelerationDuration_zero)
{
  // possible negative duration:
  // longitudinal_speed_planning: 185, 199
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

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

  double target_speed = current_twist.linear.x + epsilon;
  double result_duration =
    planner.getAccelerationDuration(target_speed, constraints, current_twist, current_accel);
  EXPECT_TRUE(result_duration >= 0);
  EXPECT_TRUE(result_duration < epsilon);

  constraints.max_speed = 100.0;
  result_duration =
    planner.getAccelerationDuration(target_speed, constraints, current_twist, current_accel);
  EXPECT_TRUE(result_duration >= 0);
  EXPECT_TRUE(result_duration < epsilon);

  current_twist.linear.x = 0.0;
  target_speed = current_twist.linear.x + epsilon;
  result_duration =
    planner.getAccelerationDuration(target_speed, constraints, current_twist, current_accel);
  EXPECT_TRUE(result_duration >= 0);
  EXPECT_TRUE(result_duration < epsilon);

  target_speed = current_twist.linear.x + 0.0101;
  result_duration =
    planner.getAccelerationDuration(target_speed, constraints, current_twist, current_accel);
  EXPECT_TRUE(result_duration >= 0);
  EXPECT_TRUE(result_duration <= 0.0101);
  target_speed = current_twist.linear.x + 0.0099;
  result_duration =
    planner.getAccelerationDuration(target_speed, constraints, current_twist, current_accel);
  EXPECT_TRUE(result_duration >= 0);
  EXPECT_TRUE(result_duration <= 0.0099);
}

TEST(LongitudinalSpeedPlanner, planConstraintsFromJerkAndTimeConstraint_jerk)
{
  // possible sqrt of a negative number, results in a nan
  // longitudinal_speed_planning: 59, 68
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

  geometry_msgs::msg::Twist current_twist{};
  geometry_msgs::msg::Accel current_accel{};
  traffic_simulator_msgs::msg::DynamicConstraints constraints{};
  current_twist.linear.x = 10.0;
  current_accel.linear.x = 1.0;
  constraints.max_speed = 10.0;
  constraints.max_acceleration = 1.0;
  constraints.max_deceleration = 1.0;
  constraints.max_acceleration_rate = 1.0;
  constraints.max_deceleration_rate = 1.0;

  double target_speed = 5.0;
  double acceleration_duration = 1.0;
  auto new_constraints = planner.planConstraintsFromJerkAndTimeConstraint(
    target_speed, current_twist, current_accel, acceleration_duration, constraints);

  const double plausible_bound = 1e2;
  EXPECT_CONSTRAINTS_BOUNDED(constraints, 0, plausible_bound);

  current_twist.linear.x = 1.0;
  new_constraints = planner.planConstraintsFromJerkAndTimeConstraint(
    target_speed, current_twist, current_accel, acceleration_duration, constraints);
  EXPECT_CONSTRAINTS_BOUNDED(constraints, 0, plausible_bound);
}

TEST(LongitudinalSpeedPlanner, planConstraintsFromJerkAndTimeConstraint_acceleration)
{
  // possible sqrt of a negative number, results in a nan
  // longitudinal_speed_planning: 59, 68
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

  geometry_msgs::msg::Twist current_twist{};
  geometry_msgs::msg::Accel current_accel{};
  traffic_simulator_msgs::msg::DynamicConstraints constraints{};
  current_twist.linear.x = 10.0;
  current_accel.linear.x = 1.0;
  constraints.max_speed = 10.0;
  constraints.max_acceleration = 1.0;
  constraints.max_deceleration = 1.0;
  constraints.max_acceleration_rate = 1.0;
  constraints.max_deceleration_rate = 1.0;

  const double epsilon = 1e5;
  double target_speed = current_twist.linear.x + epsilon;
  double acceleration_duration = 1.0;
  auto new_constraints = planner.planConstraintsFromJerkAndTimeConstraint(
    target_speed, current_twist, current_accel, acceleration_duration, constraints);

  const double plausible_bound = 1e2;
  EXPECT_CONSTRAINTS_BOUNDED(constraints, 0, plausible_bound);

  current_twist.linear.x = 0.0;
  target_speed = current_twist.linear.x + epsilon;
  new_constraints = planner.planConstraintsFromJerkAndTimeConstraint(
    target_speed, current_twist, current_accel, acceleration_duration, constraints);
  EXPECT_CONSTRAINTS_BOUNDED(constraints, 0, plausible_bound);
}

TEST(LongitudinalSpeedPlanner, planConstraintsFromJerkAndTimeConstraint_deceleration)
{
  // possible sqrt of a negative number, results in a nan
  // longitudinal_speed_planning: 59, 68
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

  geometry_msgs::msg::Twist current_twist{};
  geometry_msgs::msg::Accel current_accel{};
  traffic_simulator_msgs::msg::DynamicConstraints constraints{};
  current_twist.linear.x = 10.0;
  current_accel.linear.x = 1.0;
  constraints.max_speed = 10.0;
  constraints.max_acceleration = 1.0;
  constraints.max_deceleration = 1.0;
  constraints.max_acceleration_rate = 1.0;
  constraints.max_deceleration_rate = 1.0;

  const double epsilon = 1e5;
  double target_speed = current_twist.linear.x - epsilon;
  double acceleration_duration = 1.0;
  auto new_constraints = planner.planConstraintsFromJerkAndTimeConstraint(
    target_speed, current_twist, current_accel, acceleration_duration, constraints);

  const double plausible_bound = 1e2;
  EXPECT_CONSTRAINTS_BOUNDED(constraints, 0, plausible_bound);

  current_twist.linear.x = 0.0;
  target_speed = current_twist.linear.x - epsilon;
  new_constraints = planner.planConstraintsFromJerkAndTimeConstraint(
    target_speed, current_twist, current_accel, acceleration_duration, constraints);
  EXPECT_CONSTRAINTS_BOUNDED(constraints, 0, plausible_bound);
}

TEST(LongitudinalSpeedPlanner, getDynamicStates_targetSpeedOverLimit)
{
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

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
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

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

  double target_speed = constraints.max_speed;

  double result0_jerk =
    std::get<2>(planner.getDynamicStates(target_speed, constraints, current_twist, current_accel));
  EXPECT_DOUBLE_EQ(result0_jerk, constraints.max_acceleration_rate);

  constraints.max_acceleration_rate = 2.0;
  double result1_jerk =
    std::get<2>(planner.getDynamicStates(target_speed, constraints, current_twist, current_accel));
  EXPECT_DOUBLE_EQ(result1_jerk, constraints.max_acceleration_rate);
}

TEST(LongitudinalSpeedPlanner, getDynamicStates_shortAccel)
{
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

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

  const double epsilon = 1e-8;
  double target_speed = current_twist.linear.x - epsilon;

  double result0_jerk =
    std::get<2>(planner.getDynamicStates(target_speed, constraints, current_twist, current_accel));
  EXPECT_FALSE(result0_jerk == -constraints.max_deceleration_rate);

  constraints.max_deceleration_rate = 2.0;
  double result1_jerk =
    std::get<2>(planner.getDynamicStates(target_speed, constraints, current_twist, current_accel));
  EXPECT_FALSE(result1_jerk == -constraints.max_deceleration_rate);

  current_twist.linear.x = 0;
  target_speed = current_twist.linear.x - epsilon;
  double result2_jerk =
    std::get<2>(planner.getDynamicStates(target_speed, constraints, current_twist, current_accel));
  EXPECT_FALSE(result2_jerk == -constraints.max_deceleration_rate);
}

TEST(LongitudinalSpeedPlanner, isTargetSpeedReached_different)
{
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

  geometry_msgs::msg::Twist current_twist{};
  current_twist.linear.x = 10.0;
  double target_speed = 15.0;
  double tolerance = 3.0;

  EXPECT_FALSE(planner.isTargetSpeedReached(target_speed, current_twist, tolerance));

  target_speed = 12.0;
  EXPECT_TRUE(planner.isTargetSpeedReached(target_speed, current_twist, tolerance));
}

TEST(LongitudinalSpeedPlanner, isTargetSpeedReached_same)
{
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

  geometry_msgs::msg::Twist current_twist{};
  current_twist.linear.x = 10.0;
  double target_speed = 10.0;
  double tolerance = 1.0;

  EXPECT_TRUE(planner.isTargetSpeedReached(target_speed, current_twist, tolerance));

  tolerance = 0.0;
  EXPECT_TRUE(planner.isTargetSpeedReached(target_speed, current_twist, tolerance));
}

TEST(LongitudinalSpeedPlanner, getRunningDistance_shortTime)
{
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

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
  double target_speed = current_twist.linear.x - epsilon;
  double current_linear_jerk = 0.0;
  double distance = planner.getRunningDistance(
    target_speed, constraints, current_twist, current_accel, current_linear_jerk);

  EXPECT_TRUE(distance > 0);
  double quad_time = constraints.max_deceleration / constraints.max_deceleration_rate;
  double lin_time = (current_twist.linear.x - target_speed) / constraints.max_deceleration;
  double distance_upper_bound = current_twist.linear.x * std::max(quad_time, lin_time);
  EXPECT_TRUE(distance < distance_upper_bound);
}

TEST(LongitudinalSpeedPlanner, getRunningDistance_longTime)
{
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

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

  double target_speed = 0.0;
  double current_linear_jerk = 0.0;
  double distance = planner.getRunningDistance(
    target_speed, constraints, current_twist, current_accel, current_linear_jerk);

  EXPECT_TRUE(distance > 0);
  double quad_time = constraints.max_deceleration / constraints.max_deceleration_rate;
  double lin_time = (current_twist.linear.x - target_speed) / constraints.max_deceleration;
  double distance_upper_bound = current_twist.linear.x * std::max(quad_time, lin_time);
  EXPECT_TRUE(distance < distance_upper_bound);
}

TEST(LongitudinalSpeedPlanner, getRunningDistance_zero)
{
  const double step_time = 0.001;
  const std::string entity = "entity";
  traffic_simulator::longitudinal_speed_planning::LongitudinalSpeedPlanner planner{
    step_time, entity};

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

  double target_speed = current_twist.linear.x;
  double current_linear_jerk = 1.0;
  double distance = planner.getRunningDistance(
    target_speed, constraints, current_twist, current_accel, current_linear_jerk);

  EXPECT_TRUE(distance == 0);
}
