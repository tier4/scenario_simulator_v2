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
//
// Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

#include <gtest/gtest.h>

#include <random_test_runner/metrics/goal_reached_metric.hpp>

#include "test_utils.hpp"

TEST(Metrics, GoalReachedMetric_noGoal)
{
  GoalReachedMetric metric;
  EXPECT_FALSE(metric.isGoalReached(getCanonicalizedEntityStatus(0.0, 0.0, 0.0)));
}

TEST(Metrics, GoalReachedMetric_goalNotReached)
{
  GoalReachedMetric metric;
  metric.setGoal(geometry_msgs::build<geometry_msgs::msg::Pose>()
                   .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(5.0).y(5.0).z(0.0))
                   .orientation(geometry_msgs::msg::Quaternion()));
  EXPECT_FALSE(metric.isGoalReached(getCanonicalizedEntityStatus(0.0, 0.0, 0.0)));
  EXPECT_FALSE(metric.isGoalReached(getCanonicalizedEntityStatus(0.0, 0.0, 5.0)));
  EXPECT_FALSE(metric.isGoalReached(getCanonicalizedEntityStatus(0.0, 0.0, 0.0, 5.0)));
  EXPECT_FALSE(metric.isGoalReached(getCanonicalizedEntityStatus(0.0, 0.0, 5.0, 10.0)));
  EXPECT_FALSE(metric.isGoalReached(getCanonicalizedEntityStatus(0.0, 0.0, 10.0, 5.0)));
}

TEST(Metrics, GoalReachedMetric_goalReached)
{
  GoalReachedMetric metric;
  metric.setGoal(geometry_msgs::build<geometry_msgs::msg::Pose>()
                   .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(5.0).y(5.0).z(0.0))
                   .orientation(geometry_msgs::msg::Quaternion()));
  constexpr double a = std::sqrt(3.0) - 1e-3;
  constexpr double less = 5.0 - a;
  constexpr double more = 5.0 + a;
  EXPECT_TRUE(metric.isGoalReached(getCanonicalizedEntityStatus(0.0, 0.0, less, less)));
  EXPECT_TRUE(metric.isGoalReached(getCanonicalizedEntityStatus(0.0, 0.0, less, more)));
  EXPECT_TRUE(metric.isGoalReached(getCanonicalizedEntityStatus(0.0, 0.0, more, more)));
  EXPECT_TRUE(metric.isGoalReached(getCanonicalizedEntityStatus(0.0, 0.0, more, less)));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
