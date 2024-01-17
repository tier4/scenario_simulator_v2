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

#include <random_test_runner/metrics/ego_collision_metric.hpp>

TEST(Metrics, EgoCollisionMetric_noCollision)
{
  EgoCollisionMetric metric;
  EXPECT_TRUE(metric.isThereEgosCollisionWith("npc", 0.0));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 0.4999));
}

TEST(Metrics, EgoCollisionMetric_collision)
{
  EgoCollisionMetric metric;
  EXPECT_TRUE(metric.isThereEgosCollisionWith("npc", 0.0));
  EXPECT_TRUE(metric.isThereEgosCollisionWith("npc", 0.5001));
}

TEST(Metrics, EgoCollisionMetric_collisionOtherNPC)
{
  EgoCollisionMetric metric;
  EXPECT_TRUE(metric.isThereEgosCollisionWith("npc", 0.0));
  EXPECT_TRUE(metric.isThereEgosCollisionWith("other", 0.1));
  EXPECT_TRUE(metric.isThereEgosCollisionWith("npc", 0.5001));
}

TEST(Metrics, EgoCollisionMetric_noCollisionUpdate)
{
  EgoCollisionMetric metric;
  EXPECT_TRUE(metric.isThereEgosCollisionWith("npc", 0.0));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 0.1));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 0.5999));
}

TEST(Metrics, EgoCollisionMetric_noCollisionUpdateOtherNPC)
{
  EgoCollisionMetric metric;
  EXPECT_TRUE(metric.isThereEgosCollisionWith("npc", 0.0));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 0.1));
  EXPECT_TRUE(metric.isThereEgosCollisionWith("other", 0.4));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 0.5999));
}

TEST(Metrics, EgoCollisionMetric_collisionUpdate)
{
  EgoCollisionMetric metric;
  EXPECT_TRUE(metric.isThereEgosCollisionWith("npc", 0.0));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 0.1));
  EXPECT_TRUE(metric.isThereEgosCollisionWith("npc", 0.6001));
}

TEST(Metrics, EgoCollisionMetric_collisionUpdateOtherNPC)
{
  EgoCollisionMetric metric;
  EXPECT_TRUE(metric.isThereEgosCollisionWith("npc", 0.0));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 0.1));
  EXPECT_TRUE(metric.isThereEgosCollisionWith("other", 0.4));
  EXPECT_TRUE(metric.isThereEgosCollisionWith("npc", 0.6001));
}

TEST(Metrics, EgoCollisionMetric_collisionAfterLongTime)
{
  EgoCollisionMetric metric;
  EXPECT_TRUE(metric.isThereEgosCollisionWith("npc", 0.0));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 0.5));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 1.0));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 1.5));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 2.0));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 2.5));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 3.0));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 3.5));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 4.0));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 4.5));
  EXPECT_FALSE(metric.isThereEgosCollisionWith("npc", 5.0));
  EXPECT_TRUE(metric.isThereEgosCollisionWith("npc", 6.0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
