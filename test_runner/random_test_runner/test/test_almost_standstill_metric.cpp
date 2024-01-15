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

#include <random_test_runner/metrics/almost_standstill_metric.hpp>

#include "test_utils.hpp"

TEST(Metrics, AlmostStandstillMetric_move)
{
  AlmostStandstillMetric metric;
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(0.0, 1.0, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(2.0, 1.0, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(4.0, 1.0, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(6.0, 1.0, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(8.0, 1.0, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(10.0, 1.0, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(12.0, 1.0, 0.0)));
}

TEST(Metrics, AlmostStandstillMetric_almostStandstill)
{
  AlmostStandstillMetric metric;
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(0.0, 1e-3, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(2.0, 1e-3, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(4.0, 1e-3, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(6.0, 1e-3, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(8.0, 1e-3, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(10.0, 1e-3, 0.0)));
  EXPECT_TRUE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(10.001, 1e-3, 0.0)));
  EXPECT_TRUE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(12.0, 1e-3, 0.0)));
}

TEST(Metrics, AlmostStandstillMetric_dataTimeout)
{
  AlmostStandstillMetric metric;
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(0.0, 1e-3, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(2.0, 1e-3, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(4.0, 1e-3, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(8.0, 1e-3, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(10.0, 1e-3, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(10.001, 1e-3, 0.0)));
  EXPECT_FALSE(metric.isAlmostStandingStill(getCanonicalizedEntityStatus(12.0, 1e-3, 09.0)));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
