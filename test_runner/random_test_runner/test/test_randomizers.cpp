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

#include <random_test_runner/randomizers.hpp>

#include "expect_eq_macros.hpp"

TEST(UniformRandomizer, int_generate255)
{
  RandomizationEnginePtr engine = std::make_shared<RandomizationEngine>();
  UniformRandomizer<int> rand(engine, 0, 255);
  int sum = 0;
  for (int i = 0; i < 1e+6; ++i) {
    const auto generated = rand.generate();
    sum += generated;
    EXPECT_IN_RANGE(generated, 0, 255);
  }
  EXPECT_NEAR(static_cast<double>(sum) / 1e+6, 127.5, 1.0);
}

TEST(UniformRandomizer, int_generate10)
{
  RandomizationEnginePtr engine = std::make_shared<RandomizationEngine>();
  UniformRandomizer<int> rand(engine, 0, 10);
  int sum = 0;
  for (int i = 0; i < 1e+6; ++i) {
    const auto generated = rand.generate();
    sum += generated;
    EXPECT_IN_RANGE(generated, 0, 10);
  }
  EXPECT_NEAR(static_cast<double>(sum) / 1e+6, 5.0, 0.1);
}

TEST(UniformRandomizer, int_generate10seed)
{
  RandomizationEnginePtr engine = std::make_shared<RandomizationEngine>(1);
  UniformRandomizer<int> rand(engine, 0, 10);
  EXPECT_EQ(rand.generate(), 4);
  EXPECT_EQ(rand.generate(), 10);
  EXPECT_EQ(rand.generate(), 7);
  EXPECT_EQ(rand.generate(), 10);
  EXPECT_EQ(rand.generate(), 0);
}

TEST(UniformRandomizer, int_setRange10)
{
  RandomizationEnginePtr engine = std::make_shared<RandomizationEngine>();
  UniformRandomizer<int> rand(engine, 0, 255);
  rand.setRange(10, 20);
  int sum = 0;
  for (int i = 0; i < 1e+6; ++i) {
    const auto generated = rand.generate();
    sum += generated;
    EXPECT_IN_RANGE(generated, 10, 20);
  }
  EXPECT_NEAR(static_cast<double>(sum) / 1e+6, 15.0, 0.1);
}

TEST(UniformRandomizer, double_generate100)
{
  RandomizationEnginePtr engine = std::make_shared<RandomizationEngine>();
  UniformRandomizer<double> rand(engine, 0.0, 100.0);
  double sum = 0.0;
  for (int i = 0; i < 1e+6; ++i) {
    const auto generated = rand.generate();
    sum += generated;
    EXPECT_IN_RANGE(generated, 0.0, 100.0);
  }
  EXPECT_NEAR(sum / 1e+6, 50.0, 1.0);
}

TEST(UniformRandomizer, double_generate1)
{
  RandomizationEnginePtr engine = std::make_shared<RandomizationEngine>();
  UniformRandomizer<double> rand(engine, 0.0, 1.0);
  double sum = 0.0;
  for (int i = 0; i < 1e+6; ++i) {
    const auto generated = rand.generate();
    sum += generated;
    EXPECT_IN_RANGE(generated, 0.0, 1.0);
  }
  EXPECT_NEAR(sum / 1e+6, 0.5, 0.1);
}

TEST(UniformRandomizer, double_generate1seed)
{
  RandomizationEnginePtr engine = std::make_shared<RandomizationEngine>(1);
  UniformRandomizer<double> rand(engine, 0.0, 1.0);
  EXPECT_NEAR(rand.generate(), 0.997185, 1e-6);
  EXPECT_NEAR(rand.generate(), 0.932557, 1e-6);
  EXPECT_NEAR(rand.generate(), 0.128124, 1e-6);
  EXPECT_NEAR(rand.generate(), 0.999041, 1e-6);
  EXPECT_NEAR(rand.generate(), 0.236089, 1e-6);
}

TEST(UniformRandomizer, double_setRange1)
{
  RandomizationEnginePtr engine = std::make_shared<RandomizationEngine>();
  UniformRandomizer<double> rand(engine, 0.0, 100.0);
  rand.setRange(1.0, 2.0);
  double sum = 0.0;
  for (int i = 0; i < 1e+6; ++i) {
    const auto generated = rand.generate();
    sum += generated;
    EXPECT_IN_RANGE(generated, 1.0, 2.0);
  }
  EXPECT_NEAR(sum / 1e+6, 1.5, 0.1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
