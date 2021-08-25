// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <regex>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/math/uuid.hpp>

TEST(UUID, GenerateFromSameSeed)
{
  for (int i = 0; i < 100; i++) {
    std::string seed = "seed_" + std::to_string(i);
    EXPECT_STREQ(
      traffic_simulator::math::generateUUID(seed).c_str(),
      traffic_simulator::math::generateUUID(seed).c_str());
  }
  EXPECT_STREQ(
    traffic_simulator::math::generateUUID("test").c_str(),
    traffic_simulator::math::generateUUID("test").c_str());
}

TEST(UUID, GenerateFromAnotherSeed)
{
  EXPECT_STRNE(
    traffic_simulator::math::generateUUID("hoge").c_str(),
    traffic_simulator::math::generateUUID("fuga").c_str());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
