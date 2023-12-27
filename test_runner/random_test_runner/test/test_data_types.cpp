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

#include <random_test_runner/data_types.hpp>

#include "test_utils.hpp"

TEST(DataTypes, simulatorTypeFromString_correct)
{
  EXPECT_EQ(
    simulatorTypeFromString("simple_sensor_simulator"), SimulatorType::SIMPLE_SENSOR_SIMULATOR);
  EXPECT_EQ(simulatorTypeFromString("awsim"), SimulatorType::AWSIM);
}

TEST(DataTypes, simulatorTypeFromString_incorrect)
{
  EXPECT_THROW(simulatorTypeFromString("unknown"), std::runtime_error);
  EXPECT_THROW(simulatorTypeFromString("simple_sensor_simulator "), std::runtime_error);
  EXPECT_THROW(simulatorTypeFromString(" simple_sensor_simulator"), std::runtime_error);
  EXPECT_THROW(simulatorTypeFromString("awsim "), std::runtime_error);
  EXPECT_THROW(simulatorTypeFromString(" awsim"), std::runtime_error);
}

TEST(DataTypes, architectureTypeFromString_correct)
{
  EXPECT_EQ(architectureTypeFromString("awf/auto"), ArchitectureType::AWF_AUTO);
  EXPECT_EQ(architectureTypeFromString("awf/universe"), ArchitectureType::AWF_UNIVERSE);
  EXPECT_EQ(architectureTypeFromString("-awf/universe"), ArchitectureType::AWF_UNIVERSE);
  EXPECT_EQ(architectureTypeFromString("awf/universe-"), ArchitectureType::AWF_UNIVERSE);
  EXPECT_EQ(architectureTypeFromString("awf/universe "), ArchitectureType::AWF_UNIVERSE);
  EXPECT_EQ(architectureTypeFromString(" awf/universe"), ArchitectureType::AWF_UNIVERSE);
  EXPECT_EQ(architectureTypeFromString(" awf/universe "), ArchitectureType::AWF_UNIVERSE);
  EXPECT_EQ(architectureTypeFromString("tier4/proposal"), ArchitectureType::TIER4_PROPOSAL);
}

TEST(DataTypes, architectureTypeFromString_incorrect)
{
  EXPECT_THROW(architectureTypeFromString("unknown"), std::runtime_error);
  EXPECT_THROW(architectureTypeFromString("awf/auto "), std::runtime_error);
  EXPECT_THROW(architectureTypeFromString(" awf/auto"), std::runtime_error);
  EXPECT_THROW(architectureTypeFromString("tier4/proposal "), std::runtime_error);
  EXPECT_THROW(architectureTypeFromString(" tier4/proposal"), std::runtime_error);
}

TEST(DataTypes, stringFromArchitectureType_correct)
{
  EXPECT_EQ(stringFromArchitectureType(ArchitectureType::AWF_AUTO), "awf/auto");
  EXPECT_EQ(stringFromArchitectureType(ArchitectureType::AWF_UNIVERSE), "awf/universe");
  EXPECT_EQ(stringFromArchitectureType(ArchitectureType::TIER4_PROPOSAL), "tier4/proposal");
}

TEST(DataTypes, stringFromArchitectureType_incorrect)
{
  EXPECT_THROW(stringFromArchitectureType(static_cast<ArchitectureType>(-1)), std::runtime_error);
  EXPECT_THROW(stringFromArchitectureType(static_cast<ArchitectureType>(3)), std::runtime_error);
  EXPECT_THROW(stringFromArchitectureType(static_cast<ArchitectureType>(4)), std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
