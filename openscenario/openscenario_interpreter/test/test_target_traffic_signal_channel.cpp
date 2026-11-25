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

#include <openscenario_interpreter/syntax/traffic_signal_state.hpp>

using openscenario_interpreter::syntax::TrafficSignalState;
using TrafficSignalChannelType = TrafficSignalState::TrafficSignalChannelType;
using TargetTrafficSignalChannel = TrafficSignalState::TargetTrafficSignalChannel;

struct ValidParseTestCase
{
  std::string input;
  lanelet::Id expected_id;
  TrafficSignalChannelType::value_type expected_channel;
  bool expected_detected;
};

class TargetTrafficSignalChannelValidTest : public ::testing::TestWithParam<ValidParseTestCase>
{
};

TEST_P(TargetTrafficSignalChannelValidTest, Parse)
{
  const auto & parameter = GetParam();
  const TargetTrafficSignalChannel target(parameter.input);
  EXPECT_EQ(target.id, parameter.expected_id);
  EXPECT_EQ(target.channel.value, parameter.expected_channel);
  EXPECT_EQ(target.detected, parameter.expected_detected);
}

// clang-format off
INSTANTIATE_TEST_SUITE_P(
  ValidCases, TargetTrafficSignalChannelValidTest,
  ::testing::Values(
    ValidParseTestCase{"34802",                       34802,      TrafficSignalChannelType::conventional, false },
    ValidParseTestCase{"34802 v2i",                   34802,      TrafficSignalChannelType::v2i,          false },
    ValidParseTestCase{"34802    v2i",                34802,      TrafficSignalChannelType::v2i,          false },
    ValidParseTestCase{"34802 conventional",          34802,      TrafficSignalChannelType::conventional, false },
    ValidParseTestCase{"34802 conventional_detected", 34802,      TrafficSignalChannelType::conventional, true  },
    ValidParseTestCase{"34802 v2i_detected",          34802,      TrafficSignalChannelType::v2i,          true  },
    ValidParseTestCase{"1",                           1,          TrafficSignalChannelType::conventional, false },
    ValidParseTestCase{"9999999999",                  9999999999, TrafficSignalChannelType::conventional, false },
    ValidParseTestCase{"-1",                          -1,         TrafficSignalChannelType::conventional, false }));
// clang-format on

class TargetTrafficSignalChannelInvalidTest : public ::testing::TestWithParam<std::string>
{
};

TEST_P(TargetTrafficSignalChannelInvalidTest, ThrowsError)
{
  EXPECT_THROW(TargetTrafficSignalChannel{GetParam()}, openscenario_interpreter::Error);
}

// clang-format off
INSTANTIATE_TEST_SUITE_P(
  InvalidCases, TargetTrafficSignalChannelInvalidTest,
  ::testing::Values(
    "",
    "invalid",
    "34802 invalid",
    "34802 invalid_detected",
    "34802 v2i extra"));
// clang-format on

class TrafficSignalChannelTypeValidTest
: public ::testing::TestWithParam<std::pair<std::string, TrafficSignalChannelType::value_type>>
{
};

TEST_P(TrafficSignalChannelTypeValidTest, FromString)
{
  const auto & [input, expected] = GetParam();
  const TrafficSignalChannelType channel_type(input);
  EXPECT_EQ(channel_type.value, expected);
}

INSTANTIATE_TEST_SUITE_P(
  ValidCases, TrafficSignalChannelTypeValidTest,
  ::testing::Values(
    std::make_pair("conventional", TrafficSignalChannelType::conventional),
    std::make_pair("v2i", TrafficSignalChannelType::v2i)));

TEST(TrafficSignalChannelType, FromStringInvalid)
{
  EXPECT_THROW(TrafficSignalChannelType("invalid"), openscenario_interpreter::Error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
