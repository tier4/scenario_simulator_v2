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
using TrafficSignalType = TrafficSignalState::TrafficSignalType;

struct ValidParseTestCase
{
  std::string input;
  lanelet::Id expected_id;
  TrafficSignalType::value_type expected_type;
  bool expected_detected;
};

class ParseTrafficSignalIdValidTest : public ::testing::TestWithParam<ValidParseTestCase>
{
};

TEST_P(ParseTrafficSignalIdValidTest, Parse)
{
  const auto & parameter = GetParam();
  const auto [id, type, detected] = TrafficSignalState::parseTrafficSignalId(parameter.input);
  EXPECT_EQ(id, parameter.expected_id);
  EXPECT_EQ(type.value, parameter.expected_type);
  EXPECT_EQ(detected, parameter.expected_detected);
}

// clang-format off
INSTANTIATE_TEST_SUITE_P(
  ValidCases, ParseTrafficSignalIdValidTest,
  ::testing::Values(
    ValidParseTestCase{"34802",                       34802,      TrafficSignalType::conventional, false },
    ValidParseTestCase{"34802 v2i",                   34802,      TrafficSignalType::v2i,          false },
    ValidParseTestCase{"34802    v2i",                34802,      TrafficSignalType::v2i,          false },
    ValidParseTestCase{"34802 conventional",          34802,      TrafficSignalType::conventional, false },
    ValidParseTestCase{"34802 conventional_detected", 34802,      TrafficSignalType::conventional, true  },
    ValidParseTestCase{"34802 v2i_detected",          34802,      TrafficSignalType::v2i,          true  },
    ValidParseTestCase{"1",                           1,          TrafficSignalType::conventional, false },
    ValidParseTestCase{"9999999999",                  9999999999, TrafficSignalType::conventional, false },
    ValidParseTestCase{"-1",                          -1,         TrafficSignalType::conventional, false }));
// clang-format on

class ParseTrafficSignalIdInvalidTest : public ::testing::TestWithParam<std::string>
{
};

TEST_P(ParseTrafficSignalIdInvalidTest, ThrowsError)
{
  EXPECT_THROW(
    TrafficSignalState::parseTrafficSignalId(GetParam()), openscenario_interpreter::Error);
}

// clang-format off
INSTANTIATE_TEST_SUITE_P(
  InvalidCases, ParseTrafficSignalIdInvalidTest,
  ::testing::Values(
    "",
    "invalid",
    "34802 invalid",
    "34802 invalid_detected",
    "34802 v2i extra",
    " 34802",
    "34802 v2i "));
// clang-format on

class TrafficSignalTypeValidTest
: public ::testing::TestWithParam<std::pair<std::string, TrafficSignalType::value_type>>
{
};

TEST_P(TrafficSignalTypeValidTest, FromString)
{
  const auto & [input, expected] = GetParam();
  TrafficSignalType type(input);
  EXPECT_EQ(type.value, expected);
}

INSTANTIATE_TEST_SUITE_P(
  ValidCases, TrafficSignalTypeValidTest,
  ::testing::Values(
    std::make_pair("conventional", TrafficSignalType::conventional),
    std::make_pair("v2i", TrafficSignalType::v2i)));

TEST(TrafficSignalType, FromStringInvalid)
{
  EXPECT_THROW(TrafficSignalType("invalid"), openscenario_interpreter::Error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
