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

#include <scenario_simulator_exception/exception.hpp>
#include <sstream>

#include "openscenario_interpreter/syntax/priority.hpp"

using openscenario_interpreter::syntax::Priority;

class PriorityOperatorTest
: public ::testing::TestWithParam<std::pair<std::string, Priority::value_type>>
{
};

TEST_P(PriorityOperatorTest, ParseAndOutput)
{
  auto [input_string, input_value] = GetParam();

  std::stringstream input_stream(input_string);
  Priority input_parsed;
  input_stream >> input_parsed;
  EXPECT_EQ(input_parsed.value, input_value);

  std::stringstream output_stream;
  Priority output_priority;
  output_priority.value = input_value;
  output_stream << output_priority;
  EXPECT_EQ(output_stream.str(), input_string);
}

// clang-format off
INSTANTIATE_TEST_SUITE_P(
  ValidValues, PriorityOperatorTest,
  ::testing::Values(
    std::make_pair("override", Priority::override_),
    std::make_pair("overwrite", Priority::overwrite),
    std::make_pair("skip", Priority::skip),
    std::make_pair("parallel", Priority::parallel)));
// clang-format on

class PriorityInvalidTest : public ::testing::TestWithParam<std::string>
{
};

TEST_P(PriorityInvalidTest, ThrowOnInvalidValue)
{
  auto input = GetParam();
  std::stringstream input_stream(input);
  Priority priority;
  EXPECT_THROW(input_stream >> priority, common::SyntaxError);
}

// clang-format off
INSTANTIATE_TEST_SUITE_P(
  InvalidValues, PriorityInvalidTest,
  ::testing::Values(
    "invalid",
    "OVERRIDE",
    "OVERWRITE",
    "SKIP",
    "PARALLEL",
    "Override",
    "Overwrite",
    "Skip",
    "Parallel"));
// clang-format on

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
