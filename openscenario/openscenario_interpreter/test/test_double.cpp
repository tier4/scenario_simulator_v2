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

#include <cmath>
#include <limits>
#include <sstream>

#include "openscenario_interpreter/syntax/double.hpp"

namespace
{
openscenario_interpreter::Double parseDouble(const std::string & string)
{
  std::stringstream stream(string);
  openscenario_interpreter::Double value;
  stream >> value;
  return value;
}
}  // namespace

class DoubleInfTest : public ::testing::TestWithParam<std::pair<std::string, double>>
{
};

TEST_P(DoubleInfTest, ParseInfinity)
{
  auto [input, expected] = GetParam();
  auto value = parseDouble(input);
  EXPECT_TRUE(std::isinf(value.data));
  EXPECT_DOUBLE_EQ(value.data, expected);
}

INSTANTIATE_TEST_SUITE_P(
  InfValues, DoubleInfTest,
  ::testing::Values(
    std::make_pair("INF", std::numeric_limits<double>::infinity()),
    std::make_pair("+INF", std::numeric_limits<double>::infinity()),
    std::make_pair("-INF", -std::numeric_limits<double>::infinity())));

class DoubleNumericParseTest : public ::testing::TestWithParam<std::pair<std::string, double>>
{
};

TEST_P(DoubleNumericParseTest, ParseNumber)
{
  auto [input, expected] = GetParam();
  auto value = parseDouble(input);
  EXPECT_DOUBLE_EQ(value.data, expected);
}

INSTANTIATE_TEST_SUITE_P(
  NumericValues, DoubleNumericParseTest,
  ::testing::Values(
    // clang-format off
    std::make_pair("123.45", 123.45),
    std::make_pair("-67.89", -67.89),
    std::make_pair("0", 0.0),
    std::make_pair("-0", -0.0),
    std::make_pair("1.23e10", 1.23e10),
    std::make_pair("-4.56e-5", -4.56e-5),
    std::make_pair("1.7976931348623157e+308", 1.7976931348623157e+308),
    std::make_pair("2.2250738585072014e-308", 2.2250738585072014e-308)));
// clang-format on

TEST(DoubleStaticMethods, StaticMethods)
{
  EXPECT_TRUE(std::isinf(openscenario_interpreter::Double::infinity().data));
  EXPECT_TRUE(std::isnan(openscenario_interpreter::Double::nan().data));
  EXPECT_DOUBLE_EQ(
    openscenario_interpreter::Double::max().data, std::numeric_limits<double>::max());
  EXPECT_DOUBLE_EQ(
    openscenario_interpreter::Double::lowest().data, std::numeric_limits<double>::lowest());
}

TEST(DoubleOperators, AssignmentAndArithmetic)
{
  openscenario_interpreter::Double value;
  value = 10.0;
  EXPECT_DOUBLE_EQ(value.data, 10.0);
  value += 5.5;
  EXPECT_DOUBLE_EQ(value.data, 15.5);
  value *= 2.0;
  EXPECT_DOUBLE_EQ(value.data, 31.0);
}

TEST(DoubleOperators, OStream)
{
  std::stringstream value;
  value << openscenario_interpreter::Double(123.456);
  EXPECT_TRUE(value.str().find("123.456") == 0);

  std::stringstream inf;
  inf << openscenario_interpreter::Double::infinity();
  EXPECT_EQ(inf.str(), "inf");

  std::stringstream negative_inf;
  negative_inf << parseDouble("-INF");
  EXPECT_EQ(negative_inf.str(), "-inf");
}

TEST(DoubleSpecialCases, InvalidString)
{
  std::stringstream not_a_number("not_a_number");
  openscenario_interpreter::Double value;
  EXPECT_THROW(not_a_number >> value, std::exception);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
