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

TEST(ParseTrafficSignalID, ValidId)
{
  const auto result = TrafficSignalState::parseTrafficSignalId("34802");
  EXPECT_EQ(result.id, 34802);
  EXPECT_EQ(result.type.value, TrafficSignalType::conventional);
  EXPECT_FALSE(result.detected);
}

TEST(ParseTrafficSignalID, ValidIdWithV2IType)
{
  const auto result = TrafficSignalState::parseTrafficSignalId("34802 v2i");
  EXPECT_EQ(result.id, 34802);
  EXPECT_EQ(result.type.value, TrafficSignalType::v2i);
  EXPECT_FALSE(result.detected);
}

TEST(ParseTrafficSignalID, ValidIdWithMultipleSpaces)
{
  const auto result = TrafficSignalState::parseTrafficSignalId("34802    v2i");
  EXPECT_EQ(result.id, 34802);
  EXPECT_EQ(result.type.value, TrafficSignalType::v2i);
  EXPECT_FALSE(result.detected);
}

TEST(ParseTrafficSignalID, EmptyString)
{
  EXPECT_THROW(TrafficSignalState::parseTrafficSignalId(""), openscenario_interpreter::Error);
}

TEST(ParseTrafficSignalID, InvalidId)
{
  EXPECT_THROW(
    TrafficSignalState::parseTrafficSignalId("invalid"), openscenario_interpreter::Error);
}

TEST(ParseTrafficSignalID, InvalidType)
{
  EXPECT_THROW(
    TrafficSignalState::parseTrafficSignalId("34802 invalid"), openscenario_interpreter::Error);
}

TEST(ParseTrafficSignalID, TooManyParts)
{
  EXPECT_THROW(
    TrafficSignalState::parseTrafficSignalId("34802 v2i extra"), openscenario_interpreter::Error);
}

TEST(TrafficSignalType, FromStringConventional)
{
  TrafficSignalType type("conventional");
  EXPECT_EQ(type.value, TrafficSignalType::conventional);
}

TEST(TrafficSignalType, FromStringV2I)
{
  TrafficSignalType type("v2i");
  EXPECT_EQ(type.value, TrafficSignalType::v2i);
}

TEST(TrafficSignalType, FromStringInvalid)
{
  EXPECT_THROW(TrafficSignalType("invalid"), openscenario_interpreter::Error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
