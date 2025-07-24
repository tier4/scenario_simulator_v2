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

TEST(ParsedTrafficSignalID, ValidId)
{
  const std::string traffic_signal_id = "34802";
  openscenario_interpreter::syntax::TrafficSignalState::ParsedTrafficSignalID parsed(
    traffic_signal_id);

  EXPECT_EQ(parsed.lanelet_id, 34802);
  EXPECT_EQ(
    parsed.traffic_signal_type,
    openscenario_interpreter::syntax::TrafficSignalState::TrafficSignalType::CONVENTIONAL);
}

TEST(ParsedTrafficSignalID, ValidIdWithV2IType)
{
  const std::string traffic_signal_id = "34802 v2i";
  openscenario_interpreter::syntax::TrafficSignalState::ParsedTrafficSignalID parsed(
    traffic_signal_id);

  EXPECT_EQ(parsed.lanelet_id, 34802);
  EXPECT_EQ(
    parsed.traffic_signal_type,
    openscenario_interpreter::syntax::TrafficSignalState::TrafficSignalType::V2I);
}

TEST(ParsedTrafficSignalID, ValidIdWithMultipleSpaces)
{
  const std::string traffic_signal_id = "34802    v2i";
  openscenario_interpreter::syntax::TrafficSignalState::ParsedTrafficSignalID parsed(
    traffic_signal_id);

  EXPECT_EQ(parsed.lanelet_id, 34802);
  EXPECT_EQ(
    parsed.traffic_signal_type,
    openscenario_interpreter::syntax::TrafficSignalState::TrafficSignalType::V2I);
}

TEST(ParsedTrafficSignalID, EmptyString)
{
  const std::string traffic_signal_id = "";
  EXPECT_THROW(
    {
      openscenario_interpreter::syntax::TrafficSignalState::ParsedTrafficSignalID parsed(
        traffic_signal_id);
    },
    openscenario_interpreter::Error);
}

TEST(ParsedTrafficSignalID, InvalidId)
{
  const std::string traffic_signal_id = "invalid";
  EXPECT_THROW(
    {
      openscenario_interpreter::syntax::TrafficSignalState::ParsedTrafficSignalID parsed(
        traffic_signal_id);
    },
    openscenario_interpreter::Error);
}

TEST(ParsedTrafficSignalID, InvalidType)
{
  const std::string traffic_signal_id = "34802 invalid";
  EXPECT_THROW(
    {
      openscenario_interpreter::syntax::TrafficSignalState::ParsedTrafficSignalID parsed(
        traffic_signal_id);
    },
    openscenario_interpreter::Error);
}

TEST(ParsedTrafficSignalID, TooManyParts)
{
  const std::string traffic_signal_id = "34802 v2i extra";
  EXPECT_THROW(
    {
      openscenario_interpreter::syntax::TrafficSignalState::ParsedTrafficSignalID parsed(
        traffic_signal_id);
    },
    openscenario_interpreter::Error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
