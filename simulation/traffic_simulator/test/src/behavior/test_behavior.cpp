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

#include <traffic_simulator/data_type/behavior.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/**
 * @note 
 */
TEST(Behavior, getRequestString_none)
{
  const auto req = traffic_simulator::behavior::Request::NONE;
  std::string str;
  EXPECT_NO_THROW(str = traffic_simulator::behavior::getRequestString(req));
  EXPECT_TRUE("none" == str);
}

/**
 * @note 
 */
TEST(Behavior, getRequestString_lane_change)
{
  const auto req = traffic_simulator::behavior::Request::LANE_CHANGE;
  std::string str;
  EXPECT_NO_THROW(str = traffic_simulator::behavior::getRequestString(req));
  EXPECT_TRUE("lane_change" == str);
}

/**
 * @note 
 */
TEST(Behavior, getRequestString_follow_lane)
{
  const auto req = traffic_simulator::behavior::Request::FOLLOW_LANE;
  std::string str;
  EXPECT_NO_THROW(str = traffic_simulator::behavior::getRequestString(req));
  EXPECT_TRUE("follow_lane" == str);
}

/**
 * @note 
 */
TEST(Behavior, getRequestString_follow_polyline_trajectory)
{
  const auto req = traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY;
  std::string str;
  EXPECT_NO_THROW(str = traffic_simulator::behavior::getRequestString(req));
  EXPECT_TRUE("follow_polyline_trajectory" == str);
}

/**
 * @note 
 */
TEST(Behavior, getRequestString_walk_straight)
{
  const auto req = traffic_simulator::behavior::Request::WALK_STRAIGHT;
  std::string str;
  EXPECT_NO_THROW(str = traffic_simulator::behavior::getRequestString(req));
  EXPECT_TRUE("walk_straight" == str);
}
