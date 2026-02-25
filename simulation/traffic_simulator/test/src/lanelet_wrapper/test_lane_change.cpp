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

#include "test_lanelet_wrapper.hpp"

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace traffic_simulator::lanelet_wrapper::tests
{
/**
 * @note Testcase for countLaneChanges() function
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, CountLaneChangesAlongRoute)
{
  using helper::constructLaneletPose;
  RoutingConfiguration lane_changeable_routing_configuration;
  lane_changeable_routing_configuration.allow_lane_change = true;
  EXPECT_EQ(
    lane_change::countLaneChanges(3002176, 3002175, lane_changeable_routing_configuration),
    std::make_pair(1, 0));
  EXPECT_EQ(
    lane_change::countLaneChanges(3002176, 3002182, lane_changeable_routing_configuration),
    std::make_pair(1, 0));
  EXPECT_EQ(
    lane_change::countLaneChanges(3002176, 199, lane_changeable_routing_configuration),
    std::make_pair(1, 0));
  EXPECT_EQ(
    lane_change::countLaneChanges(3002176, 3002176, lane_changeable_routing_configuration),
    std::make_pair(0, 0));
  EXPECT_EQ(
    lane_change::countLaneChanges(3002176, 200, lane_changeable_routing_configuration),
    std::make_pair(0, 0));
  EXPECT_EQ(
    lane_change::countLaneChanges(3002176, 201, lane_changeable_routing_configuration),
    std::make_pair(0, 1));
  EXPECT_EQ(
    lane_change::countLaneChanges(3002176, 202, lane_changeable_routing_configuration),
    std::make_pair(0, 2));
  EXPECT_EQ(
    lane_change::countLaneChanges(3002176, 206, lane_changeable_routing_configuration),
    std::make_pair(0, 2));
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining with a lanelet
 * that has no changeable lanelets and direction = STRAIGHT.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getLaneChangeableLaneletId_straight)
{
  const lanelet::Id start_and_end_lanelet = 199;
  const auto result_lanelet =
    lane_change::laneChangeableLaneletId(start_and_end_lanelet, lane_change::Direction::STRAIGHT);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(start_and_end_lanelet, result_lanelet);
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining
 * with a lanelet that has no changeable lanelets and direction = LEFT.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getLaneChangeableLaneletId_leftNoChangeable)
{
  EXPECT_FALSE(lane_change::laneChangeableLaneletId(199, lane_change::Direction::LEFT).has_value());
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining with
 * a lanelet that has changeable lanelets (left direction) and direction = LEFT.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getLaneChangeableLaneletId_leftChangeable)
{
  const auto result_lanelet =
    lane_change::laneChangeableLaneletId(200, lane_change::Direction::LEFT);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), 199);
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining
 * with a lanelet that has no changeable lanelets and direction = RIGHT.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getLaneChangeableLaneletId_rightNoChangeable)
{
  EXPECT_FALSE(
    lane_change::laneChangeableLaneletId(202, lane_change::Direction::RIGHT).has_value());
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining with
 * a lanelet that has changeable lanelets (right direction) and direction = RIGHT.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getLaneChangeableLaneletId_rightChangeable)
{
  const auto result_lanelet =
    lane_change::laneChangeableLaneletId(200, lane_change::Direction::RIGHT);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), 201);
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining
 * with a lanelet that has at least two changeable lanes to the left,
 * direction = LEFT and shift = 2.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getLaneChangeableLaneletId_shift2LeftPossible)
{
  const auto result_lanelet =
    lane_change::laneChangeableLaneletId(201, lane_change::Direction::LEFT, 2);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), 199);
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining
 * with a lanelet that has 1 changeable lane to the left, direction = LEFT and shift = 2
 * - the goal is to test the branch where we expect lanelet id
 * for shifting 2 times left, but shifting 2 lanes is not possible.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getLaneChangeableLaneletId_shift2LeftNotPossible)
{
  EXPECT_FALSE(
    lane_change::laneChangeableLaneletId(200, lane_change::Direction::LEFT, 2).has_value());
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining
 * with a lanelet that has at least two changeable lanes to the right,
 * direction = RIGHT and shift = 2.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getLaneChangeableLaneletId_shift2RightPossible)
{
  const auto result_lanelet =
    lane_change::laneChangeableLaneletId(200, lane_change::Direction::RIGHT, 2);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), 202);
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining
 * with a lanelet that has 1 changeable lane to the right,
 * direction = RIGHT and shift = 2 - the goal is to test the branch where
 * we expect lanelet id for shifting 2 times right, but shifting 2 lanes is not possible.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getLaneChangeableLaneletId_shift2RightNotPossible)
{
  EXPECT_FALSE(
    lane_change::laneChangeableLaneletId(201, lane_change::Direction::RIGHT, 2).has_value());
}

/**
 * @note Test function behavior when called with a direction = RIGHT and shift = 0.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getLaneChangeableLaneletId_shift0)
{
  const lanelet::Id start_and_end_lanelet = 201;
  const auto result_lanelet =
    lane_change::laneChangeableLaneletId(start_and_end_lanelet, lane_change::Direction::RIGHT, 0);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), start_and_end_lanelet);
}

/**
 * @note Test basic functionality.
 * Test lane change possibility checking
 * correctness with lanelets that can be changed.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, canChangeLane_canChange)
{
  EXPECT_TRUE(lane_change::canChangeLane(199, 200));
}

/**
 * @note Test basic functionality.
 * Test lane change possibility checking correctness with lanelets
 * that can not be changed (e.g. goal lanelet is behind the start lanelet).
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, canChangeLane_canNotChange)
{
  EXPECT_FALSE(lane_change::canChangeLane(199, 201));
}

/**
 * @note Test function behavior when either of the lanelet ids is invalid.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, canChangeLane_invalidLaneletId)
{
  EXPECT_THROW(lane_change::canChangeLane(1000003, 1000033), std::runtime_error);
}
}  // namespace traffic_simulator::lanelet_wrapper::tests
