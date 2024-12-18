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
 * @note Test basic functionality.
 * Test following lanelets obtaining with
 * a lanelet that has lanelets after it longer than parameter distance.
 */
TEST_F(LaneletWrapperTest_StandardMap, getFollowingLanelets_straightAfter)
{
  const lanelet::Id id = 120660;
  EXPECT_EQ(route::followingLanelets(id, 1.0, true), (lanelet::Ids{id, 34468}));
}

/**
 * @note Test basic functionality.
 * Test following lanelets obtaining with a lanelet
 * that has lanelets after it longer than parameter distance, but the following lanelets
 * go through a curve (e.g there was an order to go right earlier on the lane).
 */
TEST_F(LaneletWrapperTest_StandardMap, getFollowingLanelets_curveAfter)
{
  const lanelet::Id id = 34564;
  EXPECT_EQ(route::followingLanelets(id, 40.0, true), (lanelet::Ids{id, 34411, 34462}));
}

/**
 * @note Test basic functionality.
 * Test following lanelets obtaining with a lanelet
 * that has lanelets after it for less than specified in the distance parameter
 * - the goal is for the function to return trajectory shorter than distance specified.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getFollowingLanelets_notEnoughLaneletsAfter)
{
  const lanelet::Id id = 199;
  EXPECT_EQ(route::followingLanelets(id, 1.0e3, true), (lanelet::Ids{id, 203}));
}

/**
 * @note Test basic functionality.
 * Test following lanelets obtaining
 * with a candidate trajectory longer than the given distance.
 */
TEST_F(LaneletWrapperTest_StandardMap, getFollowingLanelets_candidateTrajectory)
{
  const lanelet::Id id = 34564;
  EXPECT_EQ(
    route::followingLanelets(id, lanelet::Ids{id, 34495, 34507, 34795, 34606}, 40.0, true),
    (lanelet::Ids{id, 34495, 34507}));
}

/**
 * @note Test basic functionality.
 * Test following lanelets obtaining
 * with a candidate trajectory longer than the given distance without starting lanelet.
 */
TEST_F(LaneletWrapperTest_StandardMap, getFollowingLanelets_candidateTrajectoryFalse)
{
  const lanelet::Id id = 34564;
  EXPECT_EQ(
    route::followingLanelets(id, lanelet::Ids{id, 34495, 34507, 34795, 34606}, 40.0, false),
    (lanelet::Ids{34495, 34507}));
}

/**
 * @note Test basic functionality.
 * Test following lanelets obtaining with
 * a candidate trajectory shorter than the given distance
 * - the goal is to test generating lacking part of the trajectory.
 */
TEST_F(LaneletWrapperTest_StandardMap, getFollowingLanelets_candidateTrajectoryNotEnough)
{
  const lanelet::Id id = 34564;
  EXPECT_EQ(
    route::followingLanelets(id, lanelet::Ids{id, 34495, 34507}, 100.0, true),
    (lanelet::Ids{id, 34495, 34507, 34795, 34606}));
}

/**
 * @note Test function behavior when called with a candidate trajectory
 * that does not contain the starting lanelet.
 */
TEST_F(LaneletWrapperTest_StandardMap, getFollowingLanelets_candidatesDoNotMatch)
{
  EXPECT_THROW(route::followingLanelets(120660, lanelet::Ids{34981}, 1.0e3, true), common::Error);
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(LaneletWrapperTest_StandardMap, getFollowingLanelets_candidateTrajectoryEmpty)
{
  EXPECT_EQ(route::followingLanelets(120660, {}, 1.0e3, true).size(), static_cast<std::size_t>(0));
}

/**
 * @note Test function behavior when called with a candidate trajectory
 * that contains wrong candidates
 */
TEST_F(LaneletWrapperTest_StandardMap, getFollowingLanelets_candidatesDoNotMatchRealTrajectory)
{
  EXPECT_THROW(
    route::followingLanelets(34564, lanelet::Ids{34564, 34495, 34507, 34399, 34399}, 100.0, true),
    common::Error);
}

/**
 * @note Test basic functionality.
 * Test route obtaining correctness with a feasible route.
 */
TEST_F(LaneletWrapperTest_StandardMap, getRoute_correct)
{
  RoutingConfiguration lane_changeable_routing_configuration;
  lane_changeable_routing_configuration.allow_lane_change = true;
  EXPECT_EQ(
    route::routeFromGraph(34579, 34630, lane_changeable_routing_configuration),
    (lanelet::Ids{34579, 34774, 120659, 120660, 34468, 34438, 34408, 34624, 34630}));
}

/**
 * @note Test basic functionality.
 * Test route obtaining correctness with a feasible route and obtain it two times
 * - the goal is to test whether the route cache works correctly.
 */
TEST_F(LaneletWrapperTest_StandardMap, getRoute_correctCache)
{
  const lanelet::Id from_id = 34579;
  const lanelet::Id to_id = 34630;
  RoutingConfiguration lane_changeable_routing_configuration;
  lane_changeable_routing_configuration.allow_lane_change = true;

  EXPECT_EQ(
    route::routeFromGraph(from_id, to_id, lane_changeable_routing_configuration),
    route::routeFromGraph(from_id, to_id, lane_changeable_routing_configuration));
}

/**
 * @note Test basic functionality.
 * Test route obtaining correctness with the beginning
 * and ending that are impossible to route between.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getRoute_impossibleRouting)
{
  RoutingConfiguration lane_changeable_routing_configuration;
  lane_changeable_routing_configuration.allow_lane_change = true;
  EXPECT_EQ(
    route::routeFromGraph(199, 196, lane_changeable_routing_configuration).size(),
    static_cast<std::size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test route obtaining correctness with beginning
 * and ending of the route set to the same lanelet id.
 */
TEST_F(LaneletWrapperTest_StandardMap, getRoute_circular)
{
  const lanelet::Id from_and_to_id = 120659;

  EXPECT_EQ(
    route::routeFromGraph(from_and_to_id, from_and_to_id, RoutingConfiguration()),
    lanelet::Ids{from_and_to_id});
}

/**
 * @note Test basic functionality.
 * Test speed limit obtaining correctness
 * with ids of lanelets that have different speed limits.
 */
TEST_F(LaneletWrapperTest_StandardMap, getSpeedLimit_correct)
{
  EXPECT_NEAR(route::speedLimit(lanelet::Ids{34600, 34675}), 50.0 / 3.6, 0.01);
}

/**
 * @note Test function behavior when crosswalk lanelet id is included in the vector.
 */
TEST_F(LaneletWrapperTest_StandardMap, getSpeedLimit_crosswalk)
{
  EXPECT_NEAR(route::speedLimit(lanelet::Ids{34399, 34385, 34600, 34675}), 0.0 / 3.6, 0.01);
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(LaneletWrapperTest_StandardMap, getSpeedLimit_empty)
{
  EXPECT_THROW(route::speedLimit(lanelet::Ids{}), std::runtime_error);
}

/**
 * @note Test basic functionality.
 * Test on route checking correctness
 * with a route and a lanelet that is on the route.
 */
TEST_F(LaneletWrapperTest_StandardMap, isInRoute_onRoute)
{
  EXPECT_TRUE(route::isInRoute(34850, lanelet::Ids{34741, 34850, 34603, 34777}));
}

/**
 * @note Test basic functionality.
 * Test on route checking correctness
 * with a route and a lanelet that is not on the route.
 */
TEST_F(LaneletWrapperTest_StandardMap, isInRoute_notOnRoute)
{
  EXPECT_FALSE(route::isInRoute(34468, lanelet::Ids{34741, 34850, 34603, 34777}));
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(LaneletWrapperTest_StandardMap, isInRoute_empty)
{
  EXPECT_FALSE(route::isInRoute(34468, lanelet::Ids{}));
}

/**
 * @note Test basic functionality.
 */
TEST_F(LaneletWrapperTest_StandardMap, getPreviousLanelets)
{
  const lanelet::Id id = 34600;
  const auto result_previous = route::previousLanelets(id, 100.0);
  const lanelet::Ids actual_previous{id, 34783, 34606, 34795, 34507};

  EXPECT_EQ(result_previous, actual_previous);
}

TEST_F(LaneletWrapperTest_WithRoadShoulderMap, routingWithRoadShoulder)
{
  RoutingConfiguration routing_configuration_without_road_shoulder;
  routing_configuration_without_road_shoulder.routing_graph_type = RoutingGraphType::VEHICLE;
  const auto route_without_road_shoulder =
    route::routeFromGraph(34693, 34615, routing_configuration_without_road_shoulder);
  EXPECT_EQ(route_without_road_shoulder.size(), 0);

  // default: RoutingGraphType::VEHICLE_WITH_ROAD_SHOULDER
  const auto route_with_road_shoulder = route::routeFromGraph(34693, 34615, RoutingConfiguration());
  EXPECT_EQ(route_with_road_shoulder.size(), 4);
  EXPECT_EQ(route_with_road_shoulder[0], 34693);
  EXPECT_EQ(route_with_road_shoulder[1], 34696);  // road shoulder
  EXPECT_EQ(route_with_road_shoulder[2], 34768);  // road shoulder
  EXPECT_EQ(route_with_road_shoulder[3], 34615);
}
}  // namespace traffic_simulator::lanelet_wrapper::tests
