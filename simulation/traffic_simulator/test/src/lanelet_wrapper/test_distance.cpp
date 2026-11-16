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
 * Test lateral distance calculation correctness
 * with two lanelet poses on the same lanelet but with different offsets.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getLateralDistance_sameLane)
{
  const auto from = helper::constructLaneletPose(3002185, 0.0, 0.5);
  const auto to = helper::constructLaneletPose(3002185, 10.0, 0.2);
  const auto result = distance::lateralDistance(from, to);

  EXPECT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), to.offset - from.offset, 1e-3);
}

/**
 * @note Test basic functionality.
 * Test lateral distance calculation correctness
 * with two lanelet poses on parallel lanes with no possibility of changing the lane.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getLateralDistance_parallelLanesCanNotChange)
{
  EXPECT_FALSE(distance::lateralDistance(
                 helper::constructLaneletPose(3002185, 0.0, 0.5),
                 helper::constructLaneletPose(3002184, 10.0, 0.2))
                 .has_value());
}

/**
 * @note Test basic functionality.
 * Test lateral distance calculation correctness
 * with two lanelet poses on parallel lanes with a possibility of changing the lane.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getLateralDistance_parallelLanesCanChange)
{
  const auto from = helper::constructLaneletPose(3002185, 0.0, 0.5);
  const auto to = helper::constructLaneletPose(3002184, 10.0, 0.2);

  RoutingConfiguration lane_changeable_routing_configuration;
  lane_changeable_routing_configuration.allow_lane_change = true;
  const auto result = distance::lateralDistance(from, to, lane_changeable_routing_configuration);

  EXPECT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), 2.80373 / 2.0 + 3.03463 / 2.0 + to.offset - from.offset, 1e-3);
}

/**
 * @note Test basic functionality.
 * Test lateral distance calculation correctness
 * with two poses on lanelets that are not connected - the goal is to test
 * the scenario when the distance cannot be calculated because two positions
 * will never be able to come in contact.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getLateralDistance_notConnected)
{
  RoutingConfiguration lane_changeable_routing_configuration;
  lane_changeable_routing_configuration.allow_lane_change = true;
  EXPECT_FALSE(distance::lateralDistance(
                 helper::constructLaneletPose(3002185, 0.0, 0.5),
                 helper::constructLaneletPose(3002166, 10.0, 0.2),
                 lane_changeable_routing_configuration)
                 .has_value());
}

/**
 * @note Test basic functionality.
 * Test longitudinal distance calculation correctness
 * with two poses on the same lanelet, where the goal pose is positioned in front of the start pose.
 */
TEST_F(LaneletWrapperTest_StandardMap, getLongitudinalDistance_sameLanelet)
{
  const auto pose_from = pose::toLaneletPose(
    makePose(makePoint(3812.65, 73810.13, -2.80), makeQuaternionFromYaw(90.0)), lanelet::Id{34606});
  const auto pose_to = pose::toLaneletPose(
    makePose(makePoint(3825.10, 73786.34, -1.82), makeQuaternionFromYaw(90.0)), lanelet::Id{34606});
  ASSERT_TRUE(pose_from.has_value());
  ASSERT_TRUE(pose_to.has_value());

  const auto result_distance =
    distance::longitudinalDistance(pose_from.value(), pose_to.value(), RoutingConfiguration());

  ASSERT_TRUE(result_distance.has_value());
  EXPECT_NEAR(result_distance.value(), 27.0, 1.0);
}

/**
 * @note Test basic functionality.
 * Test longitudinal distance calculation correctness
 * with two poses on the same lanelet, where the goal pose is positioned behind the start pose.
 */
TEST_F(LaneletWrapperTest_StandardMap, getLongitudinalDistance_sameLaneletBehind)
{
  const auto pose_to = pose::toLaneletPose(
    makePose(makePoint(3812.65, 73810.13, -2.80), makeQuaternionFromYaw(90.0)), lanelet::Id{34606});
  const auto pose_from = pose::toLaneletPose(
    makePose(makePoint(3825.10, 73786.34, -1.82), makeQuaternionFromYaw(90.0)), lanelet::Id{34606});
  ASSERT_TRUE(pose_from.has_value());
  ASSERT_TRUE(pose_to.has_value());

  const auto longitudinal_distance =
    distance::longitudinalDistance(pose_from.value(), pose_to.value(), RoutingConfiguration());
  EXPECT_FALSE(longitudinal_distance.has_value());
}

/**
 * @note Test basic functionality.
 * Test longitudinal distance calculation correctness
 * with two poses on different lanelets that are a few lanelets apart (e.g. 3).
 */
TEST_F(LaneletWrapperTest_StandardMap, getLongitudinalDistance_differentLanelet)
{
  const auto pose_from =
    pose::toLaneletPose(makePose(makePoint(3801.19, 73812.70, -2.86)), lanelet::Id{120660});
  const auto pose_to =
    pose::toLaneletPose(makePose(makePoint(3724.70, 73773.00, -1.20)), lanelet::Id{34462});
  ASSERT_TRUE(pose_from.has_value());
  ASSERT_TRUE(pose_to.has_value());

  const auto result_distance =
    distance::longitudinalDistance(pose_from.value(), pose_to.value(), RoutingConfiguration());

  ASSERT_TRUE(result_distance.has_value());
  EXPECT_NEAR(result_distance.value(), 86.0, 1.0);
}

/**
 * @note Test basic functionality. Test longitudinal distance calculation correctness
 * with two poses on different lanelets where the goal pose is on lanelet unreachable
 * from the start pose lanelet - the goal is to test the branch of execution where no route is found.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, getLongitudinalDistance_differentLaneletNoRoute)
{
  const auto pose_to = pose::toLaneletPose(
    makePose(makePoint(81590.79, 50067.66, 35.0), makeQuaternionFromYaw(90.0)),
    lanelet::Id{3002185});
  const auto pose_from = pose::toLaneletPose(
    makePose(makePoint(81596.20, 50068.04, 35.0), makeQuaternionFromYaw(90.0)),
    lanelet::Id{3002166});
  ASSERT_TRUE(pose_from.has_value());
  ASSERT_TRUE(pose_to.has_value());

  EXPECT_FALSE(
    distance::longitudinalDistance(pose_from.value(), pose_to.value(), RoutingConfiguration())
      .has_value());
}

/**
 * @note Test for the corner-case fixed in https://github.com/tier4/scenario_simulator_v2/pull/1348.
 */
TEST_F(LaneletWrapperTest_KashiwanohaMap, getLongitudinalDistance_PullRequest1348)
{
  auto pose_from = helper::constructLaneletPose(34468, 10.0);
  auto pose_to = helper::constructLaneletPose(34795, 5.0);

  RoutingConfiguration lane_changeable_routing_configuration;
  lane_changeable_routing_configuration.allow_lane_change = true;
  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(
    distance::longitudinalDistance(pose_from, pose_to, lane_changeable_routing_configuration)
      .value(),
    54.18867466433655977198213804513216018676757812500000));
}

/**
 * @note Test for the corner case described in https://github.com/tier4/scenario_simulator_v2/issues/1364
 * Test in a scenario where lane change is necessary:
 * if allow_lane_change = false, std::nullopt should be returned;
 * if allow_lane_change = true, a value should be returned.
 */
TEST_F(LaneletWrapperTest_IntersectionMap, getLongitudinalDistance_laneChange)
{
  RoutingConfiguration lane_changeable_routing_configuration;
  lane_changeable_routing_configuration.allow_lane_change = true;
  RoutingConfiguration default_routing_configuration;
  {
    const auto pose_from = helper::constructLaneletPose(563L, 5.0);
    const auto pose_to = helper::constructLaneletPose(659L, 5.0);

    const auto without_lane_change =
      distance::longitudinalDistance(pose_from, pose_to, default_routing_configuration);
    EXPECT_FALSE(without_lane_change.has_value());

    const auto with_lane_change =
      distance::longitudinalDistance(pose_from, pose_to, lane_changeable_routing_configuration);
    ASSERT_TRUE(with_lane_change.has_value());
    EXPECT_NEAR(with_lane_change.value(), 157.0, 1.0);
  }
  {
    const auto pose_from = helper::constructLaneletPose(563L, 5.0);
    const auto pose_to = helper::constructLaneletPose(658L, 5.0);

    const auto without_lane_change =
      distance::longitudinalDistance(pose_from, pose_to, default_routing_configuration);
    EXPECT_FALSE(without_lane_change.has_value());

    const auto with_lane_change =
      distance::longitudinalDistance(pose_from, pose_to, lane_changeable_routing_configuration);
    ASSERT_TRUE(with_lane_change.has_value());
    EXPECT_NEAR(with_lane_change.value(), 161.0, 1.0);
  }
  {
    const auto pose_from = helper::constructLaneletPose(563L, 5.0);
    const auto pose_to = helper::constructLaneletPose(657L, 5.0);

    const auto without_lane_change =
      distance::longitudinalDistance(pose_from, pose_to, default_routing_configuration);
    EXPECT_FALSE(without_lane_change.has_value());

    const auto with_lane_change =
      distance::longitudinalDistance(pose_from, pose_to, lane_changeable_routing_configuration);
    ASSERT_TRUE(with_lane_change.has_value());
    EXPECT_NEAR(with_lane_change.value(), 161.0, 1.0);
  }
  {
    const auto pose_from = helper::constructLaneletPose(643L, 5.0);
    const auto pose_to = helper::constructLaneletPose(666L, 5.0);

    const auto without_lane_change =
      distance::longitudinalDistance(pose_from, pose_to, default_routing_configuration);
    EXPECT_FALSE(without_lane_change.has_value());

    const auto with_lane_change =
      distance::longitudinalDistance(pose_from, pose_to, lane_changeable_routing_configuration);
    ASSERT_TRUE(with_lane_change.has_value());
    EXPECT_NEAR(with_lane_change.value(), 250.0, 1.0);
  }
  {
    const auto pose_from = helper::constructLaneletPose(643L, 5.0);
    const auto pose_to = helper::constructLaneletPose(665L, 5.0);

    const auto without_lane_change =
      distance::longitudinalDistance(pose_from, pose_to, default_routing_configuration);
    EXPECT_FALSE(without_lane_change.has_value());

    const auto with_lane_change =
      distance::longitudinalDistance(pose_from, pose_to, lane_changeable_routing_configuration);
    ASSERT_TRUE(with_lane_change.has_value());
    EXPECT_NEAR(with_lane_change.value(), 253.0, 1.0);
  }
}

/**
 * @note Test basic functionality.
 * Test distance to traffic light stop line obtaining
 * correctness with a spline and a traffic light id that has a stop line on the spline.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_trafficLightOnSpline)
{
  const auto start_waypoint = makePoint(3771.06, 73728.35);
  const auto result_distance = distance::distanceToTrafficLightStopLine(
    math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
      start_waypoint, makePoint(3756.30, 73755.87), makePoint(3746.90, 73774.44)}),
    lanelet::Id{34836});
  EXPECT_TRUE(result_distance.has_value());

  const auto stopline_midpoint = makePoint(3767.00, 73736.47);

  EXPECT_NEAR(
    std::hypot(start_waypoint.x - stopline_midpoint.x, start_waypoint.y - stopline_midpoint.y),
    result_distance.value(), 1.0);
}

/**
 * @note Test basic functionality.
 * Test distance to traffic light stop line obtaining correctness
 * with a spline and a traffic light id that does not have a stop line on the spline.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_noTrafficLightOnSpline)
{
  EXPECT_FALSE(
    distance::distanceToTrafficLightStopLine(
      math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
        makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70), makePoint(3773.19, 73723.27)}),
      lanelet::Id{34836})
      .has_value());
}

/**
 * @note Test basic functionality.
 * Test distance to traffic light stop line obtaining correctness
 * with a road (waypoints) and a traffic light id has a stop line on the road.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_trafficLightOnWaypoints)
{
  const auto start_waypoint = makePoint(3771.06, 73728.35);
  const auto result_distance = distance::distanceToTrafficLightStopLine(
    std::vector<geometry_msgs::msg::Point>{
      start_waypoint, makePoint(3756.30, 73755.87), makePoint(3746.90, 73774.44)},
    lanelet::Id{34836});
  EXPECT_TRUE(result_distance.has_value());

  const auto stopline_midpoint = makePoint(3767.00, 73736.47);

  EXPECT_NEAR(
    std::hypot(start_waypoint.x - stopline_midpoint.x, start_waypoint.y - stopline_midpoint.y),
    result_distance.value(), 1.0);
}

/**
 * @note Test basic functionality.
 * Test distance to traffic light stop line obtaining correctness
 * with a road (waypoints) and a traffic light id that does not have a stop line on the road.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_noTrafficLightOnWaypoints)
{
  EXPECT_FALSE(

    distance::distanceToTrafficLightStopLine(
      std::vector<geometry_msgs::msg::Point>{
        makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70), makePoint(3773.19, 73723.27)},
      lanelet::Id{34836})
      .has_value());
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_emptyVector_waypoints)
{
  EXPECT_FALSE(distance::distanceToTrafficLightStopLine(
                 std::vector<geometry_msgs::msg::Point>{}, lanelet::Id{34836})
                 .has_value());
}

/**
 * @note Test basic functionality.
 * Test distance to traffic light stop line obtaining correctness
 * with a spline and a route that is coherent with the spline and has a traffic light on it.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_routeWithTrafficLightsOnSpline)
{
  const auto start_waypoint = makePoint(3771.06, 73728.35);

  const auto result_distance = distance::distanceToTrafficLightStopLine(
    lanelet::Ids{34576, 34570, 34564},
    math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
      start_waypoint, makePoint(3756.30, 73755.87), makePoint(3746.90, 73774.44)}));
  EXPECT_TRUE(result_distance.has_value());

  const auto stopline_midpoint = makePoint(3767.00, 73736.47);

  EXPECT_NEAR(
    std::hypot(start_waypoint.x - stopline_midpoint.x, start_waypoint.y - stopline_midpoint.y),
    result_distance.value(), 1.0);
}

/**
 * @note Test basic functionality.
 * Test distance to traffic light stop line obtaining correctness
 * with a spline and a route that is coherent with the spline and does not have a traffic light on it.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_routeWithNoTrafficLightsOnSplineCongruent)
{
  EXPECT_FALSE(
    distance::distanceToTrafficLightStopLine(
      lanelet::Ids{34690, 34759, 34576},
      math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
        makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70), makePoint(3773.19, 73723.27)}))
      .has_value());
}

/**
 * @note Test basic functionality.
 * Test distance to traffic light stop line obtaining correctness
 * with a spline and a route that is not coherent with the spline and has a traffic light on it
 * - the goal is to test the situation where the traffic light and its stop line are checked
 * against a spline that does not overlay with them.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_routeWithTrafficLightsNotOnSplineIncongruent)
{
  EXPECT_FALSE(
    distance::distanceToTrafficLightStopLine(
      lanelet::Ids{34576, 34570, 34564},
      math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
        makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70), makePoint(3773.19, 73723.27)}))
      .has_value());
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_emptyVector_splineRoute)
{
  EXPECT_FALSE(
    distance::distanceToTrafficLightStopLine(
      lanelet::Ids{},
      math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
        makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70), makePoint(3773.19, 73723.27)}))
      .has_value());
}

/**
 * @note Test basic functionality.
 * Test distance to traffic light stop line obtaining correctness
 * with a road (waypoints) and a route that is coherent with the road and has a traffic light on it.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_routeWithTrafficLightsOnWaypoints)
{
  const auto start_waypoint = makePoint(3771.06, 73728.35);

  const auto result_distance = distance::distanceToTrafficLightStopLine(
    lanelet::Ids{34576, 34570, 34564},
    std::vector<geometry_msgs::msg::Point>{
      start_waypoint, makePoint(3756.30, 73755.87), makePoint(3746.90, 73774.44)});
  EXPECT_TRUE(result_distance.has_value());

  const auto stopline_midpoint = makePoint(3767.00, 73736.47);

  EXPECT_NEAR(
    std::hypot(start_waypoint.x - stopline_midpoint.x, start_waypoint.y - stopline_midpoint.y),
    result_distance.value(), 1.0);
}

/**
 * @note Test basic functionality.
 * Test distance to traffic light stop line obtaining correctness
 * with a road (waypoints) and a route that is not coherent with the road
 * and has a traffic light on it - the goal is to test the situation where
 * the traffic light and its stop line are checked against a road that does not overlay with them.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_routeWithNoTrafficLightsOnWaypointsIncongruent)
{
  EXPECT_FALSE(

    distance::distanceToTrafficLightStopLine(
      lanelet::Ids{34690, 34759, 34576},
      std::vector<geometry_msgs::msg::Point>{
        makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70), makePoint(3773.19, 73723.27)})
      .has_value());
}

/**
 * @note Test basic functionality.
 * Test distance to traffic light stop line obtaining correctness with a road (waypoints)
 * and a route that is coherent with the road and does not have a traffic light on it.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_routeWithTrafficLightsNotOnWaypointsCongruent)
{
  EXPECT_FALSE(

    distance::distanceToTrafficLightStopLine(
      lanelet::Ids{34576, 34570, 34564},
      std::vector<geometry_msgs::msg::Point>{
        makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70), makePoint(3773.19, 73723.27)})
      .has_value());
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_emptyVector_waypointsRoute)
{
  EXPECT_FALSE(

    distance::distanceToTrafficLightStopLine(
      lanelet::Ids{},
      std::vector<geometry_msgs::msg::Point>{
        makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70), makePoint(3773.19, 73723.27)})
      .has_value());
}

/**
 * @note Test basic functionality.
 * Test distance to stop line calculation correctness
 * with a spline and a route that is coherent with the spline and has a stop line on it.
 */
TEST_F(LaneletWrapperTest_CrossroadsWithStoplinesMap, getDistanceToStopLine_stopLineOnSpline)
{
  const auto start_waypoint = makePoint(3821.86, 73777.20);
  const auto result_distance = distance::distanceToStopLine(
    lanelet::Ids{34780, 34675, 34744},
    math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
      start_waypoint, makePoint(3837.28, 73762.67), makePoint(3846.10, 73741.38)}));
  EXPECT_TRUE(result_distance.has_value());

  const auto stopline_midpoint = makePoint(3838.98, 73759.28);
  EXPECT_NEAR(
    std::hypot(start_waypoint.x - stopline_midpoint.x, start_waypoint.y - stopline_midpoint.y),
    result_distance.value(), 1.0);
}

/**
 * @note Test basic functionality.
 * Test distance to stop line calculation correctness
 * with a spline and a route that is coherent with the spline and does not have a stop line on it.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap, getDistanceToStopLine_noStopLineOnSplineCongruent)
{
  EXPECT_FALSE(
    distance::distanceToStopLine(
      lanelet::Ids{34690, 34759, 34576},
      math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
        makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70), makePoint(3773.19, 73723.27)}))
      .has_value());
}

/**
 * @note Test basic functionality.
 * Test distance to stop line calculation correctness
 * with a spline and a route that is not coherent with the spline and has a stop line on it
 * - the goal is to test the situation where the stop line is checked
 * against a spline that does not overlay with it.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToStopLine_noStopLineOnSplineIncongruent)
{
  EXPECT_FALSE(
    distance::distanceToStopLine(
      lanelet::Ids{34576, 34570, 34564},
      math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
        makePoint(3821.86, 73777.20), makePoint(3837.28, 73762.67), makePoint(3846.10, 73741.38)}))
      .has_value());
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(LaneletWrapperTest_CrossroadsWithStoplinesMap, getDistanceToStopLine_emptyVector_spline)
{
  EXPECT_FALSE(
    distance::distanceToStopLine(
      lanelet::Ids{},
      math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
        makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70), makePoint(3773.19, 73723.27)}))
      .has_value());
}

/**
 * @note Test basic functionality.
 * Test distance to stop line calculation correctness
 * with a road (waypoints) and a route that is coherent with the road and has a stop line on it.
 */
TEST_F(LaneletWrapperTest_CrossroadsWithStoplinesMap, getDistanceToStopLine_stopLineOnWaypoints)
{
  const auto start_waypoint = makePoint(3821.86, 73777.20);
  const auto result_distance = distance::distanceToStopLine(
    lanelet::Ids{34780, 34675, 34744},
    std::vector<geometry_msgs::msg::Point>{
      start_waypoint, makePoint(3837.28, 73762.67), makePoint(3846.10, 73741.38)});
  EXPECT_TRUE(result_distance.has_value());

  const auto stopline_midpoint = makePoint(3838.98, 73759.28);
  EXPECT_NEAR(
    std::hypot(start_waypoint.x - stopline_midpoint.x, start_waypoint.y - stopline_midpoint.y),
    result_distance.value(), 1.0);
}

/**
 * @note Test basic functionality.
 * Test distance to stop line calculation correctness with a road (waypoints)
 * and a route that is coherent with the road and does not have a stop line on it.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToStopLine_noStopLineOnWaypointsCongruent)
{
  EXPECT_FALSE(
    distance::distanceToStopLine(
      lanelet::Ids{34690, 34759, 34576},
      std::vector<geometry_msgs::msg::Point>{
        makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70), makePoint(3773.19, 73723.27)})
      .has_value());
}

/**
 * @note Test basic functionality.
 * Test distance to stop line calculation correctness
 * with a road (waypoints) and a route that is not coherent with the road
 * and has a stop line on it - the goal is to test the situation where the stop line
 * is checked against a road that does not overlay with it.
 */
TEST_F(
  LaneletWrapperTest_CrossroadsWithStoplinesMap,
  getDistanceToStopLine_noStopLineOnWaypointsIncongruent)
{
  EXPECT_FALSE(
    distance::distanceToStopLine(
      lanelet::Ids{34576, 34570, 34564},
      std::vector<geometry_msgs::msg::Point>{
        makePoint(3821.86, 73777.20), makePoint(3837.28, 73762.67), makePoint(3846.10, 73741.38)})
      .has_value());
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(LaneletWrapperTest_CrossroadsWithStoplinesMap, getDistanceToStopLine_emptyVector_waypoints)
{
  EXPECT_FALSE(
    distance::distanceToStopLine(
      lanelet::Ids{},
      std::vector<geometry_msgs::msg::Point>{
        makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70), makePoint(3773.19, 73723.27)})
      .has_value());
}
}  // namespace traffic_simulator::lanelet_wrapper::tests
