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
 * Test filtering correctness with some lanelet ids and a valid subtype name.
 */
TEST_F(LaneletWrapperTest_StandardMap, filterLaneletIds_correct)
{
  const lanelet::Id id_crosswalk_0 = 34399;
  const lanelet::Id id_crosswalk_1 = 34385;

  auto filtered =
    lanelet_map::filterLaneletIds({id_crosswalk_0, id_crosswalk_1, 34600, 34675}, "crosswalk");

  EXPECT_EQ(filtered.size(), static_cast<std::size_t>(2));
  EXPECT_TRUE(std::find(filtered.begin(), filtered.end(), id_crosswalk_0) != filtered.end());
  EXPECT_TRUE(std::find(filtered.begin(), filtered.end(), id_crosswalk_1) != filtered.end());
}

/**
 * @note Test function behavior when passed an empty lanelet ids vector.
 */
TEST_F(LaneletWrapperTest_StandardMap, filterLaneletIds_emptyIds)
{
  EXPECT_TRUE(lanelet_map::filterLaneletIds({}, "crosswalk").empty());
}

/**
 * @note Test function behavior when passed an invalid subtype name.
 */
TEST_F(LaneletWrapperTest_StandardMap, filterLaneletIds_invalidSubtype)
{
  EXPECT_TRUE(
    lanelet_map::filterLaneletIds({34399, 34385, 34600, 34675}, "invalid_subtype").empty());
}

/**
 * @note Test function behavior when passed a vector of invalid lanelet ids.
 */
TEST_F(LaneletWrapperTest_StandardMap, filterLaneletIds_invalidIds)
{
  EXPECT_THROW(
    auto filtered = lanelet_map::filterLaneletIds({10000000, 10000001, 10000002}, "crosswalk"),
    std::runtime_error);
}

/**
 * @note Test basic functionality.
 * Test obtaining nearest lanelet ids correctness
 * with a position in the middle of the lane and relatively big distance threshold
 * - the goal is to test successful scenario when there should be lanelets returned.
 */
TEST_F(LaneletWrapperTest_StandardMap, getNearbyLaneletIds)
{
  EXPECT_EQ(
    lanelet_map::nearbyLaneletIds(
      makePoint(3807.34, 73817.95), 10.0, false, static_cast<std::size_t>(100)),
    (lanelet::Ids{34795, 120660, 34507, 34468, 120659, 34606}));
}

/**
 * @note Test basic functionality.
 * Test obtaining nearest lanelet ids correctness
 * with a position on the side of the map and with fairly small distance threshold
 * - the goal is to test unsuccessful scenario when there should be no lanelets returned.
 */
TEST_F(LaneletWrapperTest_StandardMap, getNearbyLaneletIds_unsuccessful)
{
  EXPECT_TRUE(lanelet_map::nearbyLaneletIds(
                makePoint(3826.26, 73837.32), 10.0, false, static_cast<std::size_t>(100))
                .empty());
}

/**
 * @note Test basic functionality.
 * Test obtaining nearest lanelet ids correctness
 * (with a crosswalk) with a position on the side of the map and with fairly small distance threshold
 * - the goal is to test unsuccessful scenario when there should be no lanelets returned.
 */
TEST_F(LaneletWrapperTest_StandardMap, getNearbyLaneletIds_crosswalkUnsuccessful)
{
  EXPECT_TRUE(lanelet_map::nearbyLaneletIds(
                makePoint(3826.26, 73837.32), 10.0, true, static_cast<std::size_t>(100))
                .empty());
}

/**
 * @note Test basic functionality.
 * Test previous lanelets id obtaining correctness
 * with a lanelet that has a lanelet preceding it.
 */
TEST_F(LaneletWrapperTest_StandardMap, getPreviousLaneletIds)
{
  const auto result_ids = lanelet_map::previousLaneletIds(34468);
  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
  if (result_ids.size() == 1) {
    EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(120660));
  }
}

/**
 * @note Test basic functionality.
 * Test previous lanelets id obtaining correctness
 * with a lanelet that has a lanelet preceding it and is a shoulder lane.
 */
TEST_F(LaneletWrapperTest_WithRoadShoulderMap, getPreviousLaneletIds_RoadShoulder)
{
  const auto result_ids = lanelet_map::previousLaneletIds(34768);
  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
  if (result_ids.size() == 1) {
    EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(34696));
  }
}

/**
 * @note Test basic functionality.
 * Test previous lanelets id obtaining correctness
 * with a lanelet that has several lanelets preceding it.
 */
TEST_F(LaneletWrapperTest_StandardMap, getPreviousLaneletIds_multiplePrevious)
{
  lanelet::Ids prev_lanelets = {34411, 34465};
  auto result_ids = lanelet_map::previousLaneletIds(34462);

  std::sort(prev_lanelets.begin(), prev_lanelets.end());
  std::sort(result_ids.begin(), result_ids.end());

  EXPECT_EQ(prev_lanelets, result_ids);
}

/**
 * @note Test basic functionality.
 * Test previous lanelets id obtaining correctness
 * with a lanelet that has several lanelets preceding it and a direction specified (e.g. right)
 * - the goal is to test the function specialization that takes a direction as an argument
 * and returns only the previous lanelets that have this turn direction.
 */
TEST_F(LaneletWrapperTest_StandardMap, getPreviousLaneletIds_direction)
{
  const lanelet::Id curr_lanelet = 34462;
  const lanelet::Id prev_lanelet_left = 34411;
  const lanelet::Id prev_lanelet_straight = 34465;

  {
    const auto result_ids = lanelet_map::previousLaneletIds(curr_lanelet, "left");
    EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
    if (result_ids.size() == 1) {
      EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(prev_lanelet_left));
    }
  }
  {
    const auto result_ids = lanelet_map::previousLaneletIds(curr_lanelet, "straight");
    EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
    if (result_ids.size() == 1) {
      EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(prev_lanelet_straight));
    }
  }
}

/**
 * @note Test basic functionality.
 * Test next lanelets id obtaining correctness
 * with a lanelet that has a lanelet following it.
 */
TEST_F(LaneletWrapperTest_StandardMap, nextLaneletIds)
{
  const auto result_ids = lanelet_map::nextLaneletIds(120660);
  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
  if (result_ids.size() == 1) {
    EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(34468));
  }
}

/**
 * @note Test basic functionality.
 * Test next lanelets id obtaining correctness
 * with a lanelet that has a lanelet following it and is a shoulder lane.
 */
TEST_F(LaneletWrapperTest_WithRoadShoulderMap, nextLaneletIds_RoadShoulder)
{
  const auto result_ids = lanelet_map::nextLaneletIds(34696);
  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
  if (result_ids.size() == 1) {
    EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(34768));
  }
}

/**
 * @note Test basic functionality.
 * Test next lanelets id obtaining correctness
 * with a lanelet that has several lanelets following it.
 */
TEST_F(LaneletWrapperTest_StandardMap, nextLaneletIds_multipleNext)
{
  lanelet::Ids next_lanelets = {34438, 34465};
  auto result_ids = lanelet_map::nextLaneletIds(34468);

  std::sort(next_lanelets.begin(), next_lanelets.end());
  std::sort(result_ids.begin(), result_ids.end());

  EXPECT_EQ(next_lanelets, result_ids);
}

/**
 * @note Test basic functionality.
 * Test next lanelets id obtaining correctness
 * with a lanelet that has several lanelets following it and a direction specified (e.g. right)
 * - the goal is to test the function specialization that takes a direction as an argument
 * and returns only the next lanelets that have this turn direction.
 */
TEST_F(LaneletWrapperTest_StandardMap, nextLaneletIds_direction)
{
  const lanelet::Id curr_lanelet = 34468;

  {
    const auto result_ids = lanelet_map::nextLaneletIds(curr_lanelet, "left");
    EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
    if (result_ids.size() == 1) {
      EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(34438));
    }
  }
  {
    const auto result_ids = lanelet_map::nextLaneletIds(curr_lanelet, "straight");
    EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
    if (result_ids.size() == 1) {
      EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(34465));
    }
  }
}

/**
 * @note Test basic functionality.
 * Test in lanelet presence correctness
 * with a position that is in the given lanelet.
 */
TEST_F(LaneletWrapperTest_StandardMap, isInLanelet_correct)
{
  EXPECT_TRUE(lanelet_map::isInLanelet(34696, 10.0));
}

/**
 * @note Test basic functionality.
 * Test in lanelet presence correctness
 * with a position that is after the given lanelet.
 */
TEST_F(LaneletWrapperTest_StandardMap, isInLanelet_after)
{
  const lanelet::Id lanelet_id = 34696;
  EXPECT_FALSE(lanelet_map::isInLanelet(lanelet_id, lanelet_map::laneletLength(lanelet_id) + 5.0));
}

/**
 * @note Test basic functionality.
 * Test in lanelet presence correctness
 * with a position that is before the given lanelet.
 */
TEST_F(LaneletWrapperTest_StandardMap, isInLanelet_before)
{
  EXPECT_FALSE(lanelet_map::isInLanelet(34696, -5.0));
}

/**
 * @note Test function behavior when called with a value of s larger than the lanelet length.
 */
TEST_F(LaneletWrapperTest_StandardMap, toMapPose_sLargerThanLaneletLength)
{
  const lanelet::Id lanelet_id = 34696;

  geometry_msgs::msg::PoseStamped map_pose;
  EXPECT_NO_THROW(
    map_pose = pose::toMapPose(
      helper::constructLaneletPose(lanelet_id, lanelet_map::laneletLength(lanelet_id) + 10.0)));

  EXPECT_STREQ(map_pose.header.frame_id.c_str(), "map");
  EXPECT_POSE_NEAR(
    map_pose.pose, makePose(makePoint(3724.9, 73678.1, 2.7), makeQuaternionFromYaw(2.828)), 0.1);
}

/**
 * @note Test basic functionality.
 * Test obtaining conflicting lanelets correctness
 * with lanelets that do conflict with other lanelets.
 */
TEST_F(LaneletWrapperTest_StandardMap, getConflictingLaneIds_conflicting)
{
  lanelet::Ids actual_ids = {34495, 34498};
  auto result_ids = lanelet_map::conflictingLaneIds({34510});

  std::sort(actual_ids.begin(), actual_ids.end());
  std::sort(result_ids.begin(), result_ids.end());
  EXPECT_EQ(actual_ids, result_ids);
}

/**
 * @note Test basic functionality.
 * Test obtaining conflicting lanelets correctness
 * with lanelets that do not conflict with any other lanelets.
 */
TEST_F(LaneletWrapperTest_StandardMap, getConflictingLaneIds_notConflicting)
{
  EXPECT_EQ(lanelet_map::conflictingLaneIds({34513}).size(), static_cast<std::size_t>(0));
}

/**
 * @note Test function behavior when called with an empty vector.
 */
TEST_F(LaneletWrapperTest_StandardMap, getConflictingLaneIds_empty)
{
  EXPECT_EQ(lanelet_map::conflictingLaneIds({}).size(), static_cast<std::size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test obtaining conflicting crosswalk lanelets
 * correctness with lanelets that do conflict with crosswalk lanelets.
 */
TEST_F(LaneletWrapperTest_StandardMap, getConflictingCrosswalkIds_conflicting)
{
  lanelet::Ids actual_ids = {34399, 34385};
  auto result_ids = lanelet_map::conflictingCrosswalkIds({34633});

  std::sort(actual_ids.begin(), actual_ids.end());
  std::sort(result_ids.begin(), result_ids.end());
  EXPECT_EQ(actual_ids, result_ids);
}

/**
 * @note Test basic functionality.
 * Test obtaining conflicting crosswalk lanelets
 * correctness with lanelets that do not conflict with any crosswalk lanelets,
 * but do conflict with vehicle lanelets.
 */
TEST_F(LaneletWrapperTest_StandardMap, getConflictingCrosswalkIds_notConflictingWithCrosswalk)
{
  EXPECT_EQ(lanelet_map::conflictingCrosswalkIds({34510}).size(), static_cast<std::size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test obtaining conflicting crosswalk lanelets
 * correctness with lanelets that do not conflict with any other lanelets.
 */
TEST_F(LaneletWrapperTest_StandardMap, getConflictingCrosswalkIds_notConflicting)
{
  EXPECT_EQ(lanelet_map::conflictingCrosswalkIds({34513}).size(), static_cast<std::size_t>(0));
}

/**
 * @note Test function behavior when called with an empty vector.
 */
TEST_F(LaneletWrapperTest_StandardMap, getConflictingCrosswalkIds_empty)
{
  EXPECT_EQ(lanelet_map::conflictingCrosswalkIds({}).size(), static_cast<std::size_t>(0));
}

/**
 * @note Test basic functionality with a lanelet that has a centerline with 3 or more points.
 */
TEST_F(LaneletWrapperTest_StandardMap, getCenterPoints_correct)
{
  const std::vector<geometry_msgs::msg::Point> actual_center_points{
    makePoint(3774.1, 73748.8, -0.3), makePoint(3772.5, 73748.0, -0.2),
    makePoint(3770.8, 73747.1, -0.2), makePoint(3769.2, 73746.3, -0.2),
    makePoint(3767.6, 73745.4, -0.2), makePoint(3766.0, 73744.6, -0.1),
    makePoint(3764.4, 73743.8, -0.1), makePoint(3762.7, 73742.9, -0.1),
    makePoint(3761.1, 73742.1, 0.0),  makePoint(3759.5, 73741.3, 0.0),
    makePoint(3757.9, 73740.4, 0.1),  makePoint(3756.3, 73739.6, 0.1)};

  const auto result_center_points = lanelet_map::centerPoints(34594);

  EXPECT_EQ(result_center_points.size(), actual_center_points.size());
  for (std::size_t i = 0; i < result_center_points.size(); ++i) {
    EXPECT_POINT_NEAR_STREAM(
      result_center_points[i], actual_center_points[i], 0.1, "In this test i = " << i);
  }
}

/**
 * @note Test basic functionality with a lanelet that has a centerline with 3 or more points
 * and call the function 2 times
 * - the goal is to test whether the centerline cache works correctly.
 */
TEST_F(LaneletWrapperTest_StandardMap, getCenterPoints_correctCache)
{
  const lanelet::Id id = 34594;

  const auto result_center_points = lanelet_map::centerPoints(id);
  const auto result_center_points2 = lanelet_map::centerPoints(id);

  EXPECT_EQ(result_center_points.size(), result_center_points2.size());
  for (std::size_t i = 0; i < result_center_points.size(); ++i) {
    EXPECT_POINT_EQ_STREAM(
      result_center_points[i], result_center_points2[i], "In this test i = " << i);
  }
}

/**
 * @note Test basic functionality with a vector containing valid lanelets.
 */
TEST_F(LaneletWrapperTest_StandardMap, getCenterPoints_correctVector)
{
  const std::vector<geometry_msgs::msg::Point> actual_center_points{
    makePoint(3774.1, 73748.8, -0.3), makePoint(3772.4, 73748.0, -0.2),
    makePoint(3770.8, 73747.1, -0.2), makePoint(3769.2, 73746.3, -0.2),
    makePoint(3767.6, 73745.4, -0.1), makePoint(3766.0, 73744.6, -0.1),
    makePoint(3764.4, 73743.8, -0.1), makePoint(3762.7, 73742.9, -0.0),
    makePoint(3761.1, 73742.1, -0.0), makePoint(3759.5, 73741.3, 0.0),
    makePoint(3757.9, 73740.4, 0.1),  makePoint(3756.3, 73739.6, 0.1),
    makePoint(3754.5, 73738.7, 0.1),  makePoint(3752.7, 73737.8, 0.2),
    makePoint(3750.9, 73736.9, 0.2),  makePoint(3749.1, 73736.0, 0.3),
    makePoint(3747.4, 73735.1, 0.3),  makePoint(3745.6, 73734.2, 0.3),
    makePoint(3743.8, 73733.2, 0.4),  makePoint(3742.0, 73732.3, 0.4),
    makePoint(3740.3, 73731.4, 0.4),  makePoint(3738.5, 73730.5, 0.5),
    makePoint(3736.7, 73729.6, 0.5),  makePoint(3734.9, 73728.7, 0.6),
    makePoint(3733.2, 73727.8, 0.6),  makePoint(3731.4, 73726.9, 0.6),
    makePoint(3729.6, 73725.9, 0.7),  makePoint(3727.8, 73725.0, 0.7),
    makePoint(3726.1, 73724.1, 0.7),  makePoint(3724.3, 73723.2, 0.8),
    makePoint(3722.5, 73722.3, 0.8),  makePoint(3720.7, 73721.4, 0.8),
    makePoint(3719.0, 73720.5, 0.9),  makePoint(3717.2, 73719.5, 0.9),
    makePoint(3715.4, 73718.6, 1.0),  makePoint(3713.7, 73717.7, 1.0),
    makePoint(3711.9, 73716.7, 1.1)};

  const auto result_center_points = lanelet_map::centerPoints(lanelet::Ids{34594, 34621});

  EXPECT_EQ(result_center_points.size(), actual_center_points.size());
  for (std::size_t i = 0; i < result_center_points.size(); ++i) {
    EXPECT_POINT_NEAR_STREAM(
      result_center_points[i], actual_center_points[i], 0.1, "In this test i = " << i);
  }
}

/**
 * @note Test basic functionality with an empty lanelet vector.
 */
TEST_F(LaneletWrapperTest_StandardMap, getCenterPoints_empty)
{
  EXPECT_EQ(lanelet_map::centerPoints(lanelet::Ids{}).size(), static_cast<std::size_t>(0));
}

/**
 * @note Test basic functionality. Test lanelet length obtaining with some lanelet id.
 */
TEST_F(LaneletWrapperTest_StandardMap, getLaneletLength_simple)
{
  EXPECT_NEAR(lanelet_map::laneletLength(34468), 55.5, 1.0);
}

/**
 * @note Test basic functionality.
 * Test lanelet length obtaining with some lanelet id two times
 * (the same lanelet id) - the goal is to test lanelet length caching correctness.
 */
TEST_F(LaneletWrapperTest_StandardMap, getLaneletLength_cache)
{
  const lanelet::Id id = 34468;

  EXPECT_EQ(lanelet_map::laneletLength(id), lanelet_map::laneletLength(id));
}

/**
 * @note Test for isInIntersection function
 */
TEST_F(LaneletWrapperTest_IntersectionMap, isInIntersection)
{
  EXPECT_TRUE(lanelet_map::isInIntersection(662));
  EXPECT_FALSE(lanelet_map::isInIntersection(574));
}

/**
 * @note Test basic functionality.
 * Test obtaining stop line ids correctness with a route that has no stop lines.
 */
TEST_F(LaneletWrapperTest_StandardMap, getStopLineIdsOnPath_noStopLines)
{
  EXPECT_EQ(
    lanelet_map::stopLineIdsOnPath({34507, 34795, 34606, 34672}).size(), static_cast<size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test obtaining stop line ids correctness with a route that has a stop line.
 */
TEST_F(LaneletWrapperTest_StandardMap, getStopLineIdsOnPath_someStopLines)
{
  EXPECT_EQ(
    lanelet_map::stopLineIdsOnPath({34408, 34633, 34579, 34780, 34675, 34744, 34690}),
    (lanelet::Ids{120635}));
}

/**
 * @note Test function behavior when passed an empty vector of lanelet ids.
 */
TEST_F(LaneletWrapperTest_StandardMap, getStopLineIdsOnPath_empty)
{
  EXPECT_EQ(lanelet_map::stopLineIdsOnPath(lanelet::Ids{}).size(), static_cast<size_t>(0));
}

/**
 * @note Test obtaining stop line ids for a standard map.
 */
TEST_F(LaneletWrapperTest_StandardMap, stopLineIds_standardMap)
{
  EXPECT_EQ(traffic_simulator::lanelet_wrapper::lanelet_map::stopLineIds(), (lanelet::Ids{120635}));
}

/**
 * @note Test obtaining stop line ids for an intersection map.
 */
TEST_F(LaneletWrapperTest_IntersectionMap, stopLineIds_intersectionMap)
{
  EXPECT_EQ(traffic_simulator::lanelet_wrapper::lanelet_map::stopLineIds(), (lanelet::Ids{6960}));
}

/**
 * @note Test function behavior when used with an empty map.
 */
TEST_F(LaneletWrapperTest_EmptyMap, stopLineIds_emptyMap)
{
  EXPECT_THROW(traffic_simulator::lanelet_wrapper::lanelet_map::stopLineIds(), std::runtime_error);
}

void sortStoplines(std::vector<std::vector<geometry_msgs::msg::Point>> & stoplines)
{
  auto point_comparator =
    [](const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b) -> bool {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
  };

  std::for_each(
    stoplines.begin(), stoplines.end(),
    [&point_comparator](std::vector<geometry_msgs::msg::Point> & v) {
      std::sort(v.begin(), v.end(), point_comparator);
    });
  std::sort(
    stoplines.begin(), stoplines.end(),
    [&point_comparator](
      const std::vector<geometry_msgs::msg::Point> & a,
      const std::vector<geometry_msgs::msg::Point> & b) {
      return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end(), point_comparator);
    });
}

void compareStoplines(
  const std::vector<std::vector<geometry_msgs::msg::Point>> & a,
  const std::vector<std::vector<geometry_msgs::msg::Point>> & b)
{
  EXPECT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].size(), b[i].size()) << "In this test i = " << i;
    for (std::size_t j = 0; j < a[i].size(); ++j) {
      EXPECT_POINT_NEAR_STREAM(a[i][j], b[i][j], 1.0, "In this test i = " << i << ", j = " << j);
    }
  }
}

/**
 * @note Test basic functionality.
 * Test stop line polygon obtaining correctness with a lanelet that has a stop line.
 */
TEST_F(LaneletWrapperTest_CrossroadsWithStoplinesMap, getStopLinePolygon_stopLine)
{
  const auto result_stoplines_points = lanelet_map::stopLinePolygon(lanelet::Id{120663});
  const auto actual_stoplines_points = std::vector<geometry_msgs::msg::Point>{
    makePoint(3768.5, 73737.5, -0.5), makePoint(3765.5, 73735.5, -0.5)};

  const double tolerance = 1.0;
  EXPECT_EQ(result_stoplines_points.size(), actual_stoplines_points.size());
  EXPECT_POINT_NEAR(result_stoplines_points.at(0), actual_stoplines_points.at(0), tolerance);
  EXPECT_POINT_NEAR(result_stoplines_points.at(1), actual_stoplines_points.at(1), tolerance);
}

/**
 * @note Test function behavior with an invalid lanelet id.
 */
TEST_F(LaneletWrapperTest_CrossroadsWithStoplinesMap, getStopLinePolygon_invalidLaneletId)
{
  EXPECT_THROW(lanelet_map::stopLinePolygon(1000039), std::runtime_error);
}
}  // namespace traffic_simulator::lanelet_wrapper::tests
