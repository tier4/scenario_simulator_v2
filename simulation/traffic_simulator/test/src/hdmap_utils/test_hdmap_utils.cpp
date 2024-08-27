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

#include <geometry_msgs/msg/point.h>
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/helper/helper.hpp>

#include "../expect_eq_macros.hpp"
#include "../helper_functions.hpp"

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class HdMapUtilsTest_StandardMap : public testing::Test
{
protected:
  HdMapUtilsTest_StandardMap()
  : hdmap_utils(hdmap_utils::HdMapUtils(
      ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm",
      geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
        .latitude(35.61836750154)
        .longitude(139.78066608243)
        .altitude(0.0)))
  {
  }

  hdmap_utils::HdMapUtils hdmap_utils;
};
class HdMapUtilsTest_WithRoadShoulderMap : public testing::Test
{
protected:
  HdMapUtilsTest_WithRoadShoulderMap()
  : hdmap_utils(hdmap_utils::HdMapUtils(
      ament_index_cpp::get_package_share_directory("traffic_simulator") +
        "/map/with_road_shoulder/lanelet2_map.osm",
      geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
        .latitude(35.61836750154)
        .longitude(139.78066608243)
        .altitude(0.0)))
  {
  }

  hdmap_utils::HdMapUtils hdmap_utils;
};
class HdMapUtilsTest_EmptyMap : public testing::Test
{
protected:
  HdMapUtilsTest_EmptyMap()
  : hdmap_utils(hdmap_utils::HdMapUtils(
      ament_index_cpp::get_package_share_directory("traffic_simulator") +
        "/map/empty/lanelet2_map.osm",
      geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
        .latitude(0.0)
        .longitude(0.0)
        .altitude(0.0)))
  {
  }

  hdmap_utils::HdMapUtils hdmap_utils;
};
class HdMapUtilsTest_FourTrackHighwayMap : public testing::Test
{
protected:
  HdMapUtilsTest_FourTrackHighwayMap()
  : hdmap_utils(hdmap_utils::HdMapUtils(
      ament_index_cpp::get_package_share_directory("traffic_simulator") +
        "/map/four_track_highway/lanelet2_map.osm",
      geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
        .latitude(35.22312494055522)
        .longitude(138.8024583466017)
        .altitude(0.0)))
  {
  }

  hdmap_utils::HdMapUtils hdmap_utils;
};
class HdMapUtilsTest_CrossroadsWithStoplinesMap : public testing::Test
{
protected:
  HdMapUtilsTest_CrossroadsWithStoplinesMap()
  : hdmap_utils(hdmap_utils::HdMapUtils(
      ament_index_cpp::get_package_share_directory("traffic_simulator") +
        "/map/crossroads_with_stoplines/lanelet2_map.osm",
      geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
        .latitude(35.23808753540768)
        .longitude(139.9009591876285)
        .altitude(0.0)))
  {
  }

  hdmap_utils::HdMapUtils hdmap_utils;
};
class HdMapUtilsTest_KashiwanohaMap : public testing::Test
{
protected:
  HdMapUtilsTest_KashiwanohaMap()
  : hdmap_utils(hdmap_utils::HdMapUtils(
      ament_index_cpp::get_package_share_directory("kashiwanoha_map") + "/map/lanelet2_map.osm",
      geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
        .latitude(0.0)
        .longitude(0.0)
        .altitude(0.0)))
  {
  }

  hdmap_utils::HdMapUtils hdmap_utils;
};
/**
 * @note Test basic functionality.
 * Test initialization correctness with a correct path to a lanelet map.
 */
TEST(HdMapUtils, Construct)
{
  ASSERT_NO_THROW(
    auto hdmap_utils = hdmap_utils::HdMapUtils(
      ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm",
      geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
        .latitude(35.61836750154)
        .longitude(139.78066608243)
        .altitude(0.0)));
}

/**
 * @note Test basic functionality.
 * Test initialization correctness with an invalid path to a lanelet map.
 */
TEST(HdMapUtils, Construct_invalid)
{
  EXPECT_THROW(
    auto hdmap_utils = hdmap_utils::HdMapUtils(
      ament_index_cpp::get_package_share_directory("traffic_simulator") + "invalid_path",
      geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
        .latitude(35.61836750154)
        .longitude(139.78066608243)
        .altitude(0.0)),
    std::runtime_error);
}

/**
 * @note Test basic functionality.
 * Test map conversion to binary message correctness with a sample map.
 */
TEST_F(HdMapUtilsTest_StandardMap, toMapBin) { ASSERT_NO_THROW(hdmap_utils.toMapBin()); }

/**
 * @note Test basic functionality.
 * Test lanelet matching correctness with a small bounding box (1, 1)
 * and a pose on a lanelet and without including the crosswalk.
 */
TEST_F(HdMapUtilsTest_StandardMap, matchToLane)
{
  const auto bbox = makeSmallBoundingBox();
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(120659, 1)).pose, bbox,
      false);
    EXPECT_TRUE(id);
    EXPECT_EQ(id.value(), 120659);
  }
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34411, 1)).pose, bbox,
      false);
    EXPECT_TRUE(id);
    EXPECT_EQ(id.value(), 34411);
  }
}

/**
 * @note Test basic functionality.
 * Test lanelet matching correctness with a small bounding box (1, 1)
 * and a pose on a crosswalk lanelet and including the crosswalk.
 */
TEST_F(HdMapUtilsTest_StandardMap, matchToLane_includeCrosswalk)
{
  auto bbox = makeSmallBoundingBox();
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34399, 1)).pose, bbox,
      true);
    EXPECT_TRUE(id.has_value());
    EXPECT_EQ(id.value(), 34399);
  }
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34399, 1)).pose, bbox,
      false);
    if (id.has_value()) {
      EXPECT_NE(id.value(), 34399);
    }
  }
}

/**
 * @note Test basic functionality.
 * Test lanelet matching correctness with a small bounding box (1, 1)
 * and such a pose so that no lanelets are in the distance of 1 unit
 * - the goal is to test the branch where getDeterministicMatches returns nullopt and thus
 * this function returns nullopt as well.
 */
TEST_F(HdMapUtilsTest_StandardMap, matchToLane_noMatch)
{
  auto bbox = makeSmallBoundingBox();
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34392, 0)).pose, bbox,
      false);
    EXPECT_FALSE(id.has_value());
  }
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34378, 0)).pose, bbox,
      false);
    EXPECT_FALSE(id.has_value());
  }
}

/**
 * @note Test basic functionality.
 * Test along lanelet pose obtaining with a distance
 * along the lanelet less than the lanelet length - so the along pose is still the same lanelet.
*/
TEST_F(HdMapUtilsTest_StandardMap, AlongLaneletPose_insideDistance)
{
  EXPECT_DOUBLE_EQ(
    hdmap_utils.getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0), 30.0)
      .s,
    30.0);
  EXPECT_EQ(
    hdmap_utils.getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0), 30.0)
      .lanelet_id,
    34513);
}

/**
 * @note Test basic functionality.
 * Test along lanelet pose obtaining with a distance
 * along the lanelet more than the lanelet length - the goal is
 * to test the situation when the next lanelet is returned.
*/
TEST_F(HdMapUtilsTest_StandardMap, AlongLaneletPose_outsideDistance)
{
  EXPECT_EQ(
    hdmap_utils.getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0), 30)
      .lanelet_id,
    34513);
  EXPECT_EQ(
    hdmap_utils
      .getAlongLaneletPose(
        traffic_simulator::helper::constructLaneletPose(34513, 0),
        hdmap_utils.getLaneletLength(34513) + 10.0)
      .lanelet_id,
    34510);
}

/**
 * @note Test basic functionality.
 * Test along lanelet pose obtaining with a negative distance
 * along the lanelet and start from the beginning of one lanelet - the goal is to test
 * the situation when the previous lanelet is returned.
*/
TEST_F(HdMapUtilsTest_StandardMap, AlongLaneletPose_negativeDistance)
{
  EXPECT_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0), -10.0)
      .lanelet_id,
    34684);
  EXPECT_DOUBLE_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0), -10.0)
      .s,
    hdmap_utils.getLaneletLength(34684) - 10.0);
}

/**
 * @note Test function behavior when passed a sufficiently large along distance
 * and the last lanelet on the map as start - the goal is to test the situation
 * when desired pose is outside the lanelet map.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, AlongLaneletPose_afterLast)
{
  EXPECT_THROW(
    hdmap_utils.getAlongLaneletPose(
      traffic_simulator::helper::constructLaneletPose(206, 15.0), 30.0),
    common::SemanticError);
}

/**
 * @note Test function behavior when passed a negative along distance and first
 * lanelet on the map as start - the goal is to test the situation
 * when desired pose is outside the lanelet map.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, AlongLaneletPose_beforeFirst)
{
  EXPECT_THROW(
    hdmap_utils.getAlongLaneletPose(
      traffic_simulator::helper::constructLaneletPose(3002178, 15.0), -30.0),
    common::SemanticError);
}

/**
 * @note Test lanelet pose obtaining correctness when s < 0.
 * Function should find correct lanelet id and canonicalize lanelet pose even when s < 0.
 * Following lanelets: 34576 -> 34570 -> 34564
 * Canonicalized lanelet pose of (id=34564, s=-22) is suppose to be
 *                               (id=34576, s=-22 + length of 34570 + length of 34576)
 */
TEST_F(HdMapUtilsTest_StandardMap, CanonicalizeNegative)
{
  double non_canonicalized_lanelet_s = -22.0;
  const auto canonicalized_lanelet_pose =
    std::get<std::optional<traffic_simulator::LaneletPose>>(hdmap_utils.canonicalizeLaneletPose(
      traffic_simulator::helper::constructLaneletPose(34564, non_canonicalized_lanelet_s)));

  EXPECT_EQ(canonicalized_lanelet_pose.value().lanelet_id, 34576);
  EXPECT_EQ(
    canonicalized_lanelet_pose.value().s, non_canonicalized_lanelet_s +
                                            hdmap_utils.getLaneletLength(34570) +
                                            hdmap_utils.getLaneletLength(34576));
}

/**
 * @note Test lanelet pose obtaining correctness when s is larger than lanelet length.
 * Function should find correct lanelet id and canonicalize lanelet pose for s larger than lanelet length.
 * Following lanelets: 34981 -> 34585 -> 34579
 * Canonicalized lanelet pose of (id=34981, s=30) is suppose to be
 *                               (id=34579, s=30 - length of 34585 - length of 34981)
 */
TEST_F(HdMapUtilsTest_StandardMap, CanonicalizePositive)
{
  double non_canonicalized_lanelet_s = 30.0;
  const auto canonicalized_lanelet_pose =
    std::get<std::optional<traffic_simulator::LaneletPose>>(hdmap_utils.canonicalizeLaneletPose(
      traffic_simulator::helper::constructLaneletPose(34981, non_canonicalized_lanelet_s)));

  EXPECT_EQ(canonicalized_lanelet_pose.value().lanelet_id, 34579);
  EXPECT_EQ(
    canonicalized_lanelet_pose.value().s, non_canonicalized_lanelet_s -
                                            hdmap_utils.getLaneletLength(34585) -
                                            hdmap_utils.getLaneletLength(34981));
}

/**
 * @note Testcase for lanelet pose canonicalization when s in
 * range [0,length_of_the_lanelet]
 * Canonicalized lanelet pose of (id=34981, s=2) is suppose to be the same.
 */
TEST_F(HdMapUtilsTest_StandardMap, Canonicalize)
{
  const double non_canonicalized_lanelet_s = 2.0;
  const auto canonicalized_lanelet_pose =
    std::get<std::optional<traffic_simulator::LaneletPose>>(hdmap_utils.canonicalizeLaneletPose(
      traffic_simulator::helper::constructLaneletPose(34981, non_canonicalized_lanelet_s)));

  EXPECT_EQ(canonicalized_lanelet_pose.value().lanelet_id, 34981);
  EXPECT_EQ(canonicalized_lanelet_pose.value().s, non_canonicalized_lanelet_s);
}

/**
 * @note Test lanelet pose vector obtaining correctness when s < 0.
 * Function should find correct lanelet ids and canonicalize lanelet pose even when s < 0.
 * Following lanelets: 34576 -> 34570 -> 34564
 *                     34981 -> 34636 -> 34564
 *                     34600 -> 34648 -> 34564
 * Canonicalized lanelet pose of (id=34564, s=-22) is suppose to be
 *                               (id=34575, s=-22 + length of 34570 + length of 34576)
 *                               (id=34981, s=-22 + length of 34636 + length of 34981)
 *                               (id=34600, s=-22 + length of 34648 + length of 34600)
 */
TEST_F(HdMapUtilsTest_StandardMap, CanonicalizeAllNegative)
{
  const double non_canonicalized_lanelet_s = -22.0;
  const auto canonicalized_lanelet_poses = hdmap_utils.getAllCanonicalizedLaneletPoses(
    traffic_simulator::helper::constructLaneletPose(34564, non_canonicalized_lanelet_s));

  EXPECT_EQ(canonicalized_lanelet_poses.size(), static_cast<std::size_t>(3));
  EXPECT_EQ(canonicalized_lanelet_poses[0].lanelet_id, 34576);
  EXPECT_EQ(
    canonicalized_lanelet_poses[0].s, non_canonicalized_lanelet_s +
                                        hdmap_utils.getLaneletLength(34570) +
                                        hdmap_utils.getLaneletLength(34576));
  EXPECT_EQ(canonicalized_lanelet_poses[1].lanelet_id, 34981);
  EXPECT_EQ(
    canonicalized_lanelet_poses[1].s, non_canonicalized_lanelet_s +
                                        hdmap_utils.getLaneletLength(34636) +
                                        hdmap_utils.getLaneletLength(34981));
  EXPECT_EQ(canonicalized_lanelet_poses[2].lanelet_id, 34600);
  EXPECT_EQ(
    canonicalized_lanelet_poses[2].s, non_canonicalized_lanelet_s +
                                        hdmap_utils.getLaneletLength(34648) +
                                        hdmap_utils.getLaneletLength(34600));
}

/**
 * @note Test lanelet pose vector obtaining correctness when s is larger than lanelet length.
 * Function should find correct lanelet ids and canonicalize lanelet pose for s larger than lanelet length.
 * Following lanelets: 34981 -> 34585 -> 34579
 *                     34981 -> 34636 -> 34564
 *                     34981 -> 34651 -> 34630
 * Canonicalized lanelet pose of (id=34981, s=30) is suppose to be
 *                               (id=34579, s=30 - length of 34585 - length of 34981)
 *                               (id=34564, s=30 - length of 34636 - length of 34981)
 *                               (id=34630, s=30 - length of 34651 - length of 34981)
 */
TEST_F(HdMapUtilsTest_StandardMap, CanonicalizeAllPositive)
{
  const double non_canonicalized_lanelet_s = 30.0;
  const auto canonicalized_lanelet_poses = hdmap_utils.getAllCanonicalizedLaneletPoses(
    traffic_simulator::helper::constructLaneletPose(34981, non_canonicalized_lanelet_s));

  EXPECT_EQ(canonicalized_lanelet_poses.size(), static_cast<std::size_t>(3));
  EXPECT_EQ(canonicalized_lanelet_poses[0].lanelet_id, 34579);
  EXPECT_EQ(
    canonicalized_lanelet_poses[0].s, non_canonicalized_lanelet_s -
                                        hdmap_utils.getLaneletLength(34585) -
                                        hdmap_utils.getLaneletLength(34981));
  EXPECT_EQ(canonicalized_lanelet_poses[1].lanelet_id, 34564);
  EXPECT_EQ(
    canonicalized_lanelet_poses[1].s, non_canonicalized_lanelet_s -
                                        hdmap_utils.getLaneletLength(34636) -
                                        hdmap_utils.getLaneletLength(34981));
  EXPECT_EQ(canonicalized_lanelet_poses[2].lanelet_id, 34630);
  EXPECT_EQ(
    canonicalized_lanelet_poses[2].s, non_canonicalized_lanelet_s -
                                        hdmap_utils.getLaneletLength(34651) -
                                        hdmap_utils.getLaneletLength(34981));
}

/**
 * @note Testcase for getAllCanonicalizedLaneletPoses() function when s in
 * range [0,length_of_the_lanelet]
 * Canonicalized lanelet pose of (id=34981, s=2) is supposed to be the same.
 */
TEST_F(HdMapUtilsTest_StandardMap, CanonicalizeAll)
{
  const double non_canonicalized_lanelet_s = 2.0;
  const auto canonicalized_lanelet_poses = hdmap_utils.getAllCanonicalizedLaneletPoses(
    traffic_simulator::helper::constructLaneletPose(34981, non_canonicalized_lanelet_s));

  EXPECT_EQ(canonicalized_lanelet_poses.size(), static_cast<std::size_t>(1));
  EXPECT_EQ(canonicalized_lanelet_poses[0].lanelet_id, 34981);
  EXPECT_EQ(canonicalized_lanelet_poses[0].s, non_canonicalized_lanelet_s);
}

/**
 * @note Testcase for countLaneChanges() function
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, CountLaneChangesAlongRoute)
{
  using traffic_simulator::helper::constructLaneletPose;
  EXPECT_EQ(
    hdmap_utils.countLaneChanges(
      constructLaneletPose(3002176, 0), constructLaneletPose(3002175, 0), true),
    std::make_pair(1, 0));
  EXPECT_EQ(
    hdmap_utils.countLaneChanges(
      constructLaneletPose(3002176, 0), constructLaneletPose(3002182, 0), true),
    std::make_pair(1, 0));
  EXPECT_EQ(
    hdmap_utils.countLaneChanges(
      constructLaneletPose(3002176, 0), constructLaneletPose(199, 0), true),
    std::make_pair(1, 0));
  EXPECT_EQ(
    hdmap_utils.countLaneChanges(
      constructLaneletPose(3002176, 0), constructLaneletPose(3002176, 0), true),
    std::make_pair(0, 0));
  EXPECT_EQ(
    hdmap_utils.countLaneChanges(
      constructLaneletPose(3002176, 0), constructLaneletPose(200, 0), true),
    std::make_pair(0, 0));
  EXPECT_EQ(
    hdmap_utils.countLaneChanges(
      constructLaneletPose(3002176, 0), constructLaneletPose(201, 0), true),
    std::make_pair(0, 1));
  EXPECT_EQ(
    hdmap_utils.countLaneChanges(
      constructLaneletPose(3002176, 0), constructLaneletPose(202, 0), true),
    std::make_pair(0, 2));
  EXPECT_EQ(
    hdmap_utils.countLaneChanges(
      constructLaneletPose(3002176, 0), constructLaneletPose(206, 0), true),
    std::make_pair(0, 2));
}

/**
 * @note Test basic functionality.
 * Test filtering correctness with some lanelet ids and a valid subtype name.
 */
TEST_F(HdMapUtilsTest_StandardMap, filterLaneletIds_correct)
{
  const lanelet::Id id_crosswalk_0 = 34399;
  const lanelet::Id id_crosswalk_1 = 34385;

  auto filtered =
    hdmap_utils.filterLaneletIds({id_crosswalk_0, id_crosswalk_1, 34600, 34675}, "crosswalk");

  EXPECT_EQ(filtered.size(), static_cast<std::size_t>(2));
  EXPECT_TRUE(std::find(filtered.begin(), filtered.end(), id_crosswalk_0) != filtered.end());
  EXPECT_TRUE(std::find(filtered.begin(), filtered.end(), id_crosswalk_1) != filtered.end());
}

/**
 * @note Test function behavior when passed an empty lanelet ids vector.
 */
TEST_F(HdMapUtilsTest_StandardMap, filterLaneletIds_emptyIds)
{
  EXPECT_TRUE(hdmap_utils.filterLaneletIds({}, "crosswalk").empty());
}

/**
 * @note Test function behavior when passed an invalid subtype name.
 */
TEST_F(HdMapUtilsTest_StandardMap, filterLaneletIds_invalidSubtype)
{
  EXPECT_TRUE(
    hdmap_utils.filterLaneletIds({34399, 34385, 34600, 34675}, "invalid_subtype").empty());
}

/**
 * @note Test function behavior when passed a vector of invalid lanelet ids.
 */
TEST_F(HdMapUtilsTest_StandardMap, filterLaneletIds_invalidIds)
{
  EXPECT_THROW(
    auto filtered = hdmap_utils.filterLaneletIds({10000000, 10000001, 10000002}, "crosswalk"),
    std::runtime_error);
}

/**
 * @note Test basic functionality.
 * Test obtaining nearest lanelet ids correctness
 * with a position in the middle of the lane and relatively big distance threshold
 * - the goal is to test successful scenario when there should be lanelets returned.
 */
TEST_F(HdMapUtilsTest_StandardMap, getNearbyLaneletIds)
{
  EXPECT_EQ(
    hdmap_utils.getNearbyLaneletIds(
      makePoint(3807.34, 73817.95), 10.0, static_cast<std::size_t>(100)),
    (lanelet::Ids{34795, 120660, 34507, 34468, 120659, 34606}));
}

/**
 * @note Test basic functionality.
 * Test obtaining nearest lanelet ids correctness
 * with a position on the side of the map and with fairly small distance threshold
 * - the goal is to test unsuccessful scenario when there should be no lanelets returned.
 */
TEST_F(HdMapUtilsTest_StandardMap, getNearbyLaneletIds_unsuccessful)
{
  EXPECT_TRUE(
    hdmap_utils
      .getNearbyLaneletIds(makePoint(3826.26, 73837.32), 10.0, static_cast<std::size_t>(100))
      .empty());
}

/**
 * @note Test basic functionality.
 * Test obtaining nearest lanelet ids correctness
 * (with a crosswalk) with a position on the side of the map and with fairly small distance threshold
 * - the goal is to test unsuccessful scenario when there should be no lanelets returned.
 */
TEST_F(HdMapUtilsTest_StandardMap, getNearbyLaneletIds_crosswalkUnsuccessful)
{
  EXPECT_TRUE(
    hdmap_utils
      .getNearbyLaneletIds(makePoint(3826.26, 73837.32), 10.0, true, static_cast<std::size_t>(100))
      .empty());
}

/**
 * @note Test basic functionality.
 * Test collision point calculations
 * correctness with ids of a road and a crosswalk that do intersect.
 */
TEST_F(HdMapUtilsTest_StandardMap, getCollisionPointInLaneCoordinate_intersects)
{
  auto distance = hdmap_utils.getCollisionPointInLaneCoordinate(34633, 34399);

  EXPECT_TRUE(distance.has_value());
  EXPECT_GT(distance.value(), 0.0);
}

/**
 * @note Test basic functionality.
 * Test collision point calculations
 * correctness with ids of a road and a crosswalk that do not intersect.
 */
TEST_F(HdMapUtilsTest_StandardMap, getCollisionPointInLaneCoordinate_disjoint)
{
  EXPECT_FALSE(hdmap_utils.getCollisionPointInLaneCoordinate(34579, 34399).has_value());
}

/**
 * @note Test function behavior when called with an id of non existing lanelet.
 */
TEST_F(HdMapUtilsTest_StandardMap, getCollisionPointInLaneCoordinate_invalidLanelet)
{
  EXPECT_THROW(hdmap_utils.getCollisionPointInLaneCoordinate(1000000, 34399), std::runtime_error);
}

/**
 * @note Test function behavior when called with an id of non existing crosswalk lanelet.
 */
TEST_F(HdMapUtilsTest_StandardMap, getCollisionPointInLaneCoordinate_invalidCrosswalkLanelet)
{
  EXPECT_THROW(hdmap_utils.getCollisionPointInLaneCoordinate(34600, 1000000), std::runtime_error);
}

/**
 * @note Test basic functionality.
 * Test conversion to lanelet pose correctness with a point
 * positioned on a given lanelet with the given matching distance
 * - the goal is to test a regular usecase of correct conversion.
 */
TEST_F(HdMapUtilsTest_StandardMap, toLaneletPose_correct)
{
  const auto lanelet_pose = hdmap_utils.toLaneletPose(
    makePose(makePoint(3790.0, 73757.0), makeQuaternionFromYaw(M_PI + M_PI_2 / 3.0)),
    false);  // angle to make pose aligned with the lanelet

  const auto reference_lanelet_pose =
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::LaneletPose>()
      .lanelet_id(34600)
      .s(35.0)
      .offset(0.0)
      .rpy(geometry_msgs::msg::Vector3());

  EXPECT_TRUE(lanelet_pose.has_value());
  EXPECT_LANELET_POSE_NEAR(lanelet_pose.value(), reference_lanelet_pose, 0.1);
}

/**
 * @note Test basic functionality.
 * Test conversion to lanelet pose correctness with a point
 * positioned near a given lanelet (closer than the given matching distance) and slightly behind
 * the normal vector of the nearest point on the lanelet - the goal is to test
 * the branch of execution where the inner product between:
 * - the vector from the given pose to the nearest point on the spline and
 * - the normal vector to the nearest point on the spline
 * is negative - meaning the offset should be calculated negative
 */
TEST_F(HdMapUtilsTest_StandardMap, toLaneletPose_negativeOffset)
{
  const double yaw = M_PI + M_PI_2 / 3.0;  // angle to make pose aligned with the lanelet

  const double offset_yaw = yaw - M_PI_2;  // offset pose
  const double offset = -0.5;

  const auto pose = makePose(
    makePoint(
      3790.0 + std::cos(offset_yaw) * std::abs(offset),
      73757.0 + std::sin(offset_yaw) * std::abs(offset)),
    makeQuaternionFromYaw(yaw));

  const auto lanelet_pose = hdmap_utils.toLaneletPose(pose, false);

  const auto reference_lanelet_pose =
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::LaneletPose>()
      .lanelet_id(34600)
      .s(35.0)
      .offset(offset)
      .rpy(geometry_msgs::msg::Vector3());

  EXPECT_TRUE(lanelet_pose.has_value());
  EXPECT_LANELET_POSE_NEAR(lanelet_pose.value(), reference_lanelet_pose, 0.1);
}

/**
 * @note Test function behavior when passed a pose that is positioned on the given lanelet,
 * but is facing the wrong direction (either parallel to the lanelet or aligned but in reverse)
 * - the goal is to test the branch of execution where alignment with the lanelet
 * is checked and if pose is nor oriented in similar direction the result is decided incorrect.
 */
TEST_F(HdMapUtilsTest_StandardMap, toLaneletPose_reverse)
{
  EXPECT_FALSE(
    hdmap_utils
      .toLaneletPose(
        makePose(makePoint(3790.0, 73757.0), makeQuaternionFromYaw(M_PI_2 + M_PI_2 / 3.0)), false)
      .has_value());  // angle to make pose reverse aligned with the lanelet
}

/**
 * @note Test function behavior when passed a pose that is away
 * from the given lanelet (over the matching distance).
 */
TEST_F(HdMapUtilsTest_StandardMap, toLaneletPose_notOnLanelet)
{
  EXPECT_FALSE(
    hdmap_utils
      .toLaneletPose(
        makePose(
          makePoint(3790.0 + 5.0, 73757.0 - 5.0), makeQuaternionFromYaw(M_PI + M_PI_2 / 3.0)),
        true)
      .has_value());  // angle to make pose aligned with the lanelet
}

/**
 * @note test function behavior when passed an empty vector
 * of lanelet ids (for the vector specialization).
 */
TEST_F(HdMapUtilsTest_StandardMap, toLaneletPose_empty)
{
  EXPECT_FALSE(hdmap_utils
                 .toLaneletPose(
                   makePose(makePoint(3790.0, 73757.0), makeQuaternionFromYaw(M_PI + M_PI_2 / 3.0)),
                   lanelet::Ids{})
                 .has_value());  // angle to make pose aligned with the lanelet
}

/**
 * @note Test basic functionality.
 * Test lanelet matching correctness with a bounding box and a pose that is:
 * - exactly on the centerline of a lanelet
 * - <0.9; 1) away from the next lanelet and a fairly small matching_distance (e.g. 0.5)
 * - the goal is to test the branch in bounding box variant where the bounding box is matched
 * to the next lanelet (with hardcoded max distance of 1), but the distance is more than
 * the matching_distance. In this situation the previous lanelet has to be matched
 * - so the loop over previous lanelets is executed.
 */
TEST_F(HdMapUtilsTest_StandardMap, toLaneletPose_boundingBoxMatchPrevious)
{
  EXPECT_LANELET_POSE_NEAR(
    hdmap_utils
      .toLaneletPose(
        makePose(
          makePoint(3774.9, 73749.2),
          makeQuaternionFromYaw(
            M_PI + M_PI_2 / 3.0)),  // angle to make pose aligned with the lanelet
        makeBoundingBox(), false, 0.5)
      .value(),
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::LaneletPose>()
      .lanelet_id(34600)
      .s(52.0)
      .offset(0.0)
      .rpy(geometry_msgs::msg::Vector3()),
    0.1);
}

/**
 * @note Test basic functionality.
 * Test speed limit obtaining correctness
 * with ids of lanelets that have different speed limits.
 */
TEST_F(HdMapUtilsTest_StandardMap, getSpeedLimit_correct)
{
  EXPECT_NEAR(hdmap_utils.getSpeedLimit(lanelet::Ids{34600, 34675}), 50.0 / 3.6, 0.01);
}

/**
 * @note Test function behavior when crosswalk lanelet id is included in the vector.
 */
TEST_F(HdMapUtilsTest_StandardMap, getSpeedLimit_crosswalk)
{
  EXPECT_NEAR(hdmap_utils.getSpeedLimit(lanelet::Ids{34399, 34385, 34600, 34675}), 0.0 / 3.6, 0.01);
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(HdMapUtilsTest_StandardMap, getSpeedLimit_empty)
{
  EXPECT_THROW(hdmap_utils.getSpeedLimit(lanelet::Ids{}), std::runtime_error);
}

/**
 * @note Test basic functionality.
 * Test obtaining closest lanelet id with a pose near
 * the road lanelet (closer than the distance_threshold).
 */
TEST_F(HdMapUtilsTest_StandardMap, getClosestLaneletId_near)
{
  const auto result =
    hdmap_utils.getClosestLaneletId(makePose(makePoint(3818.91, 73787.95)), 1.0, false);

  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), 120659);
}

/**
 * @note Test basic functionality.
 * Test obtaining closest lanelet id with a pose far
 * from the road lanelet (further than the distance_threshold).
 */
TEST_F(HdMapUtilsTest_StandardMap, getClosestLaneletId_away)
{
  EXPECT_FALSE(hdmap_utils.getClosestLaneletId(makePose(makePoint(3775.82, 73743.29)), 1.0, false)
                 .has_value());
}

/**
 * @note Test basic functionality.
 * Test obtaining closest lanelet id with a pose near
 * the crosswalk lanelet (closer than the distance_threshold) and include_crosswalk = false
 * and road lanelet further than crosswalk, but closer than distance_threshold
 * - the goal is to test whether the function returns road lanelet,
 * when the crosswalk is closer, but should not be included.
 */
TEST_F(HdMapUtilsTest_StandardMap, getClosestLaneletId_crosswalkCloserButExcluded)
{
  const auto result =
    hdmap_utils.getClosestLaneletId(makePose(makePoint(3774.73, 73744.38)), 5.0, false);

  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), 34639);
  EXPECT_NE(result.value(), 34399);
}

/**
 * @note Test basic functionality.
 * Test obtaining closest lanelet id with a pose near
 * the crosswalk lanelet (closer than the distance_threshold) and include_crosswalk = false
 * and road lanelet further than crosswalk and further away than distance_threshold
 * - the goal is to test scenario when the only lanelet in the
 * considered distance is crosswalk, but should not be included.
 */
TEST_F(HdMapUtilsTest_StandardMap, getClosestLaneletId_onlyCrosswalkNearButExcluded)
{
  const auto pose = makePose(makePoint(3774.73, 73744.38));
  const double distance_threshold = 2.0;

  {
    const bool include_crosswalk = true;
    const auto result =
      hdmap_utils.getClosestLaneletId(pose, distance_threshold, include_crosswalk);

    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 34399);
  }
  {
    const bool include_crosswalk = false;
    const auto result =
      hdmap_utils.getClosestLaneletId(pose, distance_threshold, include_crosswalk);

    EXPECT_FALSE(result.has_value());
  }
}

/**
 * @note Test function behavior when the map contains no lanelets - the goal is to test
 * the branch when findNearest does not find anything and returns an empty vector.
 */
TEST_F(HdMapUtilsTest_EmptyMap, getClosestLaneletId_emptyMap)
{
  EXPECT_FALSE(
    hdmap_utils.getClosestLaneletId(makePose(makePoint(3.0, 5.0)), 7.0, false).has_value());
}

/**
 * @note Test basic functionality.
 * Test previous lanelets id obtaining correctness
 * with a lanelet that has a lanelet preceding it.
 */
TEST_F(HdMapUtilsTest_StandardMap, getPreviousLaneletIds)
{
  const auto result_ids = hdmap_utils.getPreviousLaneletIds(34468);
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
TEST_F(HdMapUtilsTest_WithRoadShoulderMap, getPreviousLaneletIds_RoadShoulder)
{
  const auto result_ids = hdmap_utils.getPreviousLaneletIds(34768);
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
TEST_F(HdMapUtilsTest_StandardMap, getPreviousLaneletIds_multiplePrevious)
{
  lanelet::Ids prev_lanelets = {34411, 34465};
  auto result_ids = hdmap_utils.getPreviousLaneletIds(34462);

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
TEST_F(HdMapUtilsTest_StandardMap, getPreviousLaneletIds_direction)
{
  const lanelet::Id curr_lanelet = 34462;
  const lanelet::Id prev_lanelet_left = 34411;
  const lanelet::Id prev_lanelet_straight = 34465;

  {
    const auto result_ids = hdmap_utils.getPreviousLaneletIds(curr_lanelet, "left");
    EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
    if (result_ids.size() == 1) {
      EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(prev_lanelet_left));
    }
  }
  {
    const auto result_ids = hdmap_utils.getPreviousLaneletIds(curr_lanelet, "straight");
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
TEST_F(HdMapUtilsTest_StandardMap, getNextLaneletIds)
{
  const auto result_ids = hdmap_utils.getNextLaneletIds(120660);
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
TEST_F(HdMapUtilsTest_WithRoadShoulderMap, getNextLaneletIds_RoadShoulder)
{
  const auto result_ids = hdmap_utils.getNextLaneletIds(34696);
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
TEST_F(HdMapUtilsTest_StandardMap, getNextLaneletIds_multipleNext)
{
  lanelet::Ids next_lanelets = {34438, 34465};
  auto result_ids = hdmap_utils.getNextLaneletIds(34468);

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
TEST_F(HdMapUtilsTest_StandardMap, getNextLaneletIds_direction)
{
  const lanelet::Id curr_lanelet = 34468;

  {
    const auto result_ids = hdmap_utils.getNextLaneletIds(curr_lanelet, "left");
    EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
    if (result_ids.size() == 1) {
      EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(34438));
    }
  }
  {
    const auto result_ids = hdmap_utils.getNextLaneletIds(curr_lanelet, "straight");
    EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
    if (result_ids.size() == 1) {
      EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(34465));
    }
  }
}

/**
 * @note Test basic functionality.
 * Test on route checking correctness
 * with a route and a lanelet that is on the route.
 */
TEST_F(HdMapUtilsTest_StandardMap, isInRoute_onRoute)
{
  EXPECT_TRUE(hdmap_utils.isInRoute(34850, lanelet::Ids{34741, 34850, 34603, 34777}));
}

/**
 * @note Test basic functionality.
 * Test on route checking correctness
 * with a route and a lanelet that is not on the route.
 */
TEST_F(HdMapUtilsTest_StandardMap, isInRoute_notOnRoute)
{
  EXPECT_FALSE(hdmap_utils.isInRoute(34468, lanelet::Ids{34741, 34850, 34603, 34777}));
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(HdMapUtilsTest_StandardMap, isInRoute_empty)
{
  EXPECT_FALSE(hdmap_utils.isInRoute(34468, lanelet::Ids{}));
}

/**
 * @note Test basic functionality.
 * Test in lanelet presence correctness
 * with a position that is in the given lanelet.
 */
TEST_F(HdMapUtilsTest_StandardMap, isInLanelet_correct)
{
  EXPECT_TRUE(hdmap_utils.isInLanelet(34696, 10.0));
}

/**
 * @note Test basic functionality.
 * Test in lanelet presence correctness
 * with a position that is after the given lanelet.
 */
TEST_F(HdMapUtilsTest_StandardMap, isInLanelet_after)
{
  const lanelet::Id lanelet_id = 34696;
  EXPECT_FALSE(hdmap_utils.isInLanelet(lanelet_id, hdmap_utils.getLaneletLength(lanelet_id) + 5.0));
}

/**
 * @note Test basic functionality.
 * Test in lanelet presence correctness
 * with a position that is before the given lanelet.
 */
TEST_F(HdMapUtilsTest_StandardMap, isInLanelet_before)
{
  EXPECT_FALSE(hdmap_utils.isInLanelet(34696, -5.0));
}

/**
 * @note Test basic functionality.
 * Test lanelet to map point transform correctness
 * with a vector of several s larger than 0 but smaller than the lanelet length.
 */
TEST_F(HdMapUtilsTest_StandardMap, toMapPoints_correctPoints)
{
  const auto points = hdmap_utils.toMapPoints(34696, std::vector<double>{10.0, 20.0, 30.0});

  EXPECT_EQ(points.size(), static_cast<std::size_t>(3));
  EXPECT_POINT_NEAR(points[0], makePoint(3768.7, 73696.2, 1.9), 0.1);
  EXPECT_POINT_NEAR(points[1], makePoint(3759.8, 73691.6, 2.1), 0.1);
  EXPECT_POINT_NEAR(points[2], makePoint(3750.9, 73687.1, 2.3), 0.1);
}

/**
 * @note Test function behavior when called with a negative s.
 */
TEST_F(HdMapUtilsTest_StandardMap, toMapPoints_negativeS)
{
  const auto points = hdmap_utils.toMapPoints(34696, std::vector<double>{-10.0, -20.0, -30.0});

  EXPECT_EQ(points.size(), static_cast<std::size_t>(3));

  EXPECT_POINT_NEAR(points[0], makePoint(3786.5, 73705.3, 1.5), 0.1);
  EXPECT_POINT_NEAR(points[1], makePoint(3795.4, 73709.9, 1.3), 0.1);
  EXPECT_POINT_NEAR(points[2], makePoint(3804.3, 73714.5, 1.1), 0.1);
}

/**
 * @note Test function behavior when called with a value of s larger than the lanelet length.
 */
TEST_F(HdMapUtilsTest_StandardMap, toMapPoints_sLargerThanLaneletLength)
{
  const lanelet::Id lanelet_id = 34696;

  const auto lanelet_length = hdmap_utils.getLaneletLength(lanelet_id);
  const auto points = hdmap_utils.toMapPoints(
    lanelet_id,
    std::vector<double>{lanelet_length + 10.0, lanelet_length + 20.0, lanelet_length + 30.0});

  EXPECT_EQ(points.size(), static_cast<std::size_t>(3));
  EXPECT_POINT_NEAR(points[0], makePoint(3725.8, 73674.2, 3.0), 0.1);
  EXPECT_POINT_NEAR(points[1], makePoint(3716.9, 73669.6, 3.1), 0.1);
  EXPECT_POINT_NEAR(points[2], makePoint(3708.0, 73665.0, 3.3), 0.1);
}

/**
 * @note Test function behavior when called with an empty vector.
 */
TEST_F(HdMapUtilsTest_StandardMap, toMapPoints_empty)
{
  std::vector<geometry_msgs::msg::Point> points;

  EXPECT_NO_THROW(points = hdmap_utils.toMapPoints(34696, {}));

  EXPECT_TRUE(points.empty());
}

/**
 * @note Test basic functionality.
 * Test lanelet to map pose transform correctness
 * with a position on the lanelet and a small offset (e.g. 0.5) - test the specialization
 * taking a lanelet id, s and an offset as parameters.
 */
TEST_F(HdMapUtilsTest_StandardMap, toMapPose_onlyOffset)
{
  const auto map_pose =
    hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34696, 10.0, 0.5));

  EXPECT_STREQ(map_pose.header.frame_id.c_str(), "map");
  EXPECT_POSE_NEAR(
    map_pose.pose, makePose(makePoint(3768.9, 73695.8, 1.9), makeQuaternionFromYaw(-2.667)), 0.1);
}

/**
 * @note Test basic functionality.
 * Test lanelet to map pose transform correctness with a position
 * on the lanelet and additional rotation of 90 degrees
 * - test the specialization taking a lanelet pose object as a parameter.
 */
TEST_F(HdMapUtilsTest_StandardMap, toMapPose_additionalRotation)
{
  const auto map_pose = hdmap_utils.toMapPose(
    traffic_simulator::helper::constructLaneletPose(34696, 10.0, 0.0, 0.0, 0.0, M_PI_4));

  EXPECT_STREQ(map_pose.header.frame_id.c_str(), "map");
  EXPECT_POSE_NEAR(
    map_pose.pose,
    makePose(makePoint(3768.7, 73696.2, 1.9), makeQuaternionFromYaw(-2.667 + M_PI_4)), 0.1);
}

/**
 * @note Test function behavior when called with a negative s.
 */
TEST_F(HdMapUtilsTest_StandardMap, toMapPose_negativeS)
{
  geometry_msgs::msg::PoseStamped map_pose;
  EXPECT_NO_THROW(
    map_pose =
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34696, -10.0)));

  EXPECT_STREQ(map_pose.header.frame_id.c_str(), "map");
  EXPECT_POSE_NEAR(
    map_pose.pose, makePose(makePoint(3784.0, 73707.8, 1.5), makeQuaternionFromYaw(-1.595)), 0.1);
}

/**
 * @note Test function behavior when called with a value of s larger than the lanelet length.
 */
TEST_F(HdMapUtilsTest_StandardMap, toMapPose_sLargerThanLaneletLength)
{
  const lanelet::Id lanelet_id = 34696;

  geometry_msgs::msg::PoseStamped map_pose;
  EXPECT_NO_THROW(
    map_pose = hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(
      lanelet_id, hdmap_utils.getLaneletLength(lanelet_id) + 10.0)));

  EXPECT_STREQ(map_pose.header.frame_id.c_str(), "map");
  EXPECT_POSE_NEAR(
    map_pose.pose, makePose(makePoint(3724.9, 73678.1, 2.7), makeQuaternionFromYaw(2.828)), 0.1);
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining with a lanelet
 * that has no changeable lanelets and direction = STRAIGHT.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getLaneChangeableLaneletId_straight)
{
  const lanelet::Id start_and_end_lanelet = 199;
  const auto result_lanelet = hdmap_utils.getLaneChangeableLaneletId(
    start_and_end_lanelet, traffic_simulator::lane_change::Direction::STRAIGHT);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(start_and_end_lanelet, result_lanelet);
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining
 * with a lanelet that has no changeable lanelets and direction = LEFT.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getLaneChangeableLaneletId_leftNoChangeable)
{
  EXPECT_FALSE(
    hdmap_utils.getLaneChangeableLaneletId(199, traffic_simulator::lane_change::Direction::LEFT)
      .has_value());
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining with
 * a lanelet that has changeable lanelets (left direction) and direction = LEFT.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getLaneChangeableLaneletId_leftChangeable)
{
  const auto result_lanelet =
    hdmap_utils.getLaneChangeableLaneletId(200, traffic_simulator::lane_change::Direction::LEFT);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), 199);
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining
 * with a lanelet that has no changeable lanelets and direction = RIGHT.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getLaneChangeableLaneletId_rightNoChangeable)
{
  EXPECT_FALSE(
    hdmap_utils.getLaneChangeableLaneletId(202, traffic_simulator::lane_change::Direction::RIGHT)
      .has_value());
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining with
 * a lanelet that has changeable lanelets (right direction) and direction = RIGHT.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getLaneChangeableLaneletId_rightChangeable)
{
  const auto result_lanelet =
    hdmap_utils.getLaneChangeableLaneletId(200, traffic_simulator::lane_change::Direction::RIGHT);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), 201);
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining
 * with a lanelet that has at least two changeable lanes to the left,
 * direction = LEFT and shift = 2.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getLaneChangeableLaneletId_shift2LeftPossible)
{
  const auto result_lanelet =
    hdmap_utils.getLaneChangeableLaneletId(201, traffic_simulator::lane_change::Direction::LEFT, 2);

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
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getLaneChangeableLaneletId_shift2LeftNotPossible)
{
  EXPECT_FALSE(
    hdmap_utils.getLaneChangeableLaneletId(200, traffic_simulator::lane_change::Direction::LEFT, 2)
      .has_value());
}

/**
 * @note Test basic functionality.
 * Test changeable lanelets id obtaining
 * with a lanelet that has at least two changeable lanes to the right,
 * direction = RIGHT and shift = 2.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getLaneChangeableLaneletId_shift2RightPossible)
{
  const auto result_lanelet = hdmap_utils.getLaneChangeableLaneletId(
    200, traffic_simulator::lane_change::Direction::RIGHT, 2);

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
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getLaneChangeableLaneletId_shift2RightNotPossible)
{
  EXPECT_FALSE(
    hdmap_utils.getLaneChangeableLaneletId(201, traffic_simulator::lane_change::Direction::RIGHT, 2)
      .has_value());
}

/**
 * @note Test function behavior when called with a direction = RIGHT and shift = 0.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getLaneChangeableLaneletId_shift0)
{
  const lanelet::Id start_and_end_lanelet = 201;
  const auto result_lanelet = hdmap_utils.getLaneChangeableLaneletId(
    start_and_end_lanelet, traffic_simulator::lane_change::Direction::RIGHT, 0);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), start_and_end_lanelet);
}

/**
 * @note Test basic functionality.
 * Test traffic lights id obtaining correctness.
 */
TEST_F(HdMapUtilsTest_StandardMap, getTrafficLightIds_correct)
{
  auto result_traffic_lights = hdmap_utils.getTrafficLightIds();

  std::sort(result_traffic_lights.begin(), result_traffic_lights.end());
  EXPECT_EQ(result_traffic_lights, (lanelet::Ids{34802, 34836}));
}

/**
 * @note Test function behavior when there are no traffic lights on the map.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getTrafficLightIds_noTrafficLight)
{
  EXPECT_EQ(hdmap_utils.getTrafficLightIds().size(), static_cast<size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test traffic light position obtaining
 * with a traffic light and bulb color specified.
 */
TEST_F(HdMapUtilsTest_StandardMap, getTrafficLightBulbPosition_correct)
{
  const lanelet::Id light_id = 34802;
  const double epsilon = 0.1;

  {
    const auto return_bulb_position = hdmap_utils.getTrafficLightBulbPosition(light_id, "green");

    EXPECT_TRUE(return_bulb_position.has_value());
    EXPECT_POINT_NEAR(return_bulb_position.value(), makePoint(3761.05, 73755.30, 5.35), epsilon);
  }

  {
    const auto return_bulb_position = hdmap_utils.getTrafficLightBulbPosition(light_id, "yellow");

    EXPECT_TRUE(return_bulb_position.has_value());
    EXPECT_POINT_NEAR(return_bulb_position.value(), makePoint(3760.60, 73755.07, 5.35), epsilon);
  }

  {
    const auto return_bulb_position = hdmap_utils.getTrafficLightBulbPosition(light_id, "red");

    EXPECT_TRUE(return_bulb_position.has_value());
    EXPECT_POINT_NEAR(return_bulb_position.value(), makePoint(3760.16, 73754.87, 5.35), epsilon);
  }

  {
    EXPECT_FALSE(hdmap_utils.getTrafficLightBulbPosition(light_id, "pink").has_value());
  }
}

/**
 * @note Test basic functionality.
 * Test traffic light position obtaining
 * with an id of a traffic light that does not exist
 * - the goal is to test the branch when no traffic light is selected.
 */
TEST_F(HdMapUtilsTest_StandardMap, getTrafficLightBulbPosition_invalidTrafficLight)
{
  EXPECT_FALSE(hdmap_utils.getTrafficLightBulbPosition(1000003, "red").has_value());
}

/**
 * @note Test basic functionality.
 * Test obtaining conflicting lanelets correctness
 * with lanelets that do conflict with other lanelets.
 */
TEST_F(HdMapUtilsTest_StandardMap, getConflictingLaneIds_conflicting)
{
  lanelet::Ids actual_ids = {34495, 34498};
  auto result_ids = hdmap_utils.getConflictingLaneIds({34510});

  std::sort(actual_ids.begin(), actual_ids.end());
  std::sort(result_ids.begin(), result_ids.end());
  EXPECT_EQ(actual_ids, result_ids);
}

/**
 * @note Test basic functionality.
 * Test obtaining conflicting lanelets correctness
 * with lanelets that do not conflict with any other lanelets.
 */
TEST_F(HdMapUtilsTest_StandardMap, getConflictingLaneIds_notConflicting)
{
  EXPECT_EQ(hdmap_utils.getConflictingLaneIds({34513}).size(), static_cast<std::size_t>(0));
}

/**
 * @note Test function behavior when called with an empty vector.
 */
TEST_F(HdMapUtilsTest_StandardMap, getConflictingLaneIds_empty)
{
  EXPECT_EQ(hdmap_utils.getConflictingLaneIds({}).size(), static_cast<std::size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test obtaining conflicting crosswalk lanelets
 * correctness with lanelets that do conflict with crosswalk lanelets.
 */
TEST_F(HdMapUtilsTest_StandardMap, getConflictingCrosswalkIds_conflicting)
{
  lanelet::Ids actual_ids = {34399, 34385};
  auto result_ids = hdmap_utils.getConflictingCrosswalkIds({34633});

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
TEST_F(HdMapUtilsTest_StandardMap, getConflictingCrosswalkIds_notConflictingWithCrosswalk)
{
  EXPECT_EQ(hdmap_utils.getConflictingCrosswalkIds({34510}).size(), static_cast<std::size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test obtaining conflicting crosswalk lanelets
 * correctness with lanelets that do not conflict with any other lanelets.
 */
TEST_F(HdMapUtilsTest_StandardMap, getConflictingCrosswalkIds_notConflicting)
{
  EXPECT_EQ(hdmap_utils.getConflictingCrosswalkIds({34513}).size(), static_cast<std::size_t>(0));
}

/**
 * @note Test function behavior when called with an empty vector.
 */
TEST_F(HdMapUtilsTest_StandardMap, getConflictingCrosswalkIds_empty)
{
  EXPECT_EQ(hdmap_utils.getConflictingCrosswalkIds({}).size(), static_cast<std::size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test clipping trajectory correctness
 * with a correct vector of lanelets and the reference lanelet
 * also correct and reasonable forward distance.
 */
TEST_F(HdMapUtilsTest_StandardMap, clipTrajectoryFromLaneletIds_correct)
{
  const lanelet::Id start_id = 34600;
  const auto result_trajectory = hdmap_utils.clipTrajectoryFromLaneletIds(
    start_id, 40.0, lanelet::Ids{start_id, 34594, 34621}, 10.0);

  const std::vector<geometry_msgs::msg::Point> actual_trajectory{
    makePoint(3785.5, 73754.7, -0.5), makePoint(3784.6, 73754.2, -0.5),
    makePoint(3783.7, 73753.8, -0.5), makePoint(3782.9, 73753.3, -0.5),
    makePoint(3782.0, 73752.9, -0.5), makePoint(3781.1, 73752.4, -0.4),
    makePoint(3780.2, 73751.9, -0.4), makePoint(3779.3, 73751.5, -0.4),
    makePoint(3778.4, 73751.0, -0.4), makePoint(3777.5, 73750.6, -0.4)};

  EXPECT_EQ(result_trajectory.size(), actual_trajectory.size());
  for (std::size_t i = 0; i < actual_trajectory.size(); ++i) {
    EXPECT_POINT_NEAR_STREAM(
      result_trajectory[i], actual_trajectory[i], 0.1, "In this test i = " << i);
  }
}

/**
 * @note Test basic functionality.
 * Test clipping trajectory correctness
 * with a correct vector of lanelets and the reference
 * lanelet not on the trajectory and reasonable forward distance.
 */
TEST_F(HdMapUtilsTest_StandardMap, clipTrajectoryFromLaneletIds_startNotOnTrajectory)
{
  EXPECT_EQ(
    hdmap_utils.clipTrajectoryFromLaneletIds(34606, 40.0, lanelet::Ids{34600, 34594, 34621}, 10.0)
      .size(),
    static_cast<std::size_t>(0));
}

/**
 * @note Test function behavior when passed an empty trajectory vector.
 */
TEST_F(HdMapUtilsTest_StandardMap, clipTrajectoryFromLaneletIds_emptyTrajectory)
{
  EXPECT_EQ(
    hdmap_utils.clipTrajectoryFromLaneletIds(34600, 40.0, lanelet::Ids{}, 10.0).size(),
    static_cast<std::size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test clipping trajectory correctness
 * with a correct vector of lanelets, and the reference lanelet
 * also correct and forward distance fairly small (e.g. 2).
 */
TEST_F(HdMapUtilsTest_StandardMap, clipTrajectoryFromLaneletIds_smallForwardDistance)
{
  const lanelet::Id start_id = 34600;

  const auto result_trajectory = hdmap_utils.clipTrajectoryFromLaneletIds(
    start_id, 40.0, lanelet::Ids{start_id, 34594, 34621}, 1.5);

  constexpr double epsilon = 0.1;

  EXPECT_EQ(result_trajectory.size(), static_cast<std::size_t>(2));
  EXPECT_POINT_NEAR(result_trajectory[0], makePoint(3785.5, 73754.7, -0.5), epsilon)
  EXPECT_POINT_NEAR(result_trajectory[1], makePoint(3784.6, 73754.2, -0.5), epsilon);
}

/**
 * @note Test basic functionality.
 * Test following lanelets obtaining with
 * a lanelet that has lanelets after it longer than parameter distance.
 */
TEST_F(HdMapUtilsTest_StandardMap, getFollowingLanelets_straightAfter)
{
  const lanelet::Id id = 120660;
  EXPECT_EQ(hdmap_utils.getFollowingLanelets(id, 1.0, true), (lanelet::Ids{id, 34468}));
}

/**
 * @note Test basic functionality.
 * Test following lanelets obtaining with a lanelet
 * that has lanelets after it longer than parameter distance, but the following lanelets
 * go through a curve (e.g there was an order to go right earlier on the lane).
 */
TEST_F(HdMapUtilsTest_StandardMap, getFollowingLanelets_curveAfter)
{
  const lanelet::Id id = 34564;
  EXPECT_EQ(hdmap_utils.getFollowingLanelets(id, 40.0, true), (lanelet::Ids{id, 34411, 34462}));
}

/**
 * @note Test basic functionality.
 * Test following lanelets obtaining with a lanelet
 * that has lanelets after it for less than specified in the distance parameter
 * - the goal is for the function to return trajectory shorter than distance specified.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getFollowingLanelets_notEnoughLaneletsAfter)
{
  const lanelet::Id id = 199;
  EXPECT_EQ(hdmap_utils.getFollowingLanelets(id, 1.0e3, true), (lanelet::Ids{id, 203}));
}

/**
 * @note Test basic functionality.
 * Test following lanelets obtaining
 * with a candidate trajectory longer than the given distance.
 */
TEST_F(HdMapUtilsTest_StandardMap, getFollowingLanelets_candidateTrajectory)
{
  const lanelet::Id id = 34564;
  EXPECT_EQ(
    hdmap_utils.getFollowingLanelets(id, lanelet::Ids{id, 34495, 34507, 34795, 34606}, 40.0, true),
    (lanelet::Ids{id, 34495, 34507}));
}

/**
 * @note Test basic functionality.
 * Test following lanelets obtaining with
 * a candidate trajectory shorter than the given distance
 * - the goal is to test generating lacking part of the trajectory.
 */
TEST_F(HdMapUtilsTest_StandardMap, getFollowingLanelets_candidateTrajectoryNotEnough)
{
  const lanelet::Id id = 34564;
  EXPECT_EQ(
    hdmap_utils.getFollowingLanelets(id, lanelet::Ids{id, 34495, 34507}, 100.0, true),
    (lanelet::Ids{id, 34495, 34507, 34795, 34606}));
}

/**
 * @note Test function behavior when called with a candidate trajectory
 * that does not contain the starting lanelet.
 */
TEST_F(HdMapUtilsTest_StandardMap, getFollowingLanelets_candidatesDoNotMatch)
{
  EXPECT_THROW(
    hdmap_utils.getFollowingLanelets(120660, lanelet::Ids{34981}, 1.0e3, true), common::Error);
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(HdMapUtilsTest_StandardMap, getFollowingLanelets_candidateTrajectoryEmpty)
{
  EXPECT_EQ(
    hdmap_utils.getFollowingLanelets(120660, {}, 1.0e3, true).size(), static_cast<std::size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test lane change possibility checking
 * correctness with lanelets that can be changed.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, canChangeLane_canChange)
{
  EXPECT_TRUE(hdmap_utils.canChangeLane(199, 200));
}

/**
 * @note Test basic functionality.
 * Test lane change possibility checking correctness with lanelets
 * that can not be changed (e.g. goal lanelet is behind the start lanelet).
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, canChangeLane_canNotChange)
{
  EXPECT_FALSE(hdmap_utils.canChangeLane(199, 201));
}

/**
 * @note Test function behavior when either of the lanelet ids is invalid.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, canChangeLane_invalidLaneletId)
{
  EXPECT_THROW(hdmap_utils.canChangeLane(1000003, 1000033), std::runtime_error);
}

/**
 * @note Test basic functionality.
 * Test lateral distance calculation correctness
 * with two lanelet poses on the same lanelet but with different offsets.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getLateralDistance_sameLane)
{
  const auto from = traffic_simulator::helper::constructLaneletPose(3002185, 0.0, 0.5);
  const auto to = traffic_simulator::helper::constructLaneletPose(3002185, 10.0, 0.2);
  const auto result = hdmap_utils.getLateralDistance(from, to);

  EXPECT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), to.offset - from.offset, 1e-3);
}

/**
 * @note Test basic functionality.
 * Test lateral distance calculation correctness
 * with two lanelet poses on parallel lanes with no possibility of changing the lane.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getLateralDistance_parallelLanesCanNotChange)
{
  EXPECT_FALSE(hdmap_utils
                 .getLateralDistance(
                   traffic_simulator::helper::constructLaneletPose(3002185, 0.0, 0.5),
                   traffic_simulator::helper::constructLaneletPose(3002184, 10.0, 0.2), false)
                 .has_value());
}

/**
 * @note Test basic functionality.
 * Test lateral distance calculation correctness
 * with two lanelet poses on parallel lanes with a possibility of changing the lane.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getLateralDistance_parallelLanesCanChange)
{
  const auto from = traffic_simulator::helper::constructLaneletPose(3002185, 0.0, 0.5);
  const auto to = traffic_simulator::helper::constructLaneletPose(3002184, 10.0, 0.2);

  const auto result = hdmap_utils.getLateralDistance(from, to, true);

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
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getLateralDistance_notConnected)
{
  EXPECT_FALSE(hdmap_utils
                 .getLateralDistance(
                   traffic_simulator::helper::constructLaneletPose(3002185, 0.0, 0.5),
                   traffic_simulator::helper::constructLaneletPose(3002166, 10.0, 0.2), true)
                 .has_value());
}

/**
 * @note Test basic functionality.
 * Test route obtaining correctness with a feasible route.
 */
TEST_F(HdMapUtilsTest_StandardMap, getRoute_correct)
{
  EXPECT_EQ(
    hdmap_utils.getRoute(34579, 34630, true),
    (lanelet::Ids{34579, 34774, 120659, 120660, 34468, 34438, 34408, 34624, 34630}));
}

/**
 * @note Test basic functionality.
 * Test route obtaining correctness with a feasible route and obtain it two times
 * - the goal is to test whether the route cache works correctly.
 */
TEST_F(HdMapUtilsTest_StandardMap, getRoute_correctCache)
{
  const lanelet::Id from_id = 34579;
  const lanelet::Id to_id = 34630;
  const bool allow_lane_change = true;

  EXPECT_EQ(
    hdmap_utils.getRoute(from_id, to_id, allow_lane_change),
    hdmap_utils.getRoute(from_id, to_id, allow_lane_change));
}

/**
 * @note Test basic functionality.
 * Test route obtaining correctness with the beginning
 * and ending that are impossible to route between.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getRoute_impossibleRouting)
{
  EXPECT_EQ(hdmap_utils.getRoute(199, 196, true).size(), static_cast<std::size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test route obtaining correctness with beginning
 * and ending of the route set to the same lanelet id.
 */
TEST_F(HdMapUtilsTest_StandardMap, getRoute_circular)
{
  const lanelet::Id from_and_to_id = 120659;

  EXPECT_EQ(
    hdmap_utils.getRoute(from_and_to_id, from_and_to_id, false), lanelet::Ids{from_and_to_id});
}

/**
 * @note Test basic functionality with a lanelet that has a centerline with 3 or more points.
 */
TEST_F(HdMapUtilsTest_StandardMap, getCenterPoints_correct)
{
  const std::vector<geometry_msgs::msg::Point> actual_center_points{
    makePoint(3774.1, 73748.8, -0.3), makePoint(3772.5, 73748.0, -0.2),
    makePoint(3770.8, 73747.1, -0.2), makePoint(3769.2, 73746.3, -0.2),
    makePoint(3767.6, 73745.4, -0.2), makePoint(3766.0, 73744.6, -0.1),
    makePoint(3764.4, 73743.8, -0.1), makePoint(3762.7, 73742.9, -0.1),
    makePoint(3761.1, 73742.1, 0.0),  makePoint(3759.5, 73741.3, 0.0),
    makePoint(3757.9, 73740.4, 0.1),  makePoint(3756.3, 73739.6, 0.1)};

  const auto result_center_points = hdmap_utils.getCenterPoints(34594);

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
TEST_F(HdMapUtilsTest_StandardMap, getCenterPoints_correctCache)
{
  const lanelet::Id id = 34594;

  const auto result_center_points = hdmap_utils.getCenterPoints(id);
  const auto result_center_points2 = hdmap_utils.getCenterPoints(id);

  EXPECT_EQ(result_center_points.size(), result_center_points2.size());
  for (std::size_t i = 0; i < result_center_points.size(); ++i) {
    EXPECT_POINT_EQ_STREAM(
      result_center_points[i], result_center_points2[i], "In this test i = " << i);
  }
}

/**
 * @note Test basic functionality with a vector containing valid lanelets.
 */
TEST_F(HdMapUtilsTest_StandardMap, getCenterPoints_correctVector)
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

  const auto result_center_points = hdmap_utils.getCenterPoints(lanelet::Ids{34594, 34621});

  EXPECT_EQ(result_center_points.size(), actual_center_points.size());
  for (std::size_t i = 0; i < result_center_points.size(); ++i) {
    EXPECT_POINT_NEAR_STREAM(
      result_center_points[i], actual_center_points[i], 0.1, "In this test i = " << i);
  }
}

/**
 * @note Test basic functionality with an empty lanelet vector.
 */
TEST_F(HdMapUtilsTest_StandardMap, getCenterPoints_empty)
{
  EXPECT_EQ(hdmap_utils.getCenterPoints(lanelet::Ids{}).size(), static_cast<std::size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test traffic light checking correctness with an id of a traffic light.
 */
TEST_F(HdMapUtilsTest_StandardMap, isTrafficLight_trafficLight)
{
  EXPECT_TRUE(hdmap_utils.isTrafficLight(34836));
}

/**
 * @note Test basic functionality.
 * Test traffic light checking correctness with an id of not a traffic light.
 */
TEST_F(HdMapUtilsTest_StandardMap, isTrafficLight_notTrafficLight)
{
  EXPECT_FALSE(hdmap_utils.isTrafficLight(120659));
}

/**
 * @note Test function behavior when called with an invalid lanelet id.
 */
TEST_F(HdMapUtilsTest_StandardMap, isTrafficLight_invalidId)
{
  EXPECT_FALSE(hdmap_utils.isTrafficLight(1000003));
}

/**
 * @note Test basic functionality.
 * Test traffic light relation checking correctness
 * with an id of a lanelet that has a relation with a traffic light.
 */
TEST_F(HdMapUtilsTest_StandardMap, isTrafficLightRegulatoryElement_trafficLightRegulatoryElement)
{
  EXPECT_TRUE(hdmap_utils.isTrafficLightRegulatoryElement(34806));
}

/**
 * @note Test basic functionality.
 * Test traffic light relation checking correctness
 * with an id of a lanelet that does not have a relation with a traffic light.
 */
TEST_F(HdMapUtilsTest_StandardMap, isTrafficLightRegulatoryElement_noTrafficLightRegulatoryElement)
{
  EXPECT_FALSE(hdmap_utils.isTrafficLightRegulatoryElement(120659));
}

/**
 * @note Test function behavior when called with an invalid lanelet id.
 */
TEST_F(HdMapUtilsTest_StandardMap, isTrafficLightRegulatoryElement_invalidId)
{
  EXPECT_FALSE(hdmap_utils.isTrafficLightRegulatoryElement(1000003));
}

/**
 * @note Test basic functionality. Test lanelet length obtaining with some lanelet id.
 */
TEST_F(HdMapUtilsTest_StandardMap, getLaneletLength_simple)
{
  EXPECT_NEAR(hdmap_utils.getLaneletLength(34468), 55.5, 1.0);
}

/**
 * @note Test basic functionality.
 * Test lanelet length obtaining with some lanelet id two times
 * (the same lanelet id) - the goal is to test lanelet length caching correctness.
 */
TEST_F(HdMapUtilsTest_StandardMap, getLaneletLength_cache)
{
  const lanelet::Id id = 34468;

  EXPECT_EQ(hdmap_utils.getLaneletLength(id), hdmap_utils.getLaneletLength(id));
}

/**
 * @note Test basic functionality.
 * Test traffic light ids obtaining correctness
 * with a route that does not have any traffic lights.
 */
TEST_F(HdMapUtilsTest_StandardMap, getTrafficLightIdsOnPath_noTrafficLights)
{
  EXPECT_EQ(
    hdmap_utils.getTrafficLightIdsOnPath(lanelet::Ids{34579, 34774, 120659, 120660, 34468, 34438})
      .size(),
    static_cast<size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test traffic light ids obtaining correctness with a route that has some traffic lights.
 */
TEST_F(HdMapUtilsTest_StandardMap, getTrafficLightIdsOnPath_trafficLights)
{
  auto result_traffic_light_ids = hdmap_utils.getTrafficLightIdsOnPath(
    {34579, 34774, 120659, 120660, 34468, 34438, 34408, 34624, 34630});
  auto actual_traffic_light_ids = lanelet::Ids{34802, 34836};

  std::sort(result_traffic_light_ids.begin(), result_traffic_light_ids.end());
  std::sort(actual_traffic_light_ids.begin(), actual_traffic_light_ids.end());

  EXPECT_EQ(result_traffic_light_ids, actual_traffic_light_ids);
}

/**
 * @note Test function behavior when passed an empty vector of lanelet ids.
 */
TEST_F(HdMapUtilsTest_StandardMap, getTrafficLightIdsOnPath_empty)
{
  EXPECT_EQ(hdmap_utils.getTrafficLightIdsOnPath({}).size(), static_cast<size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test longitudinal distance calculation correctness
 * with two poses on the same lanelet, where the goal pose is positioned in front of the start pose.
 */
TEST_F(HdMapUtilsTest_StandardMap, getLongitudinalDistance_sameLanelet)
{
  auto pose_from = hdmap_utils.toLaneletPose(
    makePose(makePoint(3812.65, 73810.13, -2.80), makeQuaternionFromYaw(90.0)), lanelet::Id{34606});
  auto pose_to = hdmap_utils.toLaneletPose(
    makePose(makePoint(3825.10, 73786.34, -1.82), makeQuaternionFromYaw(90.0)), lanelet::Id{34606});
  EXPECT_TRUE(pose_from.has_value());
  EXPECT_TRUE(pose_to.has_value());

  const auto result_distance =
    hdmap_utils.getLongitudinalDistance(pose_from.value(), pose_to.value(), false);

  EXPECT_TRUE(result_distance.has_value());
  EXPECT_NEAR(result_distance.value(), 27.0, 1.0);
}

/**
 * @note Test basic functionality.
 * Test longitudinal distance calculation correctness
 * with two poses on the same lanelet, where the goal pose is positioned behind the start pose.
 */
TEST_F(HdMapUtilsTest_StandardMap, getLongitudinalDistance_sameLaneletBehind)
{
  auto pose_to = hdmap_utils.toLaneletPose(
    makePose(makePoint(3812.65, 73810.13, -2.80), makeQuaternionFromYaw(90.0)), lanelet::Id{34606});
  auto pose_from = hdmap_utils.toLaneletPose(
    makePose(makePoint(3825.10, 73786.34, -1.82), makeQuaternionFromYaw(90.0)), lanelet::Id{34606});
  EXPECT_TRUE(pose_from.has_value());
  EXPECT_TRUE(pose_to.has_value());

  EXPECT_FALSE(
    hdmap_utils.getLongitudinalDistance(pose_from.value(), pose_to.value(), false).has_value());
}

/**
 * @note Test basic functionality.
 * Test longitudinal distance calculation correctness
 * with two poses on different lanelets  that are a few lanelets apart (e.g. 3).
 */
TEST_F(HdMapUtilsTest_StandardMap, getLongitudinalDistance_differentLanelet)
{
  auto pose_from =
    hdmap_utils.toLaneletPose(makePose(makePoint(3801.19, 73812.70, -2.86)), lanelet::Id{120660});
  auto pose_to =
    hdmap_utils.toLaneletPose(makePose(makePoint(3724.70, 73773.00, -1.20)), lanelet::Id{34462});
  EXPECT_TRUE(pose_from.has_value());
  EXPECT_TRUE(pose_to.has_value());

  const auto result_distance =
    hdmap_utils.getLongitudinalDistance(pose_from.value(), pose_to.value(), false);

  EXPECT_TRUE(result_distance.has_value());
  EXPECT_NEAR(result_distance.value(), 86.0, 1.0);
}

/**
 * @note Test basic functionality. Test longitudinal distance calculation correctness
 * with two poses on different lanelets where the goal pose is on lanelet unreachable
 * from the start pose lanelet - the goal is to test the branch of execution where no route is found.
 */
TEST_F(HdMapUtilsTest_FourTrackHighwayMap, getLongitudinalDistance_differentLaneletNoRoute)
{
  auto pose_to = hdmap_utils.toLaneletPose(
    makePose(makePoint(81590.79, 50067.66), makeQuaternionFromYaw(90.0)), lanelet::Id{3002185});
  auto pose_from = hdmap_utils.toLaneletPose(
    makePose(makePoint(81596.20, 50068.04), makeQuaternionFromYaw(90.0)), lanelet::Id{3002166});
  EXPECT_TRUE(pose_from.has_value());
  EXPECT_TRUE(pose_to.has_value());

  EXPECT_FALSE(
    hdmap_utils.getLongitudinalDistance(pose_from.value(), pose_to.value(), false).has_value());
}

/**
 * @note Test for the corner-case fixed in https://github.com/tier4/scenario_simulator_v2/pull/1348.
 */
TEST_F(HdMapUtilsTest_KashiwanohaMap, getLongitudinalDistance_PullRequest1348)
{
  auto pose_from = traffic_simulator::helper::constructLaneletPose(34468, 10.0);
  auto pose_to = traffic_simulator::helper::constructLaneletPose(34795, 5.0);

  EXPECT_NO_THROW(EXPECT_DOUBLE_EQ(
    hdmap_utils.getLongitudinalDistance(pose_from, pose_to, true).value(),
    54.18867466433655977198213804513216018676757812500000));
}

/**
 * @note Test basic functionality.
 * Test obtaining stop line ids correctness with a route that has no stop lines.
 */
TEST_F(HdMapUtilsTest_StandardMap, getStopLineIdsOnPath_noStopLines)
{
  EXPECT_EQ(
    hdmap_utils.getStopLineIdsOnPath({34507, 34795, 34606, 34672}).size(), static_cast<size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test obtaining stop line ids correctness with a route that has a stop line.
 */
TEST_F(HdMapUtilsTest_StandardMap, getStopLineIdsOnPath_someStopLines)
{
  EXPECT_EQ(
    hdmap_utils.getStopLineIdsOnPath({34408, 34633, 34579, 34780, 34675, 34744, 34690}),
    (lanelet::Ids{120635}));
}

/**
 * @note Test function behavior when passed an empty vector of lanelet ids.
 */
TEST_F(HdMapUtilsTest_StandardMap, getStopLineIdsOnPath_empty)
{
  EXPECT_EQ(hdmap_utils.getStopLineIdsOnPath(lanelet::Ids{}).size(), static_cast<size_t>(0));
}

/**
 * @note Test basic functionality.
 * Test obtaining traffic light stop line ids
 * correctness with a traffic light that has one stop line.
 */
TEST_F(HdMapUtilsTest_CrossroadsWithStoplinesMap, getTrafficLightStopLineIds_stopLine)
{
  EXPECT_EQ(hdmap_utils.getTrafficLightStopLineIds(34802), (lanelet::Ids{34805}));
}

/**
 * @note Test basic functionality.
 * Test obtaining traffic light stop line ids
 * correctness with a traffic light that has several stop lines
 * - the goal is to test the scenario where one traffic light has multiple stop lines
 * (e.g. on a road with two parallel lanes with the same direction).
 */
TEST_F(HdMapUtilsTest_CrossroadsWithStoplinesMap, getTrafficLightStopLineIds_severalStopLines)
{
  auto result_stoplines = hdmap_utils.getTrafficLightStopLineIds(34836);
  auto actual_stoplines = lanelet::Ids{120663, 34805};

  std::sort(result_stoplines.begin(), result_stoplines.end());
  std::sort(actual_stoplines.begin(), actual_stoplines.end());

  EXPECT_EQ(result_stoplines, actual_stoplines);
}

/**
 * @note Test function behavior when passed an invalid traffic light id.
 */
TEST_F(HdMapUtilsTest_CrossroadsWithStoplinesMap, getTrafficLightStopLineIds_invalidTrafficLightId)
{
  EXPECT_THROW(hdmap_utils.getTrafficLightStopLineIds(1000039), std::runtime_error);
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
 * Test obtaining traffic light stop line points correctness
 * with a traffic light id that has only one traffic light stop line.
 */
TEST_F(HdMapUtilsTest_CrossroadsWithStoplinesMap, getTrafficLightStopLinesPoints_stopLine)
{
  auto result_stoplines_points = hdmap_utils.getTrafficLightStopLinesPoints(34802);
  auto actual_stoplines_points = std::vector<std::vector<geometry_msgs::msg::Point>>{
    {makePoint(3762.0, 73756.0, -0.5), makePoint(3759.0, 73754.5, -0.5)}};

  EXPECT_EQ(result_stoplines_points.size(), actual_stoplines_points.size());

  sortStoplines(result_stoplines_points);
  sortStoplines(actual_stoplines_points);

  compareStoplines(actual_stoplines_points, result_stoplines_points);
}

/**
 * @note Test basic functionality.
 * Test obtaining traffic light stop line points correctness
 * with a traffic light id that has multiple traffic light stop lines.
 */
TEST_F(HdMapUtilsTest_CrossroadsWithStoplinesMap, getTrafficLightStopLinesPoints_severalStopLines)
{
  auto result_stoplines_points = hdmap_utils.getTrafficLightStopLinesPoints(34836);
  auto actual_stoplines_points = std::vector<std::vector<geometry_msgs::msg::Point>>{
    {makePoint(3762.0, 73756.0, -0.5), makePoint(3759.0, 73754.5, -0.5)},
    {makePoint(3768.5, 73737.0, -0.5), makePoint(3765.5, 73736.0, -0.5)}};

  EXPECT_EQ(result_stoplines_points.size(), actual_stoplines_points.size());

  sortStoplines(result_stoplines_points);
  sortStoplines(actual_stoplines_points);

  compareStoplines(actual_stoplines_points, result_stoplines_points);
}

/**
 * @note Test function behavior when passed an invalid traffic light id.
 */
TEST_F(
  HdMapUtilsTest_CrossroadsWithStoplinesMap, getTrafficLightStopLinesPoints_invalidTrafficLightId)
{
  EXPECT_THROW(hdmap_utils.getTrafficLightStopLinesPoints(1000039), std::runtime_error);
}

/**
 * @note Test basic functionality.
 * Test stop line polygon obtaining correctness with a lanelet that has a stop line.
 */
TEST_F(HdMapUtilsTest_CrossroadsWithStoplinesMap, getStopLinePolygon_stopLine)
{
  const auto result_stoplines_points = hdmap_utils.getStopLinePolygon(lanelet::Id{120663});
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
TEST_F(HdMapUtilsTest_CrossroadsWithStoplinesMap, getStopLinePolygon_invalidLaneletId)
{
  EXPECT_THROW(hdmap_utils.getStopLinePolygon(1000039), std::runtime_error);
}

/**
 * @note Test basic functionality.
 * Test distance to traffic light stop line obtaining
 * correctness with a spline and a traffic light id that has a stop line on the spline.
 */
TEST_F(
  HdMapUtilsTest_CrossroadsWithStoplinesMap, getDistanceToTrafficLightStopLine_trafficLightOnSpline)
{
  const auto start_waypoint = makePoint(3771.06, 73728.35);
  const auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine(
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
  HdMapUtilsTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_noTrafficLightOnSpline)
{
  EXPECT_FALSE(hdmap_utils
                 .getDistanceToTrafficLightStopLine(
                   math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
                     makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70),
                     makePoint(3773.19, 73723.27)}),
                   lanelet::Id{34836})
                 .has_value());
}

/**
 * @note Test basic functionality.
 * Test distance to traffic light stop line obtaining correctness
 * with a road (waypoints) and a traffic light id has a stop line on the road.
 */
TEST_F(
  HdMapUtilsTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_trafficLightOnWaypoints)
{
  const auto start_waypoint = makePoint(3771.06, 73728.35);
  const auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine(
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
  HdMapUtilsTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_noTrafficLightOnWaypoints)
{
  EXPECT_FALSE(
    hdmap_utils
      .getDistanceToTrafficLightStopLine(
        std::vector<geometry_msgs::msg::Point>{
          makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70), makePoint(3773.19, 73723.27)},
        lanelet::Id{34836})
      .has_value());
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(
  HdMapUtilsTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_emptyVector_waypoints)
{
  EXPECT_FALSE(hdmap_utils
                 .getDistanceToTrafficLightStopLine(
                   std::vector<geometry_msgs::msg::Point>{}, lanelet::Id{34836})
                 .has_value());
}

/**
 * @note Test basic functionality.
 * Test distance to traffic light stop line obtaining correctness
 * with a spline and a route that is coherent with the spline and has a traffic light on it.
 */
TEST_F(
  HdMapUtilsTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_routeWithTrafficLightsOnSpline)
{
  const auto start_waypoint = makePoint(3771.06, 73728.35);

  const auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine(
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
  HdMapUtilsTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_routeWithNoTrafficLightsOnSplineCongruent)
{
  EXPECT_FALSE(hdmap_utils
                 .getDistanceToTrafficLightStopLine(
                   lanelet::Ids{34690, 34759, 34576},
                   math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
                     makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70),
                     makePoint(3773.19, 73723.27)}))
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
  HdMapUtilsTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_routeWithTrafficLightsNotOnSplineIncongruent)
{
  EXPECT_FALSE(hdmap_utils
                 .getDistanceToTrafficLightStopLine(
                   lanelet::Ids{34576, 34570, 34564},
                   math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
                     makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70),
                     makePoint(3773.19, 73723.27)}))
                 .has_value());
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(
  HdMapUtilsTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_emptyVector_splineRoute)
{
  EXPECT_FALSE(
    hdmap_utils
      .getDistanceToTrafficLightStopLine(
        lanelet::Ids{}, math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
                          makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70),
                          makePoint(3773.19, 73723.27)}))
      .has_value());
}

/**
 * @note Test basic functionality.
 * Test distance to traffic light stop line obtaining correctness
 * with a road (waypoints) and a route that is coherent with the road and has a traffic light on it.
 */
TEST_F(
  HdMapUtilsTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_routeWithTrafficLightsOnWaypoints)
{
  const auto start_waypoint = makePoint(3771.06, 73728.35);

  const auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine(
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
  HdMapUtilsTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_routeWithNoTrafficLightsOnWaypointsIncongruent)
{
  EXPECT_FALSE(
    hdmap_utils
      .getDistanceToTrafficLightStopLine(
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
  HdMapUtilsTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_routeWithTrafficLightsNotOnWaypointsCongruent)
{
  EXPECT_FALSE(
    hdmap_utils
      .getDistanceToTrafficLightStopLine(
        lanelet::Ids{34576, 34570, 34564},
        std::vector<geometry_msgs::msg::Point>{
          makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70), makePoint(3773.19, 73723.27)})
      .has_value());
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(
  HdMapUtilsTest_CrossroadsWithStoplinesMap,
  getDistanceToTrafficLightStopLine_emptyVector_waypointsRoute)
{
  EXPECT_FALSE(
    hdmap_utils
      .getDistanceToTrafficLightStopLine(
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
TEST_F(HdMapUtilsTest_CrossroadsWithStoplinesMap, getDistanceToStopLine_stopLineOnSpline)
{
  const auto start_waypoint = makePoint(3821.86, 73777.20);
  const auto result_distance = hdmap_utils.getDistanceToStopLine(
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
TEST_F(HdMapUtilsTest_CrossroadsWithStoplinesMap, getDistanceToStopLine_noStopLineOnSplineCongruent)
{
  EXPECT_FALSE(hdmap_utils
                 .getDistanceToStopLine(
                   lanelet::Ids{34690, 34759, 34576},
                   math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
                     makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70),
                     makePoint(3773.19, 73723.27)}))
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
  HdMapUtilsTest_CrossroadsWithStoplinesMap, getDistanceToStopLine_noStopLineOnSplineIncongruent)
{
  EXPECT_FALSE(hdmap_utils
                 .getDistanceToStopLine(
                   lanelet::Ids{34576, 34570, 34564},
                   math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
                     makePoint(3821.86, 73777.20), makePoint(3837.28, 73762.67),
                     makePoint(3846.10, 73741.38)}))
                 .has_value());
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(HdMapUtilsTest_CrossroadsWithStoplinesMap, getDistanceToStopLine_emptyVector_spline)
{
  EXPECT_FALSE(
    hdmap_utils
      .getDistanceToStopLine(
        lanelet::Ids{}, math::geometry::CatmullRomSpline(std::vector<geometry_msgs::msg::Point>{
                          makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70),
                          makePoint(3773.19, 73723.27)}))
      .has_value());
}

/**
 * @note Test basic functionality.
 * Test distance to stop line calculation correctness
 * with a road (waypoints) and a route that is coherent with the road and has a stop line on it.
 */
TEST_F(HdMapUtilsTest_CrossroadsWithStoplinesMap, getDistanceToStopLine_stopLineOnWaypoints)
{
  const auto start_waypoint = makePoint(3821.86, 73777.20);
  const auto result_distance = hdmap_utils.getDistanceToStopLine(
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
  HdMapUtilsTest_CrossroadsWithStoplinesMap, getDistanceToStopLine_noStopLineOnWaypointsCongruent)
{
  EXPECT_FALSE(
    hdmap_utils
      .getDistanceToStopLine(
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
  HdMapUtilsTest_CrossroadsWithStoplinesMap, getDistanceToStopLine_noStopLineOnWaypointsIncongruent)
{
  EXPECT_FALSE(
    hdmap_utils
      .getDistanceToStopLine(
        lanelet::Ids{34576, 34570, 34564},
        std::vector<geometry_msgs::msg::Point>{
          makePoint(3821.86, 73777.20), makePoint(3837.28, 73762.67), makePoint(3846.10, 73741.38)})
      .has_value());
}

/**
 * @note Test function behavior when an empty vector is passed.
 */
TEST_F(HdMapUtilsTest_CrossroadsWithStoplinesMap, getDistanceToStopLine_emptyVector_waypoints)
{
  EXPECT_FALSE(
    hdmap_utils
      .getDistanceToStopLine(
        lanelet::Ids{},
        std::vector<geometry_msgs::msg::Point>{
          makePoint(3807.63, 73715.99), makePoint(3785.76, 73707.70), makePoint(3773.19, 73723.27)})
      .has_value());
}
