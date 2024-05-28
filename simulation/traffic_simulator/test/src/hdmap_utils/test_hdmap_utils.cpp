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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/helper/helper.hpp>

#include "../expect_eq_macros.hpp"

static const std::string map_path = "/map/lanelet2_map.osm";
static const std::string with_road_shoulder_map_path = "/map/with_road_shoulder/lanelet2_map.osm";
static const std::string empty_map_path = "/map/empty/lanelet2_map.osm";
static const std::string four_track_highway_map_path = "/map/four_track_highway/lanelet2_map.osm";
static const std::string crossroads_with_stoplines_map_path =
  "/map/crossroads_with_stoplines/lanelet2_map.osm";

auto makeHdMapUtilsInstance(const std::string relative_path = map_path) -> hdmap_utils::HdMapUtils
{
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + relative_path;
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  return hdmap_utils::HdMapUtils(path, origin);
}

auto makePoint(const double x, const double y, const double z = 0.0) -> geometry_msgs::msg::Point
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

auto makeBoundingBox(const double center_y = 0.0) -> traffic_simulator_msgs::msg::BoundingBox
{
  traffic_simulator_msgs::msg::BoundingBox bbox;
  bbox.center.x = 1.0;
  bbox.center.y = center_y;
  bbox.dimensions.x = 4.0;
  bbox.dimensions.y = 2.0;
  bbox.dimensions.z = 1.5;
  return bbox;
}

auto makePose(
  geometry_msgs::msg::Point position,
  geometry_msgs::msg::Quaternion orientation = geometry_msgs::msg::Quaternion())
  -> geometry_msgs::msg::Pose
{
  geometry_msgs::msg::Pose pose;
  pose.position = position;
  pose.orientation = orientation;
  return pose;
}

auto makeSmallBoundingBox(const double center_y = 0.0) -> traffic_simulator_msgs::msg::BoundingBox
{
  traffic_simulator_msgs::msg::BoundingBox bbox;
  bbox.center.x = 0.0;
  bbox.center.y = center_y;
  bbox.dimensions.x = 1.0;
  bbox.dimensions.y = 1.0;
  bbox.dimensions.z = 1.0;
  return bbox;
}

auto makeQuaternionFromYaw(const double yaw) -> geometry_msgs::msg::Quaternion
{
  geometry_msgs::msg::Vector3 v;
  v.z = yaw;
  return quaternion_operation::convertEulerAngleToQuaternion(v);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(HdMapUtils, Construct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  ASSERT_NO_THROW(hdmap_utils.toMapBin());
}

TEST(HdMapUtils, matchToLane)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  traffic_simulator_msgs::msg::BoundingBox bbox;
  bbox.center.x = 0.0;
  bbox.center.y = 0.0;
  bbox.dimensions.x = 1.0;
  bbox.dimensions.y = 1.0;
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(120659, 1, 0)).pose,
      bbox, false);
    EXPECT_TRUE(id);
    EXPECT_EQ(id.value(), 120659);
  }
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34411, 1, 0)).pose,
      bbox, false);
    EXPECT_TRUE(id);
    EXPECT_EQ(id.value(), 34411);
  }
}

TEST(HdMapUtils, AlongLaneletPose)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  EXPECT_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0, 0), 30)
      .lanelet_id,
    34513);
  EXPECT_EQ(
    hdmap_utils
      .getAlongLaneletPose(
        traffic_simulator::helper::constructLaneletPose(34513, 0, 0),
        hdmap_utils.getLaneletLength(34513) + 10.0)
      .lanelet_id,
    34510);
  EXPECT_DOUBLE_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0, 0), 30.0)
      .s,
    30.0);
  EXPECT_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0, 0), 30.0)
      .lanelet_id,
    34513);

  EXPECT_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0, 0), -10.0)
      .lanelet_id,
    34684);
  EXPECT_DOUBLE_EQ(
    hdmap_utils
      .getAlongLaneletPose(traffic_simulator::helper::constructLaneletPose(34513, 0, 0), -10.0)
      .s,
    hdmap_utils.getLaneletLength(34684) - 10.0);
}

TEST(HdMapUtils, AlongLaneletPose_afterLast)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);

  const lanelet::Id lanelet_id = 206;
  const auto pose = traffic_simulator::helper::constructLaneletPose(lanelet_id, 15.0);

  EXPECT_THROW(hdmap_utils.getAlongLaneletPose(pose, 30.0), common::SemanticError);
}

TEST(HdMapUtils, AlongLaneletPose_beforeFirst)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);

  const lanelet::Id lanelet_id = 3002178;
  const auto pose = traffic_simulator::helper::constructLaneletPose(lanelet_id, 15.0);

  EXPECT_THROW(hdmap_utils.getAlongLaneletPose(pose, -30.0), common::SemanticError);
}

/**
 * @note Testcase for lanelet pose canonicalization when s < 0
 * Following lanelets: 34576 -> 34570 -> 34564
 * Canonicalized lanelet pose of (id=34564, s=-22) is suppose to be
 *                               (id=34576, s=-22 + length of 34570 + length of 34576)
 */
TEST(HdMapUtils, CanonicalizeNegative)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  double non_canonicalized_lanelet_s = -22;
  const auto non_canonicalized_lanelet_pose =
    traffic_simulator::helper::constructLaneletPose(34564, non_canonicalized_lanelet_s, 0);
  const auto canonicalized_lanelet_pose = std::get<std::optional<traffic_simulator::LaneletPose>>(
    hdmap_utils.canonicalizeLaneletPose(non_canonicalized_lanelet_pose));

  EXPECT_EQ(canonicalized_lanelet_pose.value().lanelet_id, 34576);
  EXPECT_EQ(
    canonicalized_lanelet_pose.value().s, non_canonicalized_lanelet_s +
                                            hdmap_utils.getLaneletLength(34570) +
                                            hdmap_utils.getLaneletLength(34576));
}

/**
 * @note Testcase for lanelet pose canonicalization when s > length of lanelet pose
 * Following lanelets: 34981 -> 34585 -> 34579
 * Canonicalized lanelet pose of (id=34981, s=30) is suppose to be
 *                               (id=34579, s=30 - length of 34585 - length of 34981)
 */
TEST(HdMapUtils, CanonicalizePositive)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  double non_canonicalized_lanelet_s = 30;
  const auto non_canonicalized_lanelet_pose =
    traffic_simulator::helper::constructLaneletPose(34981, non_canonicalized_lanelet_s, 0);
  const auto canonicalized_lanelet_pose = std::get<std::optional<traffic_simulator::LaneletPose>>(
    hdmap_utils.canonicalizeLaneletPose(non_canonicalized_lanelet_pose));

  EXPECT_EQ(canonicalized_lanelet_pose.value().lanelet_id, 34579);
  EXPECT_EQ(
    canonicalized_lanelet_pose.value().s, non_canonicalized_lanelet_s -
                                            hdmap_utils.getLaneletLength(34585) -
                                            hdmap_utils.getLaneletLength(34981));
}

/**
 * @note Testcase for lanelet pose canonicalization when s in
 * range [0,length_of_the_lanelet]
 * Canonicalized lanelet pose of (id=34981, s=2) is suppose to be the same
 */
TEST(HdMapUtils, Canonicalize)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  double non_canonicalized_lanelet_s = 2;
  const auto non_canonicalized_lanelet_pose =
    traffic_simulator::helper::constructLaneletPose(34981, non_canonicalized_lanelet_s, 0);
  const auto canonicalized_lanelet_pose = std::get<std::optional<traffic_simulator::LaneletPose>>(
    hdmap_utils.canonicalizeLaneletPose(non_canonicalized_lanelet_pose));

  EXPECT_EQ(canonicalized_lanelet_pose.value().lanelet_id, 34981);
  EXPECT_EQ(canonicalized_lanelet_pose.value().s, non_canonicalized_lanelet_s);
}

/**
 * @note Testcase for getAllCanonicalizedLaneletPoses() function when s < 0
 * Following lanelets: 34576 -> 34570 -> 34564
 *                     34981 -> 34636 -> 34564
 *                     34600 -> 34648 -> 34564
 * Canonicalized lanelet pose of (id=34564, s=-22) is suppose to be
 *                               (id=34575, s=-22 + length of 34570 + length of 34576)
 *                               (id=34981, s=-22 + length of 34636 + length of 34981)
 *                               (id=34600, s=-22 + length of 34648 + length of 34600)
 */
TEST(HdMapUtils, CanonicalizeAllNegative)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  double non_canonicalized_lanelet_s = -22;
  const auto non_canonicalized_lanelet_pose =
    traffic_simulator::helper::constructLaneletPose(34564, non_canonicalized_lanelet_s, 0);
  const auto canonicalized_lanelet_poses =
    hdmap_utils.getAllCanonicalizedLaneletPoses(non_canonicalized_lanelet_pose);

  EXPECT_EQ(canonicalized_lanelet_poses.size(), static_cast<long unsigned int>(3));
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
 * @note Testcase for getAllCanonicalizedLaneletPoses() function when s > length of lanelet pose
 * Following lanelets: 34981 -> 34585 -> 34579
 *                     34981 -> 34636 -> 34564
 *                     34981 -> 34651 -> 34630
 * Canonicalized lanelet pose of (id=34981, s=30) is suppose to be
 *                               (id=34579, s=30 - length of 34585 - length of 34981)
 *                               (id=34564, s=30 - length of 34636 - length of 34981)
 *                               (id=34630, s=30 - length of 34651 - length of 34981)
 */
TEST(HdMapUtils, CanonicalizeAllPositive)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  double non_canonicalized_lanelet_s = 30;
  const auto non_canonicalized_lanelet_pose =
    traffic_simulator::helper::constructLaneletPose(34981, non_canonicalized_lanelet_s, 0);
  const auto canonicalized_lanelet_poses =
    hdmap_utils.getAllCanonicalizedLaneletPoses(non_canonicalized_lanelet_pose);

  EXPECT_EQ(canonicalized_lanelet_poses.size(), static_cast<long unsigned int>(3));
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
 * Canonicalized lanelet pose of (id=34981, s=2) is suppose to be the same
 */
TEST(HdMapUtils, CanonicalizeAll)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  double non_canonicalized_lanelet_s = 2;
  const auto non_canonicalized_lanelet_pose =
    traffic_simulator::helper::constructLaneletPose(34981, non_canonicalized_lanelet_s, 0);
  const auto canonicalized_lanelet_poses =
    hdmap_utils.getAllCanonicalizedLaneletPoses(non_canonicalized_lanelet_pose);

  EXPECT_EQ(canonicalized_lanelet_poses.size(), static_cast<long unsigned int>(1));
  EXPECT_EQ(canonicalized_lanelet_poses[0].lanelet_id, 34981);
  EXPECT_EQ(canonicalized_lanelet_poses[0].s, non_canonicalized_lanelet_s);
}

TEST(HdMapUtils, Construct_invalid)
{
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "invalid_path";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.61836750154;
  origin.longitude = 139.78066608243;
  EXPECT_THROW(hdmap_utils::HdMapUtils hdmap_utils(path, origin), std::runtime_error);
}

TEST(HdMapUtils, filterLaneletIds_emptyIds)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  lanelet::Ids ids = {};
  const char * subtype = "crosswalk";
  auto filtered = hdmap_utils.filterLaneletIds(ids, subtype);

  EXPECT_TRUE(filtered.empty());
}

TEST(HdMapUtils, filterLaneletIds_invalidIds)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id_invalid_0 = 10000000;
  lanelet::Id id_invalid_1 = 10000001;
  lanelet::Id id_invalid_2 = 10000002;
  lanelet::Ids ids = {id_invalid_0, id_invalid_1, id_invalid_2};
  const char * subtype = "crosswalk";

  EXPECT_THROW(auto filtered = hdmap_utils.filterLaneletIds(ids, subtype), std::runtime_error);
}

TEST(HdMapUtils, filterLaneletIds_correct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id_crosswalk_0 = 34399;
  lanelet::Id id_crosswalk_1 = 34385;
  lanelet::Id id_road_0 = 34600;
  lanelet::Id id_road_1 = 34675;
  lanelet::Ids ids = {id_crosswalk_0, id_crosswalk_1, id_road_0, id_road_1};
  const char * subtype = "crosswalk";

  auto filtered = hdmap_utils.filterLaneletIds(ids, subtype);

  EXPECT_EQ(filtered.size(), 2);
  EXPECT_TRUE(std::find(filtered.begin(), filtered.end(), id_crosswalk_0) != filtered.end());
  EXPECT_TRUE(std::find(filtered.begin(), filtered.end(), id_crosswalk_1) != filtered.end());
}

TEST(HdMapUtils, filterLaneletIds_invalidSubtype)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id_crosswalk_0 = 34399;
  lanelet::Id id_crosswalk_1 = 34385;
  lanelet::Id id_road_0 = 34600;
  lanelet::Id id_road_1 = 34675;
  lanelet::Ids ids = {id_crosswalk_0, id_crosswalk_1, id_road_0, id_road_1};
  const char * subtype = "invalid_subtype";

  auto filtered = hdmap_utils.filterLaneletIds(ids, subtype);

  EXPECT_TRUE(filtered.empty());
}

TEST(HdMapUtils, getNearbyLaneletIds)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  auto point = makePoint(3807.34, 73817.95);
  const double distance_threshold = 10.0;
  size_t search_count = 100;
  auto lanelets = hdmap_utils.getNearbyLaneletIds(point, distance_threshold, search_count);

  lanelet::Ids nearbyLanelets = {34795, 120660, 34507, 34468, 120659, 34606};
  EXPECT_EQ(lanelets, nearbyLanelets);
}

TEST(HdMapUtils, getNearbyLaneletIds_unsuccessful)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  auto point = makePoint(3826.26, 73837.32);
  const double distance_threshold = 10.0;
  size_t search_count = 100;
  auto lanelets = hdmap_utils.getNearbyLaneletIds(point, distance_threshold, search_count);

  EXPECT_TRUE(lanelets.empty());
}

TEST(HdMapUtils, getNearbyLaneletIds_crosswalkIncluded)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  auto point = makePoint(3768.01, 73750.97);
  const double distance_threshold = 0.0;
  bool include_crosswalk = true;
  size_t search_count = 100;
  auto lanelets =
    hdmap_utils.getNearbyLaneletIds(point, distance_threshold, include_crosswalk, search_count);

  lanelet::Id id_crosswalk = 34399;
  lanelet::Id id_road = 34633;
  EXPECT_EQ(lanelets.size(), 2);
  EXPECT_TRUE(std::find(lanelets.begin(), lanelets.end(), id_crosswalk) != lanelets.end());
  EXPECT_TRUE(std::find(lanelets.begin(), lanelets.end(), id_road) != lanelets.end());
}

TEST(HdMapUtils, getNearbyLaneletIds_crosswalkNotIncluded)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  auto point = makePoint(3768.01, 73750.97);
  const double distance_threshold = 0.0;
  bool include_crosswalk = false;
  size_t search_count = 100;
  auto lanelets =
    hdmap_utils.getNearbyLaneletIds(point, distance_threshold, include_crosswalk, search_count);

  lanelet::Id id_road = 34633;
  EXPECT_EQ(lanelets.size(), 1);
  EXPECT_EQ(lanelets[0], id_road);
}

TEST(HdMapUtils, getNearbyLaneletIds_crosswalkUnsuccessful)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  auto point = makePoint(3826.26, 73837.32);
  const double distance_threshold = 10.0;
  bool include_crosswalk = true;
  size_t search_count = 100;
  auto lanelets =
    hdmap_utils.getNearbyLaneletIds(point, distance_threshold, include_crosswalk, search_count);

  EXPECT_TRUE(lanelets.empty());
}

TEST(HdMapUtils, getCollisionPointInLaneCoordinate_intersects)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  lanelet::Id id_crosswalk = 34399;
  lanelet::Id id_road = 34633;
  auto distance = hdmap_utils.getCollisionPointInLaneCoordinate(id_road, id_crosswalk);

  EXPECT_TRUE(distance.has_value());
  EXPECT_GT(distance.value(), 0.0);
}

TEST(HdMapUtils, getCollisionPointInLaneCoordinate_disjoint)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  lanelet::Id id_crosswalk = 34399;
  lanelet::Id id_road = 34579;
  auto distance = hdmap_utils.getCollisionPointInLaneCoordinate(id_road, id_crosswalk);

  EXPECT_FALSE(distance.has_value());
}

TEST(HdMapUtils, getCollisionPointInLaneCoordinate_invalidLanelet)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  lanelet::Id id_crosswalk = 34399;
  lanelet::Id id_road_invalid = 1000000;

  std::optional<double> distance;
  EXPECT_THROW(
    distance = hdmap_utils.getCollisionPointInLaneCoordinate(id_road_invalid, id_crosswalk),
    std::runtime_error);
}

TEST(HdMapUtils, getCollisionPointInLaneCoordinate_invalidCrosswalkLanelet)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  lanelet::Id id_crosswalk_invalid = 1000000;
  lanelet::Id id_road = 34600;

  std::optional<double> distance;
  EXPECT_THROW(
    distance = hdmap_utils.getCollisionPointInLaneCoordinate(id_road, id_crosswalk_invalid),
    std::runtime_error);
}

TEST(HdMapUtils, matchToLane_includeCrosswalk)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  auto bbox = makeSmallBoundingBox();
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34399, 1, 0)).pose,
      bbox, true);
    EXPECT_TRUE(id.has_value());
    EXPECT_EQ(id.value(), 34399);
  }
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34399, 1, 0)).pose,
      bbox, false);
    if (id.has_value()) {
      EXPECT_NE(id.value(), 34399);
    }
  }
}

TEST(HdMapUtils, matchToLane_noMatch)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  auto bbox = makeSmallBoundingBox();
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34392, 0, 0)).pose,
      bbox, false);
    EXPECT_FALSE(id.has_value());
  }
  {
    const auto id = hdmap_utils.matchToLane(
      hdmap_utils.toMapPose(traffic_simulator::helper::constructLaneletPose(34378, 0, 0)).pose,
      bbox, false);
    EXPECT_FALSE(id.has_value());
  }
}

TEST(HdMapUtils, toLaneletPose_correct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id id_road = 34600;
  const double yaw = M_PI + M_PI_2 / 3.0;
  const auto pose = makePose(makePoint(3790.0, 73757.0), makeQuaternionFromYaw(yaw));

  const auto lanelet_pose = hdmap_utils.toLaneletPose(pose, false);

  const auto reference_lanelet_pose =
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::LaneletPose>()
      .lanelet_id(id_road)
      .s(35.0)
      .offset(0.0)
      .rpy(geometry_msgs::msg::Vector3());

  EXPECT_TRUE(lanelet_pose.has_value());
  EXPECT_LANELET_POSE_NEAR(lanelet_pose.value(), reference_lanelet_pose, 0.1);
}

TEST(HdMapUtils, toLaneletPose_negativeOffset)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id id_road = 34600;
  const double yaw = M_PI + M_PI_2 / 3.0;

  const double offset_yaw = yaw - M_PI_2;
  const double offset = -0.5;
  const double offset_x = std::cos(offset_yaw) * std::abs(offset);
  const double offset_y = std::sin(offset_yaw) * std::abs(offset);

  const auto pose =
    makePose(makePoint(3790.0 + offset_x, 73757.0 + offset_y), makeQuaternionFromYaw(yaw));

  const auto lanelet_pose = hdmap_utils.toLaneletPose(pose, false);

  const auto reference_lanelet_pose =
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::LaneletPose>()
      .lanelet_id(id_road)
      .s(35.0)
      .offset(offset)
      .rpy(geometry_msgs::msg::Vector3());

  EXPECT_TRUE(lanelet_pose.has_value());
  EXPECT_LANELET_POSE_NEAR(lanelet_pose.value(), reference_lanelet_pose, 0.1);
}

TEST(HdMapUtils, toLaneletPose_reverse)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const double yaw = M_PI_2 + M_PI_2 / 3.0;
  const auto pose = makePose(makePoint(3790.0, 73757.0), makeQuaternionFromYaw(yaw));

  const auto lanelet_pose = hdmap_utils.toLaneletPose(pose, false);

  EXPECT_FALSE(lanelet_pose.has_value());
}

TEST(HdMapUtils, toLaneletPose_notOnLanelet)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const double yaw = M_PI + M_PI_2 / 3.0;
  const auto pose = makePose(makePoint(3790.0 + 5.0, 73757.0 - 5.0), makeQuaternionFromYaw(yaw));

  const auto lanelet_pose = hdmap_utils.toLaneletPose(pose, true);

  EXPECT_FALSE(lanelet_pose.has_value());
}

TEST(HdMapUtils, toLaneletPose_empty)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const double yaw = M_PI + M_PI_2 / 3.0;
  const auto pose = makePose(makePoint(3790.0, 73757.0), makeQuaternionFromYaw(yaw));

  const lanelet::Ids ids{};

  const auto lanelet_pose = hdmap_utils.toLaneletPose(pose, ids);

  EXPECT_FALSE(lanelet_pose.has_value());
}

TEST(HdMapUtils, toLaneletPose_boundingBoxMatchPrevious)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id id_road = 34600;
  const double yaw = M_PI + M_PI_2 / 3.0;
  const auto pose = makePose(makePoint(3774.9, 73749.2), makeQuaternionFromYaw(yaw));

  const auto bbox = makeBoundingBox();

  const auto lanelet_pose = hdmap_utils.toLaneletPose(pose, bbox, false, 0.5);

  const auto reference_lanelet_pose =
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::LaneletPose>()
      .lanelet_id(id_road)
      .s(52.0)
      .offset(0.0)
      .rpy(geometry_msgs::msg::Vector3());

  EXPECT_LANELET_POSE_NEAR(lanelet_pose.value(), reference_lanelet_pose, 0.1);
}

TEST(HdMapUtils, getSpeedLimit_correct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id_road_0 = 34600;
  lanelet::Id id_road_1 = 34675;
  lanelet::Ids ids = {id_road_0, id_road_1};

  double speed_limit = hdmap_utils.getSpeedLimit(ids);

  const double true_limit = 50.0 / 3.6;
  const double eps = 0.01;
  EXPECT_NEAR(speed_limit, true_limit, eps);
}

TEST(HdMapUtils, getSpeedLimit_crosswalk)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id_crosswalk_0 = 34399;
  lanelet::Id id_crosswalk_1 = 34385;
  lanelet::Id id_road_0 = 34600;
  lanelet::Id id_road_1 = 34675;
  lanelet::Ids ids = {id_crosswalk_0, id_crosswalk_1, id_road_0, id_road_1};

  double speed_limit = hdmap_utils.getSpeedLimit(ids);

  const double true_limit = 0.0 / 3.6;
  const double eps = 0.01;
  EXPECT_NEAR(speed_limit, true_limit, eps);
}

TEST(HdMapUtils, getSpeedLimit_empty)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Ids ids = {};

  EXPECT_THROW(static_cast<void>(hdmap_utils.getSpeedLimit(ids)), std::runtime_error);
}

TEST(HdMapUtils, getClosestLaneletId_near)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id = 120659;
  auto position = makePoint(3818.91, 73787.95);
  auto pose = makePose(position);
  const double distance_threshold = 1.0;
  const bool include_crosswalk = false;

  auto result = hdmap_utils.getClosestLaneletId(pose, distance_threshold, include_crosswalk);

  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), id);
}

TEST(HdMapUtils, getClosestLaneletId_away)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  auto position = makePoint(3775.82, 73743.29);
  auto pose = makePose(position);
  const double distance_threshold = 1.0;
  const bool include_crosswalk = false;

  auto result = hdmap_utils.getClosestLaneletId(pose, distance_threshold, include_crosswalk);

  EXPECT_FALSE(result.has_value());
}

TEST(HdMapUtils, getClosestLaneletId_crosswalkCloserButExcluded)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id_crosswalk = 34399;
  lanelet::Id id_road = 34639;
  auto position = makePoint(3774.73, 73744.38);
  auto pose = makePose(position);
  const double distance_threshold = 5.0;
  const bool include_crosswalk = false;

  auto result = hdmap_utils.getClosestLaneletId(pose, distance_threshold, include_crosswalk);

  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), id_road);
  EXPECT_NE(result.value(), id_crosswalk);
}

TEST(HdMapUtils, getClosestLaneletId_onlyCrosswalkNearButExcluded)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  lanelet::Id id_crosswalk = 34399;
  lanelet::Id id_road = 34639;
  auto position = makePoint(3774.73, 73744.38);
  auto pose = makePose(position);
  const double distance_threshold = 5.0;
  const bool include_crosswalk = true;

  auto result = hdmap_utils.getClosestLaneletId(pose, distance_threshold, include_crosswalk);

  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result.value(), id_crosswalk);
  EXPECT_NE(result.value(), id_road);
}

TEST(HdMapUtils, getClosestLaneletId_emptyMap)
{
  auto hdmap_utils = makeHdMapUtilsInstance(empty_map_path);
  auto position = makePoint(3.0, 5.0);
  auto pose = makePose(position);
  const double distance_threshold = 7.0;
  const bool include_crosswalk = false;

  auto result = hdmap_utils.getClosestLaneletId(pose, distance_threshold, include_crosswalk);

  EXPECT_FALSE(result.has_value());
}

TEST(HdMapUtils, getPreviousLaneletIds)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id curr_lanelet = 34468;
  const lanelet::Id prev_lanelet = 120660;

  const auto result_ids = hdmap_utils.getPreviousLaneletIds(curr_lanelet);
  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
  if (result_ids.size() == 1) {
    EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(prev_lanelet));
  }
}

TEST(HdMapUtils, getNextLaneletIds)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id curr_lanelet = 120660;
  const lanelet::Id next_lanelet = 34468;

  const auto result_ids = hdmap_utils.getNextLaneletIds(curr_lanelet);
  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
  if (result_ids.size() == 1) {
    EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(next_lanelet));
  }
}

TEST(HdMapUtils, getPreviousLaneletIds_RoadShoulder)
{
  std::string map_path(with_road_shoulder_map_path);
  auto hdmap_utils = makeHdMapUtilsInstance(map_path);
  const lanelet::Id curr_lanelet = 34768;
  const lanelet::Id prev_lanelet = 34696;

  const auto result_ids = hdmap_utils.getPreviousLaneletIds(curr_lanelet);
  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
  if (result_ids.size() == 1) {
    EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(prev_lanelet));
  }
}

TEST(HdMapUtils, getNextLaneletIds_RoadShoulder)
{
  std::string map_path(with_road_shoulder_map_path);
  auto hdmap_utils = makeHdMapUtilsInstance(map_path);
  const lanelet::Id curr_lanelet = 34696;
  const lanelet::Id next_lanelet = 34768;

  const auto result_ids = hdmap_utils.getNextLaneletIds(curr_lanelet);
  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
  if (result_ids.size() == 1) {
    EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(next_lanelet));
  }
}

TEST(HdMapUtils, getNextLaneletIds_multipleNext)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id curr_lanelet = 34468;
  const lanelet::Id next_lanelet_0 = 34438;
  const lanelet::Id next_lanelet_1 = 34465;
  lanelet::Ids next_lanelets = {next_lanelet_0, next_lanelet_1};

  auto result_ids = hdmap_utils.getNextLaneletIds(curr_lanelet);

  std::sort(next_lanelets.begin(), next_lanelets.end());
  std::sort(result_ids.begin(), result_ids.end());

  EXPECT_EQ(next_lanelets, result_ids);
}

TEST(HdMapUtils, getPreviousLaneletIds_multiplePrevious)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id curr_lanelet = 34462;
  const lanelet::Id prev_lanelet_0 = 34411;
  const lanelet::Id prev_lanelet_1 = 34465;
  lanelet::Ids prev_lanelets = {prev_lanelet_0, prev_lanelet_1};

  auto result_ids = hdmap_utils.getPreviousLaneletIds(curr_lanelet);

  std::sort(prev_lanelets.begin(), prev_lanelets.end());
  std::sort(result_ids.begin(), result_ids.end());

  EXPECT_EQ(prev_lanelets, result_ids);
}

TEST(HdMapUtils, getNextLaneletIds_direction)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id curr_lanelet = 34468;
  const lanelet::Id next_lanelet_left = 34438;
  const lanelet::Id next_lanelet_straight = 34465;

  {
    std::string turn_direction = "left";
    const auto result_ids = hdmap_utils.getNextLaneletIds(curr_lanelet, turn_direction);
    EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
    if (result_ids.size() == 1) {
      EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(next_lanelet_left));
    }
  }
  {
    std::string turn_direction = "straight";
    const auto result_ids = hdmap_utils.getNextLaneletIds(curr_lanelet, turn_direction);
    EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
    if (result_ids.size() == 1) {
      EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(next_lanelet_straight));
    }
  }
}

TEST(HdMapUtils, getPreviousLaneletIds_direction)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id curr_lanelet = 34462;
  const lanelet::Id prev_lanelet_left = 34411;
  const lanelet::Id prev_lanelet_straight = 34465;

  {
    std::string turn_direction = "left";
    const auto result_ids = hdmap_utils.getPreviousLaneletIds(curr_lanelet, turn_direction);
    EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
    if (result_ids.size() == 1) {
      EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(prev_lanelet_left));
    }
  }
  {
    std::string turn_direction = "straight";
    const auto result_ids = hdmap_utils.getPreviousLaneletIds(curr_lanelet, turn_direction);
    EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(1));
    if (result_ids.size() == 1) {
      EXPECT_EQ(result_ids[0], static_cast<lanelet::Id>(prev_lanelet_straight));
    }
  }
}

TEST(HdMapUtils, isInRoute_empty)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34468;
  lanelet::Ids route = {};

  EXPECT_FALSE(hdmap_utils.isInRoute(lanelet_id, route));
}

TEST(HdMapUtils, isInRoute_notOnRoute)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34468;
  const lanelet::Id route_part_0 = 34741;
  const lanelet::Id route_part_1 = 34850;
  const lanelet::Id route_part_2 = 34603;
  const lanelet::Id route_part_3 = 34777;
  lanelet::Ids route = {route_part_0, route_part_1, route_part_2, route_part_3};

  EXPECT_FALSE(hdmap_utils.isInRoute(lanelet_id, route));
}

TEST(HdMapUtils, isInRoute_onRoute)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id route_part_0 = 34741;
  const lanelet::Id route_part_1 = 34850;
  const lanelet::Id route_part_2 = 34603;
  const lanelet::Id route_part_3 = 34777;
  const lanelet::Id lanelet_id = route_part_1;
  lanelet::Ids route = {route_part_0, route_part_1, route_part_2, route_part_3};

  EXPECT_TRUE(hdmap_utils.isInRoute(lanelet_id, route));
}

TEST(HdMapUtils, isInLanelet_correct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const double s = 10;

  EXPECT_TRUE(hdmap_utils.isInLanelet(lanelet_id, s));
}

TEST(HdMapUtils, isInLanelet_after)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const double s = hdmap_utils.getLaneletLength(lanelet_id) + 5.0;

  EXPECT_FALSE(hdmap_utils.isInLanelet(lanelet_id, s));
}

TEST(HdMapUtils, isInLanelet_before)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const double s = -5;

  EXPECT_FALSE(hdmap_utils.isInLanelet(lanelet_id, s));
}

TEST(HdMapUtils, toMapPoints_correctPoints)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const std::vector<double> s{10.0, 20.0, 30.0};

  const auto points = hdmap_utils.toMapPoints(lanelet_id, s);

  EXPECT_EQ(points.size(), static_cast<std::size_t>(3));
  EXPECT_POINT_NEAR(points[0], makePoint(3768.7, 73696.2, 1.9), 0.1);
  EXPECT_POINT_NEAR(points[1], makePoint(3759.8, 73691.6, 2.1), 0.1);
  EXPECT_POINT_NEAR(points[2], makePoint(3750.9, 73687.1, 2.3), 0.1);
}

TEST(HdMapUtils, toMapPoints_negativeS)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const std::vector<double> s{-10.0, -20.0, -30.0};

  const auto points = hdmap_utils.toMapPoints(lanelet_id, s);

  EXPECT_EQ(points.size(), static_cast<std::size_t>(3));

  EXPECT_POINT_NEAR(points[0], makePoint(3786.5, 73705.3, 1.5), 0.1);
  EXPECT_POINT_NEAR(points[1], makePoint(3795.4, 73709.9, 1.3), 0.1);
  EXPECT_POINT_NEAR(points[2], makePoint(3804.3, 73714.5, 1.1), 0.1);
}

TEST(HdMapUtils, toMapPoints_sLargerThanLaneletLength)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;

  const auto lanelet_length = hdmap_utils.getLaneletLength(lanelet_id);
  const std::vector<double> s{lanelet_length + 10.0, lanelet_length + 20.0, lanelet_length + 30.0};

  const auto points = hdmap_utils.toMapPoints(lanelet_id, s);

  EXPECT_EQ(points.size(), static_cast<std::size_t>(3));
  EXPECT_POINT_NEAR(points[0], makePoint(3725.8, 73674.2, 3.0), 0.1);
  EXPECT_POINT_NEAR(points[1], makePoint(3716.9, 73669.6, 3.1), 0.1);
  EXPECT_POINT_NEAR(points[2], makePoint(3708.0, 73665.0, 3.3), 0.1);
}

TEST(HdMapUtils, toMapPoints_empty)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;

  std::vector<geometry_msgs::msg::Point> points;

  EXPECT_NO_THROW(points = hdmap_utils.toMapPoints(lanelet_id, {}));

  EXPECT_TRUE(points.empty());
}

TEST(HdMapUtils, toMapPose_onlyOffset)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const double s = 10.0;

  traffic_simulator_msgs::msg::LaneletPose pose;
  pose.lanelet_id = lanelet_id;
  pose.s = s;
  pose.offset = 0.5;

  const auto map_pose = hdmap_utils.toMapPose(pose);

  EXPECT_STREQ(map_pose.header.frame_id.c_str(), "map");
  EXPECT_POSE_NEAR(
    map_pose.pose, makePose(makePoint(3768.9, 73695.8, 1.9), makeQuaternionFromYaw(-2.667)), 0.1);
}

TEST(HdMapUtils, toMapPose_additionalRotation)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const double s = 10.0;

  traffic_simulator_msgs::msg::LaneletPose pose;
  pose.lanelet_id = lanelet_id;
  pose.s = s;
  pose.rpy.z = M_PI_4;

  const auto map_pose = hdmap_utils.toMapPose(pose);

  EXPECT_STREQ(map_pose.header.frame_id.c_str(), "map");
  EXPECT_POSE_NEAR(
    map_pose.pose,
    makePose(makePoint(3768.7, 73696.2, 1.9), makeQuaternionFromYaw(-2.667 + M_PI_4)), 0.1);
}

TEST(HdMapUtils, toMapPose_negativeS)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const double s = -10.0;

  traffic_simulator_msgs::msg::LaneletPose pose;
  pose.lanelet_id = lanelet_id;
  pose.s = s;

  geometry_msgs::msg::PoseStamped map_pose;
  EXPECT_NO_THROW(map_pose = hdmap_utils.toMapPose(pose));

  EXPECT_STREQ(map_pose.header.frame_id.c_str(), "map");
  EXPECT_POSE_NEAR(
    map_pose.pose, makePose(makePoint(3784.0, 73707.8, 1.5), makeQuaternionFromYaw(-1.595)), 0.1);
}

TEST(HdMapUtils, toMapPose_sLargerThanLaneletLength)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34696;
  const double s = hdmap_utils.getLaneletLength(lanelet_id) + 10.0;

  traffic_simulator_msgs::msg::LaneletPose pose;
  pose.lanelet_id = lanelet_id;
  pose.s = s;

  geometry_msgs::msg::PoseStamped map_pose;
  EXPECT_NO_THROW(map_pose = hdmap_utils.toMapPose(pose));

  EXPECT_STREQ(map_pose.header.frame_id.c_str(), "map");
  EXPECT_POSE_NEAR(
    map_pose.pose, makePose(makePoint(3724.9, 73678.1, 2.7), makeQuaternionFromYaw(2.828)), 0.1);
}

TEST(HdMapUtils, getLaneChangeableLaneletId_straight)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);
  const lanelet::Id start_lanelet = 199;
  const lanelet::Id end_lanelet = start_lanelet;
  const auto direction = traffic_simulator::lane_change::Direction::STRAIGHT;

  const auto result_lanelet = hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(end_lanelet, result_lanelet);
}

TEST(HdMapUtils, getLaneChangeableLaneletId_leftNoChangable)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);
  const lanelet::Id start_lanelet = 199;
  const auto direction = traffic_simulator::lane_change::Direction::LEFT;

  const auto result_lanelet = hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction);

  EXPECT_FALSE(result_lanelet.has_value());
}

TEST(HdMapUtils, getLaneChangeableLaneletId_leftChangable)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);
  const lanelet::Id start_lanelet = 200;
  const lanelet::Id end_lanelet = 199;
  const auto direction = traffic_simulator::lane_change::Direction::LEFT;

  const auto result_lanelet = hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), end_lanelet);
}

TEST(HdMapUtils, getLaneChangeableLaneletId_rightNoChangable)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);
  const lanelet::Id start_lanelet = 202;
  const auto direction = traffic_simulator::lane_change::Direction::RIGHT;

  const auto result_lanelet = hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction);

  EXPECT_FALSE(result_lanelet.has_value());
}

TEST(HdMapUtils, getLaneChangeableLaneletId_rightChangable)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);
  const lanelet::Id start_lanelet = 200;
  const lanelet::Id end_lanelet = 201;
  const auto direction = traffic_simulator::lane_change::Direction::RIGHT;

  const auto result_lanelet = hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), end_lanelet);
}

TEST(HdMapUtils, getLaneChangeableLaneletId_shift2LeftPossible)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);
  const lanelet::Id start_lanelet = 201;
  const lanelet::Id end_lanelet = 199;
  const auto direction = traffic_simulator::lane_change::Direction::LEFT;
  const uint8_t shift = 2;

  const auto result_lanelet =
    hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction, shift);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), end_lanelet);
}

TEST(HdMapUtils, getLaneChangeableLaneletId_shift2LeftNotPossible)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);
  const lanelet::Id start_lanelet = 200;
  const auto direction = traffic_simulator::lane_change::Direction::LEFT;
  const uint8_t shift = 2;

  const auto result_lanelet =
    hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction, shift);

  EXPECT_FALSE(result_lanelet.has_value());
}

TEST(HdMapUtils, getLaneChangeableLaneletId_shift2RightPossible)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);
  const lanelet::Id start_lanelet = 200;
  const lanelet::Id end_lanelet = 202;
  const auto direction = traffic_simulator::lane_change::Direction::RIGHT;
  const uint8_t shift = 2;

  const auto result_lanelet =
    hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction, shift);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), end_lanelet);
}

TEST(HdMapUtils, getLaneChangeableLaneletId_shift2RightNotPossible)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);
  const lanelet::Id start_lanelet = 201;
  const auto direction = traffic_simulator::lane_change::Direction::RIGHT;
  const uint8_t shift = 2;

  const auto result_lanelet =
    hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction, shift);

  EXPECT_FALSE(result_lanelet.has_value());
}

TEST(HdMapUtils, getLaneChangeableLaneletId_shift0)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);
  const lanelet::Id start_lanelet = 201;
  const lanelet::Id end_lanelet = 201;
  const auto direction = traffic_simulator::lane_change::Direction::RIGHT;
  const uint8_t shift = 0;

  const auto result_lanelet =
    hdmap_utils.getLaneChangeableLaneletId(start_lanelet, direction, shift);

  EXPECT_TRUE(result_lanelet.has_value());
  EXPECT_EQ(result_lanelet.value(), end_lanelet);
}

TEST(HdMapUtils, getPreviousLanelets_straightBefore)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);
  const lanelet::Id lanelet_id = 202;

  auto result_previous = hdmap_utils.getPreviousLanelets(lanelet_id, 50.0);

  lanelet::Ids actual_previous{202, 3002185, 3002181};

  EXPECT_EQ(result_previous, actual_previous);
}

TEST(HdMapUtils, getPreviousLanelets_curveBefore)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id lanelet_id = 34600;

  auto result_previous = hdmap_utils.getPreviousLanelets(lanelet_id, 100.0);

  lanelet::Ids actual_previous{34600, 34783, 34606, 34795, 34507};

  EXPECT_EQ(result_previous, actual_previous);
}

TEST(HdMapUtils, getPreviousLanelets_notEnoughLaneletsBefore)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);
  const lanelet::Id lanelet_id = 202;

  auto result_previous = hdmap_utils.getPreviousLanelets(lanelet_id, 200.0);

  lanelet::Ids actual_previous{202, 3002185, 3002181, 3002178};

  EXPECT_EQ(result_previous, actual_previous);
}

TEST(HdMapUtils, getTrafficLightIds_correct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Ids traffic_lights = {34802, 34836};

  auto result_traffic_lights = hdmap_utils.getTrafficLightIds();

  std::sort(result_traffic_lights.begin(), result_traffic_lights.end());
  EXPECT_EQ(result_traffic_lights, traffic_lights);
}

TEST(HdMapUtils, getTrafficLightIds_noTrafficLight)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);

  auto result_traffic_lights = hdmap_utils.getTrafficLightIds();

  EXPECT_EQ(result_traffic_lights.size(), static_cast<size_t>(0));
}

TEST(HdMapUtils, getTrafficLightBulbPosition_correct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id light_id = 34802;
  const double epsilon = 0.1;

  {
    const std::string color = "green";
    const auto actual_bulb_position = makePoint(3761.05, 73755.30, 5.35);

    const auto return_bulb_position = hdmap_utils.getTrafficLightBulbPosition(light_id, color);

    EXPECT_TRUE(return_bulb_position.has_value());
    EXPECT_POINT_NEAR(return_bulb_position.value(), actual_bulb_position, epsilon);
  }

  {
    const std::string color = "yellow";
    const auto actual_bulb_position = makePoint(3760.60, 73755.07, 5.35);

    const auto return_bulb_position = hdmap_utils.getTrafficLightBulbPosition(light_id, color);

    EXPECT_TRUE(return_bulb_position.has_value());
    EXPECT_POINT_NEAR(return_bulb_position.value(), actual_bulb_position, epsilon);
  }

  {
    const std::string color = "red";
    const auto actual_bulb_position = makePoint(3760.16, 73754.87, 5.35);

    const auto return_bulb_position = hdmap_utils.getTrafficLightBulbPosition(light_id, color);

    EXPECT_TRUE(return_bulb_position.has_value());
    EXPECT_POINT_NEAR(return_bulb_position.value(), actual_bulb_position, epsilon);
  }

  {
    const std::string color = "pink";

    const auto return_bulb_position = hdmap_utils.getTrafficLightBulbPosition(light_id, color);

    EXPECT_FALSE(return_bulb_position.has_value());
  }
}

TEST(HdMapUtils, getTrafficLightBulbPosition_invalidTrafficLight)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id light_id = 1000003;

  {
    const std::string color = "red";

    const auto return_bulb_position = hdmap_utils.getTrafficLightBulbPosition(light_id, color);

    EXPECT_FALSE(return_bulb_position.has_value());
  }
}

TEST(HdMapUtils, getConflictingLaneIds_conflicting)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id id = 34510;
  lanelet::Ids actual_ids = {34495, 34498};

  auto result_ids = hdmap_utils.getConflictingLaneIds({id});

  std::sort(actual_ids.begin(), actual_ids.end());
  std::sort(result_ids.begin(), result_ids.end());
  EXPECT_EQ(actual_ids, result_ids);
}

TEST(HdMapUtils, getConflictingLaneIds_notConflicting)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id id = 34513;

  auto result_ids = hdmap_utils.getConflictingLaneIds({id});

  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(0));
}

TEST(HdMapUtils, getConflictingLaneIds_empty)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  auto result_ids = hdmap_utils.getConflictingLaneIds({});

  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(0));
}

TEST(HdMapUtils, getConflictingCrosswalkIds_conflicting)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id id = 34633;
  lanelet::Ids actual_ids = {34399, 34385};

  auto result_ids = hdmap_utils.getConflictingCrosswalkIds({id});

  std::sort(actual_ids.begin(), actual_ids.end());
  std::sort(result_ids.begin(), result_ids.end());
  EXPECT_EQ(actual_ids, result_ids);
}

TEST(HdMapUtils, getConflictingCrosswalkIds_notConflictingWithCrosswalk)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id id = 34510;

  auto result_ids = hdmap_utils.getConflictingCrosswalkIds({id});

  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(0));
}

TEST(HdMapUtils, getConflictingCrosswalkIds_notConflicting)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id id = 34513;

  auto result_ids = hdmap_utils.getConflictingCrosswalkIds({id});

  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(0));
}

TEST(HdMapUtils, getConflictingCrosswalkIds_empty)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  auto result_ids = hdmap_utils.getConflictingCrosswalkIds({});

  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(0));
}

TEST(HdMapUtils, clipTrajectoryFromLaneletIds_correct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id id = 34600;
  const double s = 40.0;
  const double forward_distance = 10.0;
  const lanelet::Ids route{id, 34594, 34621};

  const auto result_trajectory =
    hdmap_utils.clipTrajectoryFromLaneletIds(id, s, route, forward_distance);

  constexpr double epsilon = 0.1;

  const std::vector<geometry_msgs::msg::Point> actual_trajectory{
    makePoint(3785.5, 73754.7, -0.5), makePoint(3784.6, 73754.2, -0.5),
    makePoint(3783.7, 73753.8, -0.5), makePoint(3782.9, 73753.3, -0.5),
    makePoint(3782.0, 73752.9, -0.5), makePoint(3781.1, 73752.4, -0.4),
    makePoint(3780.2, 73751.9, -0.4), makePoint(3779.3, 73751.5, -0.4),
    makePoint(3778.4, 73751.0, -0.4), makePoint(3777.5, 73750.6, -0.4)};

  EXPECT_EQ(result_trajectory.size(), actual_trajectory.size());
  for (std::size_t i = 0; i < actual_trajectory.size(); ++i) {
    EXPECT_POINT_NEAR_STREAM(
      result_trajectory[i], actual_trajectory[i], epsilon, "In this test i = " << i);
  }
}

TEST(HdMapUtils, clipTrajectoryFromLaneletIds_startNotOnTrajectory)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id id = 34606;
  const double s = 40.0;
  const double forward_distance = 10.0;
  const lanelet::Ids route{34600, 34594, 34621};

  const auto result_trajectory =
    hdmap_utils.clipTrajectoryFromLaneletIds(id, s, route, forward_distance);

  EXPECT_EQ(result_trajectory.size(), static_cast<std::size_t>(0));
}

TEST(HdMapUtils, clipTrajectoryFromLaneletIds_emptyTrajectory)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id id = 34600;
  const double s = 40.0;
  const double forward_distance = 10.0;
  const lanelet::Ids route{};

  const auto result_trajectory =
    hdmap_utils.clipTrajectoryFromLaneletIds(id, s, route, forward_distance);

  EXPECT_EQ(result_trajectory.size(), static_cast<std::size_t>(0));
}

TEST(HdMapUtils, clipTrajectoryFromLaneletIds_smallForwardDistance)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id id = 34600;
  const double s = 40.0;
  const double forward_distance = 1.5;
  const lanelet::Ids route{id, 34594, 34621};

  const auto result_trajectory =
    hdmap_utils.clipTrajectoryFromLaneletIds(id, s, route, forward_distance);

  constexpr double epsilon = 0.1;

  EXPECT_EQ(result_trajectory.size(), static_cast<std::size_t>(2));
  EXPECT_POINT_NEAR(result_trajectory[0], makePoint(3785.5, 73754.7, -0.5), epsilon)
  EXPECT_POINT_NEAR(result_trajectory[1], makePoint(3784.6, 73754.2, -0.5), epsilon);
}

TEST(HdMapUtils, getFollowingLanelets_straightAfter)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id id = 120660;
  const double distance = 1.0;
  const bool include_self = true;

  const auto result_ids = hdmap_utils.getFollowingLanelets(id, distance, include_self);
  const lanelet::Ids actual_ids = {id, 34468};

  EXPECT_EQ(result_ids, actual_ids);
}

TEST(HdMapUtils, getFollowingLanelets_curveAfter)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id id = 34564;
  const double distance = 40.0;
  const bool include_self = true;

  const auto result_ids = hdmap_utils.getFollowingLanelets(id, distance, include_self);
  const lanelet::Ids actual_ids = {id, 34411, 34462};

  EXPECT_EQ(result_ids, actual_ids);
}

TEST(HdMapUtils, getFollowingLanelets_notEnoughLaneletsAfter)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);

  const lanelet::Id id = 199;
  const double distance = 1.0e100;
  const bool include_self = true;

  const auto result_ids = hdmap_utils.getFollowingLanelets(id, distance, include_self);
  const lanelet::Ids actual_ids = {id, 203};

  EXPECT_EQ(result_ids, actual_ids);
}

TEST(HdMapUtils, getFollowingLanelets_candidateTrajectory)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id id = 34564;
  const double distance = 40;
  const bool include_self = true;

  const lanelet::Ids route{id, 34495, 34507, 34795, 34606};

  const auto result_following = hdmap_utils.getFollowingLanelets(id, route, distance, include_self);

  const lanelet::Ids actual_following{id, 34495, 34507};

  EXPECT_EQ(result_following, actual_following);
}

TEST(HdMapUtils, getFollowingLanelets_candidateTrajectoryNotEnough)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id id = 34564;
  const double distance = 100;
  const bool include_self = true;

  const lanelet::Ids route{id, 34495, 34507};

  const auto result_following = hdmap_utils.getFollowingLanelets(id, route, distance, include_self);

  const lanelet::Ids actual_following{id, 34495, 34507, 34795, 34606};

  EXPECT_EQ(result_following, actual_following);
}

TEST(HdMapUtils, getFollowingLanelets_candidateTrajectoryEmpty)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id id = 120660;
  const double distance = 1.0e100;
  const bool include_self = true;

  const auto result_ids = hdmap_utils.getFollowingLanelets(id, {}, distance, include_self);

  EXPECT_EQ(result_ids.size(), static_cast<std::size_t>(0));
}

TEST(HdMapUtils, getFollowingLanelets_candidatesDoNotMatch)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id id = 120660;
  const lanelet::Ids candidates = {34981};
  const double distance = 1.0e100;
  const bool include_self = true;

  EXPECT_THROW(
    hdmap_utils.getFollowingLanelets(id, candidates, distance, include_self), common::Error);
}

TEST(HdMapUtils, canChangeLane_canChange)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);

  const lanelet::Id from_id = 199;
  const lanelet::Id to_id = 200;

  const bool verdict = hdmap_utils.canChangeLane(from_id, to_id);

  EXPECT_TRUE(verdict);
}

TEST(HdMapUtils, canChangeLane_canNotChange)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);

  const lanelet::Id from_id = 199;
  const lanelet::Id to_id = 201;

  const bool verdict = hdmap_utils.canChangeLane(from_id, to_id);

  EXPECT_FALSE(verdict);
}

TEST(HdMapUtils, canChangeLane_invalidLaneletId)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);

  const lanelet::Id from_id = 1000003;
  const lanelet::Id to_id = 1000033;

  EXPECT_THROW(hdmap_utils.canChangeLane(from_id, to_id), std::runtime_error);
}

TEST(HdMapUtils, getLateralDistance_sameLane)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);

  traffic_simulator_msgs::msg::LaneletPose from, to;
  from.lanelet_id = 3002185;
  from.s = 0.0;
  from.offset = 0.5;

  to.lanelet_id = 3002185;
  to.s = 10.0;
  to.offset = 0.2;

  const auto result = hdmap_utils.getLateralDistance(from, to);
  constexpr double epsilon = 1e-3;

  EXPECT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), to.offset - from.offset, epsilon);
}

TEST(HdMapUtils, getLateralDistance_parallelLanesCanNotChange)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);

  traffic_simulator_msgs::msg::LaneletPose from, to;
  from.lanelet_id = 3002185;
  from.s = 0.0;
  from.offset = 0.5;

  to.lanelet_id = 3002184;
  to.s = 10.0;
  to.offset = 0.2;

  const auto result = hdmap_utils.getLateralDistance(from, to, false);

  EXPECT_FALSE(result.has_value());
}

TEST(HdMapUtils, getLateralDistance_parallelLanesCanChange)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);

  traffic_simulator_msgs::msg::LaneletPose from, to;
  from.lanelet_id = 3002185;
  from.s = 0.0;
  from.offset = 0.5;

  to.lanelet_id = 3002184;
  to.s = 10.0;
  to.offset = 0.2;

  const double width1 = 2.80373, width2 = 3.03463;
  const double actual_distance = width1 / 2.0 + width2 / 2.0 + to.offset - from.offset;

  const auto result = hdmap_utils.getLateralDistance(from, to, true);
  constexpr double epsilon = 1e-3;

  EXPECT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), actual_distance, epsilon);
}

TEST(HdMapUtils, getLateralDistance_notConnected)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);

  traffic_simulator_msgs::msg::LaneletPose from, to;
  from.lanelet_id = 3002185;
  from.s = 0.0;
  from.offset = 0.5;

  to.lanelet_id = 3002166;
  to.s = 10.0;
  to.offset = 0.2;

  const auto result = hdmap_utils.getLateralDistance(from, to, true);

  EXPECT_FALSE(result.has_value());
}

TEST(HdMapUtils, getRoute_correct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id from_id = 34579;
  const lanelet::Id to_id = 34630;
  const bool allow_lane_change = true;

  const auto result_route = hdmap_utils.getRoute(from_id, to_id, allow_lane_change);

  const lanelet::Ids actual_route = {34579, 34774, 120659, 120660, 34468,
                                     34438, 34408, 34624,  34630};

  EXPECT_EQ(result_route, actual_route);
}

TEST(HdMapUtils, getRoute_correctCache)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id from_id = 34579;
  const lanelet::Id to_id = 34630;
  const bool allow_lane_change = true;

  const auto result_route_nohit = hdmap_utils.getRoute(from_id, to_id, allow_lane_change);
  const auto result_route_hit = hdmap_utils.getRoute(from_id, to_id, allow_lane_change);

  EXPECT_EQ(result_route_hit, result_route_nohit);
}

TEST(HdMapUtils, getRoute_impossibleRouting)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);

  const lanelet::Id from_id = 199;
  const lanelet::Id to_id = 196;
  const bool allow_lane_change = true;

  const auto result_route = hdmap_utils.getRoute(from_id, to_id, allow_lane_change);

  EXPECT_EQ(result_route.size(), static_cast<std::size_t>(0));
}

TEST(HdMapUtils, getRoute_circular)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id from_id = 120659;
  const lanelet::Id to_id = 120659;
  const bool allow_lane_change = false;

  const auto result_route = hdmap_utils.getRoute(from_id, to_id, allow_lane_change);

  EXPECT_EQ(result_route, lanelet::Ids{120659});
}

TEST(HdMapUtils, getCenterPoints_correct)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id id = 34594;

  const std::vector<geometry_msgs::msg::Point> actual_center_points{
    makePoint(3774.1, 73748.8, -0.3), makePoint(3772.5, 73748.0, -0.2),
    makePoint(3770.8, 73747.1, -0.2), makePoint(3769.2, 73746.3, -0.2),
    makePoint(3767.6, 73745.4, -0.2), makePoint(3766.0, 73744.6, -0.1),
    makePoint(3764.4, 73743.8, -0.1), makePoint(3762.7, 73742.9, -0.1),
    makePoint(3761.1, 73742.1, 0.0),  makePoint(3759.5, 73741.3, 0.0),
    makePoint(3757.9, 73740.4, 0.1),  makePoint(3756.3, 73739.6, 0.1)};

  constexpr double epsilon = 0.1;

  const auto result_center_points = hdmap_utils.getCenterPoints(id);

  EXPECT_EQ(result_center_points.size(), actual_center_points.size());
  for (std::size_t i = 0; i < result_center_points.size(); ++i) {
    EXPECT_POINT_NEAR_STREAM(
      result_center_points[i], actual_center_points[i], epsilon, "In this test i = " << i);
  }
}

TEST(HdMapUtils, getCenterPoints_correctCache)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Id id = 34594;

  const auto result_center_points = hdmap_utils.getCenterPoints(id);
  const auto result_center_points2 = hdmap_utils.getCenterPoints(id);

  EXPECT_EQ(result_center_points.size(), result_center_points2.size());
  for (std::size_t i = 0; i < result_center_points.size(); ++i) {
    EXPECT_POINT_EQ_STREAM(
      result_center_points[i], result_center_points2[i], "In this test i = " << i);
  }
}

TEST(HdMapUtils, getCenterPoints_correctVector)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Ids ids{34594, 34621};

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

  constexpr double epsilon = 0.1;

  const auto result_center_points = hdmap_utils.getCenterPoints(ids);

  EXPECT_EQ(result_center_points.size(), actual_center_points.size());
  for (std::size_t i = 0; i < result_center_points.size(); ++i) {
    EXPECT_POINT_NEAR_STREAM(
      result_center_points[i], actual_center_points[i], epsilon, "In this test i = " << i);
  }
}

TEST(HdMapUtils, getCenterPoints_empty)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  const lanelet::Ids ids{};

  const auto result_center_points = hdmap_utils.getCenterPoints(ids);

  EXPECT_EQ(result_center_points.size(), static_cast<std::size_t>(0));
}

TEST(HdMapUtils, isTrafficLight_trafficLight)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id id = 34836;

  const auto verdict = hdmap_utils.isTrafficLight(id);

  EXPECT_TRUE(verdict);
}

TEST(HdMapUtils, isTrafficLight_notTrafficLight)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id id = 120659;

  const auto verdict = hdmap_utils.isTrafficLight(id);

  EXPECT_FALSE(verdict);
}

TEST(HdMapUtils, isTrafficLight_invalidId)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id id = 1000003;

  const auto verdict = hdmap_utils.isTrafficLight(id);

  EXPECT_FALSE(verdict);
}

TEST(HdMapUtils, isTrafficLightRegulatoryElement_trafficLightRegulatoryElement)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id id = 34806;

  const auto verdict = hdmap_utils.isTrafficLightRegulatoryElement(id);

  EXPECT_TRUE(verdict);
}

TEST(HdMapUtils, isTrafficLightRegulatoryElement_noTrafficLightRegulatoryElement)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id id = 120659;

  const auto verdict = hdmap_utils.isTrafficLightRegulatoryElement(id);

  EXPECT_FALSE(verdict);
}

TEST(HdMapUtils, isTrafficLightRegulatoryElement_invalidId)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id id = 1000003;

  const auto verdict = hdmap_utils.isTrafficLightRegulatoryElement(id);

  EXPECT_FALSE(verdict);
}

TEST(HdMapUtils, getLaneletLength_simple)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id id = 34468;

  const double result_length = hdmap_utils.getLaneletLength(id);
  const double actual_length = 55.5;
  const double epsilon = 1.0;

  EXPECT_NEAR(result_length, actual_length, epsilon);
}

TEST(HdMapUtils, getLaneletLength_cache)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Id id = 34468;

  const double result_length_nohit = hdmap_utils.getLaneletLength(id);
  const double result_length_hit = hdmap_utils.getLaneletLength(id);

  EXPECT_EQ(result_length_nohit, result_length_hit);
}

TEST(HdMapUtils, getTrafficLightIdsOnPath_trafficLights)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Ids route = {34579, 34774, 120659, 120660, 34468, 34438, 34408, 34624, 34630};

  auto result_traffic_light_ids = hdmap_utils.getTrafficLightIdsOnPath(route);
  auto actual_traffic_light_ids = lanelet::Ids{34802, 34836};

  std::sort(result_traffic_light_ids.begin(), result_traffic_light_ids.end());
  std::sort(actual_traffic_light_ids.begin(), actual_traffic_light_ids.end());

  EXPECT_EQ(result_traffic_light_ids, actual_traffic_light_ids);
}

TEST(HdMapUtils, getTrafficLightIdsOnPath_noTrafficLights)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Ids route = {34579, 34774, 120659, 120660, 34468, 34438};

  auto result_traffic_light_ids = hdmap_utils.getTrafficLightIdsOnPath(route);

  EXPECT_EQ(result_traffic_light_ids.size(), static_cast<size_t>(0));
}

TEST(HdMapUtils, getTrafficLightIdsOnPath_empty)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  auto result_traffic_light_ids = hdmap_utils.getTrafficLightIdsOnPath({});

  EXPECT_EQ(result_traffic_light_ids.size(), static_cast<size_t>(0));
}

TEST(HdMapUtils, getLongitudinalDistance_sameLanelet)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  auto pose_from = hdmap_utils.toLaneletPose(
    makePose(makePoint(3812.65, 73810.13, -2.80), makeQuaternionFromYaw(90.0)), lanelet::Id{34606});
  auto pose_to = hdmap_utils.toLaneletPose(
    makePose(makePoint(3825.10, 73786.34, -1.82), makeQuaternionFromYaw(90.0)), lanelet::Id{34606});
  EXPECT_TRUE(pose_from.has_value());
  EXPECT_TRUE(pose_to.has_value());

  const bool allow_lane_change = false;

  const auto result_distance =
    hdmap_utils.getLongitudinalDistance(pose_from.value(), pose_to.value(), allow_lane_change);

  const double actual_distance = 27.0;
  const double epsilon = 1.0;

  EXPECT_TRUE(result_distance.has_value());
  EXPECT_NEAR(result_distance.value(), actual_distance, epsilon);
}

TEST(HdMapUtils, getLongitudinalDistance_differentLanelet)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  auto pose_from =
    hdmap_utils.toLaneletPose(makePose(makePoint(3801.19, 73812.70, -2.86)), lanelet::Id{120660});
  auto pose_to =
    hdmap_utils.toLaneletPose(makePose(makePoint(3724.70, 73773.00, -1.20)), lanelet::Id{34462});
  EXPECT_TRUE(pose_from.has_value());
  EXPECT_TRUE(pose_to.has_value());

  const bool allow_lane_change = false;

  const auto result_distance =
    hdmap_utils.getLongitudinalDistance(pose_from.value(), pose_to.value(), allow_lane_change);

  const double actual_distance = 86.0;
  const double epsilon = 1.0;

  EXPECT_TRUE(result_distance.has_value());
  EXPECT_NEAR(result_distance.value(), actual_distance, epsilon);
}

TEST(HdMapUtils, getLongitudinalDistance_sameLaneletBehind)
{
  auto hdmap_utils = makeHdMapUtilsInstance();
  auto pose_to = hdmap_utils.toLaneletPose(
    makePose(makePoint(3812.65, 73810.13, -2.80), makeQuaternionFromYaw(90.0)), lanelet::Id{34606});
  auto pose_from = hdmap_utils.toLaneletPose(
    makePose(makePoint(3825.10, 73786.34, -1.82), makeQuaternionFromYaw(90.0)), lanelet::Id{34606});
  EXPECT_TRUE(pose_from.has_value());
  EXPECT_TRUE(pose_to.has_value());

  const bool allow_lane_change = false;

  const auto result_distance =
    hdmap_utils.getLongitudinalDistance(pose_from.value(), pose_to.value(), allow_lane_change);

  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getLongitudinalDistance_differentLaneletNoRoute)
{
  auto hdmap_utils = makeHdMapUtilsInstance(four_track_highway_map_path);
  auto pose_to = hdmap_utils.toLaneletPose(
    makePose(makePoint(81590.79, 50067.66), makeQuaternionFromYaw(90.0)), lanelet::Id{3002185});
  auto pose_from = hdmap_utils.toLaneletPose(
    makePose(makePoint(81596.20, 50068.04), makeQuaternionFromYaw(90.0)), lanelet::Id{3002166});
  EXPECT_TRUE(pose_from.has_value());
  EXPECT_TRUE(pose_to.has_value());

  const bool allow_lane_change = false;

  const auto result_distance =
    hdmap_utils.getLongitudinalDistance(pose_from.value(), pose_to.value(), allow_lane_change);

  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getStopLineIdsOnPath_someStopLines)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Ids route = {34507, 34795, 34606, 34672, 34675, 34744, 34690};

  const auto result_stoplines = hdmap_utils.getStopLineIdsOnPath(route);
  const auto actual_stoplines = lanelet::Ids{120635};

  EXPECT_EQ(result_stoplines, actual_stoplines);
}

TEST(HdMapUtils, getStopLineIdsOnPath_noStopLines)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const lanelet::Ids route = {34507, 34795, 34606, 34672};

  const auto result_stoplines = hdmap_utils.getStopLineIdsOnPath(route);

  EXPECT_EQ(result_stoplines.size(), static_cast<size_t>(0));
}

TEST(HdMapUtils, getStopLineIdsOnPath_empty)
{
  auto hdmap_utils = makeHdMapUtilsInstance();

  const auto result_stoplines = hdmap_utils.getStopLineIdsOnPath({});

  EXPECT_EQ(result_stoplines.size(), static_cast<size_t>(0));
}

TEST(HdMapUtils, getTrafficLightStopLineIds_stopLine)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto result_stoplines = hdmap_utils.getTrafficLightStopLineIds(34802);
  auto actual_stoplines = lanelet::Ids{34805};

  EXPECT_EQ(result_stoplines, actual_stoplines);
}

TEST(HdMapUtils, getTrafficLightStopLineIds_severalStopLines)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto result_stoplines = hdmap_utils.getTrafficLightStopLineIds(34836);
  auto actual_stoplines = lanelet::Ids{120663, 34805};

  std::sort(result_stoplines.begin(), result_stoplines.end());
  std::sort(actual_stoplines.begin(), actual_stoplines.end());

  EXPECT_EQ(result_stoplines, actual_stoplines);
}

TEST(HdMapUtils, getTrafficLightStopLineIds_invalidTrafficLightId)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  const lanelet::Id invalid_id = 1000039;
  EXPECT_THROW(hdmap_utils.getTrafficLightStopLineIds(invalid_id), std::runtime_error);
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
  constexpr double epsilon = 1.0;
  EXPECT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].size(), b[i].size()) << "In this test i = " << i;
    for (std::size_t j = 0; j < a[i].size(); ++j) {
      EXPECT_POINT_NEAR_STREAM(
        a[i][j], b[i][j], epsilon, "In this test i = " << i << ", j = " << j);
    }
  }
}

TEST(HdMapUtils, getTrafficLightStopLinesPoints_stopLine)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto result_stoplines_points = hdmap_utils.getTrafficLightStopLinesPoints(34802);
  auto actual_stoplines_points = std::vector<std::vector<geometry_msgs::msg::Point>>{
    {makePoint(3762.0, 73756.0, -0.5), makePoint(3759.0, 73754.5, -0.5)}};

  EXPECT_EQ(result_stoplines_points.size(), actual_stoplines_points.size());

  sortStoplines(result_stoplines_points);
  sortStoplines(actual_stoplines_points);

  compareStoplines(actual_stoplines_points, result_stoplines_points);
}

TEST(HdMapUtils, getTrafficLightStopLinesPoints_severalStopLines)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto result_stoplines_points = hdmap_utils.getTrafficLightStopLinesPoints(34836);
  auto actual_stoplines_points = std::vector<std::vector<geometry_msgs::msg::Point>>{
    {makePoint(3762.0, 73756.0, -0.5), makePoint(3759.0, 73754.5, -0.5)},
    {makePoint(3768.5, 73737.0, -0.5), makePoint(3765.5, 73736.0, -0.5)}};

  EXPECT_EQ(result_stoplines_points.size(), actual_stoplines_points.size());

  sortStoplines(result_stoplines_points);
  sortStoplines(actual_stoplines_points);

  compareStoplines(actual_stoplines_points, result_stoplines_points);
}

TEST(HdMapUtils, getTrafficLightStopLinesPoints_invalidTrafficLightId)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  const lanelet::Id invalid_id = 1000039;
  EXPECT_THROW(hdmap_utils.getTrafficLightStopLinesPoints(invalid_id), std::runtime_error);
}

TEST(HdMapUtils, getStopLinePolygon_stopLine)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);
  const lanelet::Id linestring_id_with_stopline{120663};

  auto result_stoplines_points = hdmap_utils.getStopLinePolygon(linestring_id_with_stopline);
  auto actual_stoplines_points = std::vector<geometry_msgs::msg::Point>{
    makePoint(3768.5, 73737.5, -0.5), makePoint(3765.5, 73735.5, -0.5)};

  const double epsilon = 1.0;
  EXPECT_EQ(result_stoplines_points.size(), actual_stoplines_points.size());
  EXPECT_POINT_NEAR(result_stoplines_points.at(0), actual_stoplines_points.at(0), epsilon);
  EXPECT_POINT_NEAR(result_stoplines_points.at(1), actual_stoplines_points.at(1), epsilon);
}

TEST(HdMapUtils, getStopLinePolygon_noStopLine)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);
  const lanelet::Id linestring_id_no_stopline{34629};

  EXPECT_THROW(hdmap_utils.getStopLinePolygon(linestring_id_no_stopline), std::runtime_error);
}

TEST(HdMapUtils, getStopLinePolygon_invalidLaneletId)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  const lanelet::Id invalid_id = 1000039;
  EXPECT_THROW(hdmap_utils.getStopLinePolygon(invalid_id), std::runtime_error);
}

TEST(HdMapUtils, getDistanceToTrafficLightStopLine_trafficLightOnSpline)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3771.06, 73728.35);
  auto waypoint_1 = makePoint(3756.30, 73755.87);
  auto waypoint_2 = makePoint(3746.90, 73774.44);
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};
  auto spline = math::geometry::CatmullRomSpline(waypoints);

  auto traffic_light_id = lanelet::Id{34836};
  auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine(spline, traffic_light_id);
  EXPECT_TRUE(result_distance.has_value());

  auto stopline_midpoint = makePoint(3767.00, 73736.47);
  double approx_distance =
    std::hypot(waypoint_0.x - stopline_midpoint.x, waypoint_0.y - stopline_midpoint.y);
  double epsilon = 1.0;

  EXPECT_NEAR(approx_distance, result_distance.value(), epsilon);
}

TEST(HdMapUtils, getDistanceToTrafficLightStopLine_noTrafficLightOnSpline)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3807.63, 73715.99);
  auto waypoint_1 = makePoint(3785.76, 73707.70);
  auto waypoint_2 = makePoint(3773.19, 73723.27);
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};
  auto spline = math::geometry::CatmullRomSpline(waypoints);

  auto traffic_light_id = lanelet::Id{34836};
  auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine(spline, traffic_light_id);
  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getDistanceToTrafficLightStopLine_trafficLightOnWaypoints)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3771.06, 73728.35);
  auto waypoint_1 = makePoint(3756.30, 73755.87);
  auto waypoint_2 = makePoint(3746.90, 73774.44);
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};

  auto traffic_light_id = lanelet::Id{34836};
  auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine(waypoints, traffic_light_id);
  EXPECT_TRUE(result_distance.has_value());

  auto stopline_midpoint = makePoint(3767.00, 73736.47);
  double approx_distance =
    std::hypot(waypoint_0.x - stopline_midpoint.x, waypoint_0.y - stopline_midpoint.y);
  double epsilon = 1.0;

  EXPECT_NEAR(approx_distance, result_distance.value(), epsilon);
}

TEST(HdMapUtils, getDistanceToTrafficLightStopLine_noTrafficLightOnWaypoints)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3807.63, 73715.99);
  auto waypoint_1 = makePoint(3785.76, 73707.70);
  auto waypoint_2 = makePoint(3773.19, 73723.27);
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};

  auto traffic_light_id = lanelet::Id{34836};
  auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine(waypoints, traffic_light_id);
  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getDistanceToTrafficLightStopLine_emptyVector_waypoints)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoints = std::vector<geometry_msgs::msg::Point>{};

  auto traffic_light_id = lanelet::Id{34836};
  auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine(waypoints, traffic_light_id);
  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getDistanceToTrafficLightStopLine_routeWithTrafficLightsOnSpline)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3771.06, 73728.35);
  auto waypoint_1 = makePoint(3756.30, 73755.87);
  auto waypoint_2 = makePoint(3746.90, 73774.44);
  auto lanelets = lanelet::Ids{34576, 34570, 34564};
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};
  auto spline = math::geometry::CatmullRomSpline(waypoints);

  auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine(lanelets, spline);
  EXPECT_TRUE(result_distance.has_value());

  auto stopline_midpoint = makePoint(3767.00, 73736.47);
  double approx_distance =
    std::hypot(waypoint_0.x - stopline_midpoint.x, waypoint_0.y - stopline_midpoint.y);
  double epsilon = 1.0;

  EXPECT_NEAR(approx_distance, result_distance.value(), epsilon);
}

TEST(HdMapUtils, getDistanceToTrafficLightStopLine_routeWithNoTrafficLightsOnSplineCongruent)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3807.63, 73715.99);
  auto waypoint_1 = makePoint(3785.76, 73707.70);
  auto waypoint_2 = makePoint(3773.19, 73723.27);
  auto lanelets = lanelet::Ids{34690, 34759, 34576};
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};
  auto spline = math::geometry::CatmullRomSpline(waypoints);

  auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine(lanelets, spline);
  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getDistanceToTrafficLightStopLine_routeWithTrafficLightsNotOnSplineIncongruent)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3807.63, 73715.99);
  auto waypoint_1 = makePoint(3785.76, 73707.70);
  auto waypoint_2 = makePoint(3773.19, 73723.27);
  auto lanelets = lanelet::Ids{34576, 34570, 34564};
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};
  auto spline = math::geometry::CatmullRomSpline(waypoints);

  auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine(lanelets, spline);
  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getDistanceToTrafficLightStopLine_emptyVector_splineRoute)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3807.63, 73715.99);
  auto waypoint_1 = makePoint(3785.76, 73707.70);
  auto waypoint_2 = makePoint(3773.19, 73723.27);
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};
  auto spline = math::geometry::CatmullRomSpline(waypoints);

  auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine({}, spline);
  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getDistanceToTrafficLightStopLine_routeWithTrafficLightsOnWaypoints)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3771.06, 73728.35);
  auto waypoint_1 = makePoint(3756.30, 73755.87);
  auto waypoint_2 = makePoint(3746.90, 73774.44);
  auto lanelets = lanelet::Ids{34576, 34570, 34564};
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};

  auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine(lanelets, waypoints);
  EXPECT_TRUE(result_distance.has_value());

  auto stopline_midpoint = makePoint(3767.00, 73736.47);
  double approx_distance =
    std::hypot(waypoint_0.x - stopline_midpoint.x, waypoint_0.y - stopline_midpoint.y);
  double epsilon = 1.0;

  EXPECT_NEAR(approx_distance, result_distance.value(), epsilon);
}

TEST(HdMapUtils, getDistanceToTrafficLightStopLine_routeWithNoTrafficLightsOnWaypointsIncongruent)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3807.63, 73715.99);
  auto waypoint_1 = makePoint(3785.76, 73707.70);
  auto waypoint_2 = makePoint(3773.19, 73723.27);
  auto lanelets = lanelet::Ids{34690, 34759, 34576};
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};

  auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine(lanelets, waypoints);
  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getDistanceToTrafficLightStopLine_routeWithTrafficLightsNotOnWaypointsCongruent)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3807.63, 73715.99);
  auto waypoint_1 = makePoint(3785.76, 73707.70);
  auto waypoint_2 = makePoint(3773.19, 73723.27);
  auto lanelets = lanelet::Ids{34576, 34570, 34564};
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};

  auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine(lanelets, waypoints);
  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getDistanceToTrafficLightStopLine_emptyVector_waypointsRoute)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3807.63, 73715.99);
  auto waypoint_1 = makePoint(3785.76, 73707.70);
  auto waypoint_2 = makePoint(3773.19, 73723.27);
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};

  auto result_distance = hdmap_utils.getDistanceToTrafficLightStopLine({}, waypoints);
  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getDistanceToStopLine_stopLineOnSpline)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3821.86, 73777.20);
  auto waypoint_1 = makePoint(3837.28, 73762.67);
  auto waypoint_2 = makePoint(3846.10, 73741.38);
  auto lanelets = lanelet::Ids{34780, 34675, 34744};
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};
  auto spline = math::geometry::CatmullRomSpline(waypoints);

  auto result_distance = hdmap_utils.getDistanceToStopLine(lanelets, spline);
  EXPECT_TRUE(result_distance.has_value());

  auto stopline_midpoint = makePoint(3838.98, 73759.28);
  double approx_distance =
    std::hypot(waypoint_0.x - stopline_midpoint.x, waypoint_0.y - stopline_midpoint.y);
  double epsilon = 1.0;

  EXPECT_NEAR(approx_distance, result_distance.value(), epsilon);
}

TEST(HdMapUtils, getDistanceToStopLine_noStopLineOnSplineIncongruent)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3821.86, 73777.20);
  auto waypoint_1 = makePoint(3837.28, 73762.67);
  auto waypoint_2 = makePoint(3846.10, 73741.38);
  auto lanelets = lanelet::Ids{34576, 34570, 34564};
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};
  auto spline = math::geometry::CatmullRomSpline(waypoints);

  auto result_distance = hdmap_utils.getDistanceToStopLine(lanelets, spline);
  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getDistanceToStopLine_noStopLineOnSplineCongruent)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3807.63, 73715.99);
  auto waypoint_1 = makePoint(3785.76, 73707.70);
  auto waypoint_2 = makePoint(3773.19, 73723.27);
  auto lanelets = lanelet::Ids{34690, 34759, 34576};
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};
  auto spline = math::geometry::CatmullRomSpline(waypoints);

  auto result_distance = hdmap_utils.getDistanceToStopLine(lanelets, spline);
  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getDistanceToStopLine_emptyVector_spline)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3807.63, 73715.99);
  auto waypoint_1 = makePoint(3785.76, 73707.70);
  auto waypoint_2 = makePoint(3773.19, 73723.27);
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};
  auto spline = math::geometry::CatmullRomSpline(waypoints);

  auto result_distance = hdmap_utils.getDistanceToStopLine({}, spline);
  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getDistanceToStopLine_stopLineOnWaypoints)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3821.86, 73777.20);
  auto waypoint_1 = makePoint(3837.28, 73762.67);
  auto waypoint_2 = makePoint(3846.10, 73741.38);
  auto lanelets = lanelet::Ids{34780, 34675, 34744};
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};

  auto result_distance = hdmap_utils.getDistanceToStopLine(lanelets, waypoints);
  EXPECT_TRUE(result_distance.has_value());

  auto stopline_midpoint = makePoint(3838.98, 73759.28);
  double approx_distance =
    std::hypot(waypoint_0.x - stopline_midpoint.x, waypoint_0.y - stopline_midpoint.y);
  double epsilon = 1.0;

  EXPECT_NEAR(approx_distance, result_distance.value(), epsilon);
}

TEST(HdMapUtils, getDistanceToStopLine_noStopLineOnWaypointsIncongruent)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3821.86, 73777.20);
  auto waypoint_1 = makePoint(3837.28, 73762.67);
  auto waypoint_2 = makePoint(3846.10, 73741.38);
  auto lanelets = lanelet::Ids{34576, 34570, 34564};
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};

  auto result_distance = hdmap_utils.getDistanceToStopLine(lanelets, waypoints);
  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getDistanceToStopLine_noStopLineOnWaypointsCongruent)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3807.63, 73715.99);
  auto waypoint_1 = makePoint(3785.76, 73707.70);
  auto waypoint_2 = makePoint(3773.19, 73723.27);
  auto lanelets = lanelet::Ids{34690, 34759, 34576};
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};

  auto result_distance = hdmap_utils.getDistanceToStopLine(lanelets, waypoints);
  EXPECT_FALSE(result_distance.has_value());
}

TEST(HdMapUtils, getDistanceToStopLine_emptyVector_waypoints)
{
  auto hdmap_utils = makeHdMapUtilsInstance(crossroads_with_stoplines_map_path);

  auto waypoint_0 = makePoint(3807.63, 73715.99);
  auto waypoint_1 = makePoint(3785.76, 73707.70);
  auto waypoint_2 = makePoint(3773.19, 73723.27);
  auto waypoints = std::vector<geometry_msgs::msg::Point>{waypoint_0, waypoint_1, waypoint_2};

  auto result_distance = hdmap_utils.getDistanceToStopLine({}, waypoints);
  EXPECT_FALSE(result_distance.has_value());
}
