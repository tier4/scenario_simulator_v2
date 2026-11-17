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
 * Test lanelet matching correctness with a small bounding box (1, 1)
 * and a pose on a crosswalk lanelet and including the crosswalk.
 */
TEST_F(LaneletWrapperTest_StandardMap, matchToLane_includeCrosswalk)
{
  auto bbox = makeSmallBoundingBox();
  {
    const auto id =
      pose::matchToLane(pose::toMapPose(helper::constructLaneletPose(34399, 1)).pose, bbox, true);
    EXPECT_TRUE(id.has_value());
    EXPECT_EQ(id.value(), 34399);
  }
  {
    const auto id =
      pose::matchToLane(pose::toMapPose(helper::constructLaneletPose(34399, 1)).pose, bbox, false);
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
TEST_F(LaneletWrapperTest_StandardMap, matchToLane_noMatch)
{
  auto bbox = makeSmallBoundingBox();
  {
    const auto id =
      pose::matchToLane(pose::toMapPose(helper::constructLaneletPose(34392, 0)).pose, bbox, false);
    EXPECT_FALSE(id.has_value());
  }
  {
    const auto id =
      pose::matchToLane(pose::toMapPose(helper::constructLaneletPose(34378, 0)).pose, bbox, false);
    EXPECT_FALSE(id.has_value());
  }
}

/**
 * @note Test basic functionality.
 * Test along lanelet pose obtaining with a distance
 * along the lanelet less than the lanelet length - so the along pose is still the same lanelet.
*/
TEST_F(LaneletWrapperTest_StandardMap, AlongLaneletPose_insideDistance)
{
  EXPECT_DOUBLE_EQ(pose::alongLaneletPose(helper::constructLaneletPose(34513, 0), 30.0).s, 30.0);
  EXPECT_EQ(pose::alongLaneletPose(helper::constructLaneletPose(34513, 0), 30.0).lanelet_id, 34513);
}

/**
 * @note Test basic functionality.
 * Test along lanelet pose obtaining with a distance
 * along the lanelet more than the lanelet length - the goal is
 * to test the situation when the next lanelet is returned.
*/
TEST_F(LaneletWrapperTest_StandardMap, AlongLaneletPose_outsideDistance)
{
  EXPECT_EQ(pose::alongLaneletPose(helper::constructLaneletPose(34513, 0), 30).lanelet_id, 34513);
  EXPECT_EQ(
    pose::alongLaneletPose(
      helper::constructLaneletPose(34513, 0), lanelet_map::laneletLength(34513) + 10.0)
      .lanelet_id,
    34510);
}

/**
 * @note Test basic functionality.
 * Test along lanelet pose obtaining with a negative distance
 * along the lanelet and start from the beginning of one lanelet - the goal is to test
 * the situation when the previous lanelet is returned.
*/
TEST_F(LaneletWrapperTest_StandardMap, AlongLaneletPose_negativeDistance)
{
  EXPECT_EQ(
    pose::alongLaneletPose(helper::constructLaneletPose(34513, 0), -10.0).lanelet_id, 34684);
  EXPECT_DOUBLE_EQ(
    pose::alongLaneletPose(helper::constructLaneletPose(34513, 0), -10.0).s,
    lanelet_map::laneletLength(34684) - 10.0);
}

/**
 * @note Test function behavior when passed a sufficiently large along distance
 * and the last lanelet on the map as start - the goal is to test the situation
 * when desired pose is outside the lanelet map.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, AlongLaneletPose_afterLast)
{
  EXPECT_THROW(
    pose::alongLaneletPose(helper::constructLaneletPose(206, 15.0), 30.0), common::SemanticError);
}

/**
 * @note Test function behavior when passed a negative along distance and first
 * lanelet on the map as start - the goal is to test the situation
 * when desired pose is outside the lanelet map.
 */
TEST_F(LaneletWrapperTest_FourTrackHighwayMap, AlongLaneletPose_beforeFirst)
{
  EXPECT_THROW(
    pose::alongLaneletPose(helper::constructLaneletPose(3002178, 15.0), -30.0),
    common::SemanticError);
}

/**
 * @note Test lanelet pose obtaining correctness when s < 0.
 * Function should find correct lanelet id and canonicalize lanelet pose even when s < 0.
 * Following lanelets: 34576 -> 34570 -> 34564
 * Canonicalized lanelet pose of (id=34564, s=-22) is suppose to be
 *                               (id=34576, s=-22 + length of 34570 + length of 34576)
 */
TEST_F(LaneletWrapperTest_StandardMap, CanonicalizeNegative)
{
  double non_canonicalized_lanelet_s = -22.0;
  const auto canonicalized_lanelet_pose =
    std::get<std::optional<LaneletPose>>(pose::canonicalizeLaneletPose(
      helper::constructLaneletPose(34564, non_canonicalized_lanelet_s)));

  EXPECT_EQ(canonicalized_lanelet_pose.value().lanelet_id, 34576);
  EXPECT_EQ(
    canonicalized_lanelet_pose.value().s, non_canonicalized_lanelet_s +
                                            lanelet_map::laneletLength(34570) +
                                            lanelet_map::laneletLength(34576));
}

/**
 * @note Test lanelet pose obtaining correctness when s is larger than lanelet length.
 * Function should find correct lanelet id and canonicalize lanelet pose for s larger than lanelet length.
 * Following lanelets: 34981 -> 34585 -> 34579
 * Canonicalized lanelet pose of (id=34981, s=30) is suppose to be
 *                               (id=34579, s=30 - length of 34585 - length of 34981)
 */
TEST_F(LaneletWrapperTest_StandardMap, CanonicalizePositive)
{
  double non_canonicalized_lanelet_s = 30.0;
  const auto canonicalized_lanelet_pose =
    std::get<std::optional<LaneletPose>>(pose::canonicalizeLaneletPose(
      helper::constructLaneletPose(34981, non_canonicalized_lanelet_s)));

  EXPECT_EQ(canonicalized_lanelet_pose.value().lanelet_id, 34579);
  EXPECT_EQ(
    canonicalized_lanelet_pose.value().s, non_canonicalized_lanelet_s -
                                            lanelet_map::laneletLength(34585) -
                                            lanelet_map::laneletLength(34981));
}

/**
 * @note Testcase for lanelet pose canonicalization when s in
 * range [0,length_of_the_lanelet]
 * Canonicalized lanelet pose of (id=34981, s=2) is suppose to be the same.
 */
TEST_F(LaneletWrapperTest_StandardMap, Canonicalize)
{
  const double non_canonicalized_lanelet_s = 2.0;
  const auto canonicalized_lanelet_pose =
    std::get<std::optional<LaneletPose>>(pose::canonicalizeLaneletPose(
      helper::constructLaneletPose(34981, non_canonicalized_lanelet_s)));

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
TEST_F(LaneletWrapperTest_StandardMap, CanonicalizeAllNegative)
{
  const double non_canonicalized_lanelet_s = -22.0;
  const auto canonicalized_lanelet_poses =
    pose::alternativeLaneletPoses(helper::constructLaneletPose(34564, non_canonicalized_lanelet_s));

  EXPECT_EQ(canonicalized_lanelet_poses.size(), static_cast<std::size_t>(3));
  EXPECT_EQ(canonicalized_lanelet_poses[0].lanelet_id, 34576);
  EXPECT_EQ(
    canonicalized_lanelet_poses[0].s, non_canonicalized_lanelet_s +
                                        lanelet_map::laneletLength(34570) +
                                        lanelet_map::laneletLength(34576));
  EXPECT_EQ(canonicalized_lanelet_poses[1].lanelet_id, 34981);
  EXPECT_EQ(
    canonicalized_lanelet_poses[1].s, non_canonicalized_lanelet_s +
                                        lanelet_map::laneletLength(34636) +
                                        lanelet_map::laneletLength(34981));
  EXPECT_EQ(canonicalized_lanelet_poses[2].lanelet_id, 34600);
  EXPECT_EQ(
    canonicalized_lanelet_poses[2].s, non_canonicalized_lanelet_s +
                                        lanelet_map::laneletLength(34648) +
                                        lanelet_map::laneletLength(34600));
}

/**
 * @note Test basic functionality.
 * Test lanelet to map pose transform correctness
 * with a position on the lanelet and a small offset (e.g. 0.5) - test the specialization
 * taking a lanelet id, s and an offset as parameters.
 */
TEST_F(LaneletWrapperTest_StandardMap, toMapPose_onlyOffset)
{
  const auto map_pose = pose::toMapPose(helper::constructLaneletPose(34696, 10.0, 0.5));

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
TEST_F(LaneletWrapperTest_StandardMap, toMapPose_additionalRotation)
{
  const auto map_pose =
    pose::toMapPose(helper::constructLaneletPose(34696, 10.0, 0.0, 0.0, 0.0, M_PI_4));

  EXPECT_STREQ(map_pose.header.frame_id.c_str(), "map");
  EXPECT_POSE_NEAR(
    map_pose.pose,
    makePose(makePoint(3768.7, 73696.2, 1.9), makeQuaternionFromYaw(-2.667 + M_PI_4)), 0.1);
}

/**
 * @note Test function behavior when called with a negative s.
 */
TEST_F(LaneletWrapperTest_StandardMap, toMapPose_negativeS)
{
  geometry_msgs::msg::PoseStamped map_pose;
  EXPECT_NO_THROW(map_pose = pose::toMapPose(helper::constructLaneletPose(34696, -10.0)));

  EXPECT_STREQ(map_pose.header.frame_id.c_str(), "map");
  EXPECT_POSE_NEAR(
    map_pose.pose, makePose(makePoint(3784.0, 73707.8, 1.5), makeQuaternionFromYaw(-1.595)), 0.1);
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
TEST_F(LaneletWrapperTest_StandardMap, CanonicalizeAllPositive)
{
  const double non_canonicalized_lanelet_s = 30.0;
  const auto canonicalized_lanelet_poses =
    pose::alternativeLaneletPoses(helper::constructLaneletPose(34981, non_canonicalized_lanelet_s));

  EXPECT_EQ(canonicalized_lanelet_poses.size(), static_cast<std::size_t>(3));
  EXPECT_EQ(canonicalized_lanelet_poses[0].lanelet_id, 34579);
  EXPECT_EQ(
    canonicalized_lanelet_poses[0].s, non_canonicalized_lanelet_s -
                                        lanelet_map::laneletLength(34585) -
                                        lanelet_map::laneletLength(34981));
  EXPECT_EQ(canonicalized_lanelet_poses[1].lanelet_id, 34564);
  EXPECT_EQ(
    canonicalized_lanelet_poses[1].s, non_canonicalized_lanelet_s -
                                        lanelet_map::laneletLength(34636) -
                                        lanelet_map::laneletLength(34981));
  EXPECT_EQ(canonicalized_lanelet_poses[2].lanelet_id, 34630);
  EXPECT_EQ(
    canonicalized_lanelet_poses[2].s, non_canonicalized_lanelet_s -
                                        lanelet_map::laneletLength(34651) -
                                        lanelet_map::laneletLength(34981));
}

/**
 * @note Testcase for getAllCanonicalizedLaneletPoses() function when s in
 * range [0,length_of_the_lanelet]
 * Canonicalized lanelet pose of (id=34981, s=2) is supposed to be the same.
 */
TEST_F(LaneletWrapperTest_StandardMap, CanonicalizeAll)
{
  const double non_canonicalized_lanelet_s = 2.0;
  const auto canonicalized_lanelet_poses =
    pose::alternativeLaneletPoses(helper::constructLaneletPose(34981, non_canonicalized_lanelet_s));

  EXPECT_EQ(canonicalized_lanelet_poses.size(), static_cast<std::size_t>(1));
  EXPECT_EQ(canonicalized_lanelet_poses[0].lanelet_id, 34981);
  EXPECT_EQ(canonicalized_lanelet_poses[0].s, non_canonicalized_lanelet_s);
}

/**
 * @note Test basic functionality.
 * Test conversion to lanelet pose correctness with a point
 * positioned on a given lanelet with the given matching distance
 * - the goal is to test a regular usecase of correct conversion.
 */
TEST_F(LaneletWrapperTest_StandardMap, toLaneletPose_correct)
{
  const auto lanelet_pose = pose::toLaneletPose(
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
TEST_F(LaneletWrapperTest_StandardMap, toLaneletPose_negativeOffset)
{
  const double yaw = M_PI + M_PI_2 / 3.0;  // angle to make pose aligned with the lanelet

  const double offset_yaw = yaw - M_PI_2;  // offset pose
  const double offset = -0.5;

  const auto pose = makePose(
    makePoint(
      3790.0 + std::cos(offset_yaw) * std::abs(offset),
      73757.0 + std::sin(offset_yaw) * std::abs(offset)),
    makeQuaternionFromYaw(yaw));

  const auto lanelet_pose = pose::toLaneletPose(pose, false);

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
TEST_F(LaneletWrapperTest_StandardMap, toLaneletPose_reverse)
{
  EXPECT_FALSE(
    pose::toLaneletPose(
      makePose(makePoint(3790.0, 73757.0), makeQuaternionFromYaw(M_PI_2 + M_PI_2 / 3.0)), false)
      .has_value());  // angle to make pose reverse aligned with the lanelet
}

/**
 * @note Test function behavior when passed a pose that is away
 * from the given lanelet (over the matching distance).
 */
TEST_F(LaneletWrapperTest_StandardMap, toLaneletPose_notOnLanelet)
{
  EXPECT_FALSE(
    pose::toLaneletPose(
      makePose(makePoint(3790.0 + 5.0, 73757.0 - 5.0), makeQuaternionFromYaw(M_PI + M_PI_2 / 3.0)),
      true)
      .has_value());  // angle to make pose aligned with the lanelet
}

/**
 * @note test function behavior when passed an empty vector
 * of lanelet ids (for the vector specialization).
 */
TEST_F(LaneletWrapperTest_StandardMap, toLaneletPose_empty)
{
  EXPECT_FALSE(pose::toLaneletPose(
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
TEST_F(LaneletWrapperTest_StandardMap, toLaneletPose_boundingBoxMatchPrevious)
{
  EXPECT_LANELET_POSE_NEAR(
    pose::toLaneletPose(
      makePose(
        makePoint(3774.9, 73749.2),
        makeQuaternionFromYaw(M_PI + M_PI_2 / 3.0)),  // angle to make pose aligned with the lanelet
      makeBoundingBox(), false, 0.5)
      .value(),
    traffic_simulator_msgs::build<traffic_simulator_msgs::msg::LaneletPose>()
      .lanelet_id(34600)
      .s(52.0)
      .offset(0.0)
      .rpy(geometry_msgs::msg::Vector3()),
    0.1);
}
}  // namespace traffic_simulator::lanelet_wrapper::tests
