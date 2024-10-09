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

#include <traffic_simulator/utils/pose.hpp>

#include "../helper_functions.hpp"

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class PoseTest : public testing::Test
{
protected:
  PoseTest()
  : hdmap_utils(std::make_shared<hdmap_utils::HdMapUtils>(
      ament_index_cpp::get_package_share_directory("traffic_simulator") +
        "/map/four_track_highway/lanelet2_map.osm",
      geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
        .latitude(35.22312494055522)
        .longitude(138.8024583466017)
        .altitude(0.0)))
  {
  }

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils;
};

/**
 * @note Test created object values.
 */
TEST(pose, quietNaNPose)
{
  const auto pose = traffic_simulator::pose::quietNaNPose();

  EXPECT_TRUE(std::isnan(pose.position.x));
  EXPECT_TRUE(std::isnan(pose.position.y));
  EXPECT_TRUE(std::isnan(pose.position.z));

  EXPECT_DOUBLE_EQ(pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.w, 1.0);
}

/**
 * @note Test created object values.
 */
TEST(pose, quietNaNLaneletPose)
{
  const auto pose = traffic_simulator::pose::quietNaNLaneletPose();

  EXPECT_EQ(pose.lanelet_id, std::numeric_limits<std::int64_t>::max());
  EXPECT_TRUE(std::isnan(pose.s));
  EXPECT_TRUE(std::isnan(pose.offset));
  EXPECT_TRUE(std::isnan(pose.rpy.x));
  EXPECT_TRUE(std::isnan(pose.rpy.y));
  EXPECT_TRUE(std::isnan(pose.rpy.z));
}

/**
 * @note Test canonicalization with a default constructed LaneletPose.
 */
TEST_F(PoseTest, canonicalize_default)
{
  const auto pose =
    traffic_simulator::pose::canonicalize(traffic_simulator_msgs::msg::LaneletPose(), hdmap_utils);

  EXPECT_FALSE(pose.has_value());
}

/**
 * @note Test canonicalization with an invalid LaneletPose.
 */
TEST_F(PoseTest, canonicalize_invalid)
{
  EXPECT_THROW(
    traffic_simulator::pose::canonicalize(
      traffic_simulator::pose::quietNaNLaneletPose(), hdmap_utils),
    std::runtime_error);
}

/**
 * @note Test canonicalization with a valid constructed LaneletPose.
 */
TEST_F(PoseTest, canonicalize_valid)
{
  const auto pose = traffic_simulator::helper::constructLaneletPose(195, 0.0, 0.0);
  std::optional<traffic_simulator::CanonicalizedLaneletPose> canonicalized_pose;

  EXPECT_NO_THROW(canonicalized_pose = traffic_simulator::pose::canonicalize(pose, hdmap_utils));
  ASSERT_TRUE(canonicalized_pose.has_value());
  EXPECT_LANELET_POSE_EQ(
    static_cast<traffic_simulator::LaneletPose>(canonicalized_pose.value()), pose);
}

/**
 * @note Test conversion to geometry_msgs::msg::Pose from CanonicalizedLaneletPose.
 */
TEST_F(PoseTest, toMapPose_CanonicalizedLaneletPose)
{
  const traffic_simulator::lanelet_pose::CanonicalizedLaneletPose canonicalized_pose(
    traffic_simulator::helper::constructLaneletPose(195, 0.0, 0.0), hdmap_utils);

  const geometry_msgs::msg::Pose pose = makePose(
    makePoint(81585.1622, 50176.9202, 34.2595),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(-0.6397).w(0.7686));

  EXPECT_POSE_NEAR(traffic_simulator::pose::toMapPose(canonicalized_pose), pose, 0.01);
}

/**
 * @note Test conversion to geometry_msgs::msg::Pose from LaneletPose with HdMapUtils pointer.
 */
TEST_F(PoseTest, toMapPose_LaneletPose)
{
  const auto lanelet_pose = traffic_simulator::helper::constructLaneletPose(195, 0.0, 0.0);

  const geometry_msgs::msg::Pose pose = makePose(
    makePoint(81585.1622, 50176.9202, 34.2595),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(-0.6397).w(0.7686));

  EXPECT_POSE_NEAR(traffic_simulator::pose::toMapPose(lanelet_pose, hdmap_utils), pose, 0.01);
}

/**
 * @note Test function behavior with a pose that can be canonicalized.
 */
TEST_F(PoseTest, toCanonicalizedLaneletPose_noBoundingBox_noRoute_valid)
{
  const auto lanelet_pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 0.0, 0.0, hdmap_utils);

  const geometry_msgs::msg::Pose pose = static_cast<geometry_msgs::msg::Pose>(lanelet_pose);

  const auto canonicalized_pose =
    traffic_simulator::pose::toCanonicalizedLaneletPose(pose, true, hdmap_utils);

  ASSERT_TRUE(canonicalized_pose.has_value());
  EXPECT_POSE_NEAR(pose, static_cast<geometry_msgs::msg::Pose>(canonicalized_pose.value()), 0.01);
  EXPECT_LANELET_POSE_NEAR(
    static_cast<traffic_simulator::LaneletPose>(canonicalized_pose.value()),
    static_cast<traffic_simulator::LaneletPose>(lanelet_pose), 0.01);
}

/**
 * @note Test function behavior with a pose that can not be canonicalized.
 */
TEST_F(PoseTest, toCanonicalizedLaneletPose_noBoundingBox_noRoute_invalid)
{
  const geometry_msgs::msg::Pose pose = makePose(makePoint(0.0, 0.0, 0.0));

  EXPECT_EQ(
    traffic_simulator::pose::toCanonicalizedLaneletPose(pose, true, hdmap_utils), std::nullopt);
}

/**
 * @note Test function behavior with a pose that can be canonicalized.
 */
TEST_F(PoseTest, toCanonicalizedLaneletPose_BoundingBox_noRoute_valid)
{
  const auto lanelet_pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 0.0, 0.0, hdmap_utils);

  const geometry_msgs::msg::Pose pose = static_cast<geometry_msgs::msg::Pose>(lanelet_pose);

  const auto canonicalized_pose = traffic_simulator::pose::toCanonicalizedLaneletPose(
    pose, makeBoundingBox(), true, 1.0, hdmap_utils);

  ASSERT_TRUE(canonicalized_pose.has_value());
  EXPECT_POSE_NEAR(pose, static_cast<geometry_msgs::msg::Pose>(canonicalized_pose.value()), 0.01);
  EXPECT_LANELET_POSE_NEAR(
    static_cast<traffic_simulator::LaneletPose>(canonicalized_pose.value()),
    static_cast<traffic_simulator::LaneletPose>(lanelet_pose), 0.01);
}

/**
 * @note Test function behavior with a pose that can not be canonicalized, matching distance too large.
 */
TEST_F(PoseTest, toCanonicalizedLaneletPose_BoundingBox_noRoute_invalid)
{
  const auto lanelet_pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 10.0, 0.0, hdmap_utils);

  const geometry_msgs::msg::Pose pose = static_cast<geometry_msgs::msg::Pose>(lanelet_pose);

  EXPECT_EQ(
    traffic_simulator::pose::toCanonicalizedLaneletPose(
      pose, makeSmallBoundingBox(), true, 0.0, hdmap_utils),
    std::nullopt);
}

/**
 * @note Test function behavior with an empty unique_route_lanelets vector and a pose that can not be canonicalized.
 */
TEST_F(PoseTest, toCanonicalizedLaneletPose_BoundingBox_route_emptyInvalid)
{
  const geometry_msgs::msg::Pose pose = makePose(makePoint(0.0, 0.0, 0.0));

  EXPECT_EQ(
    traffic_simulator::pose::toCanonicalizedLaneletPose(
      pose, makeBoundingBox(), lanelet::Ids{}, true, 1.0, hdmap_utils),
    std::nullopt);
}

/**
 * @note Test function behavior with an empty unique_route_lanelets vector and a pose that can be canonicalized.
 */
TEST_F(PoseTest, toCanonicalizedLaneletPose_BoundingBox_route_emptyValid)
{
  const auto lanelet_pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 0.0, 0.0, hdmap_utils);

  const geometry_msgs::msg::Pose pose = static_cast<geometry_msgs::msg::Pose>(lanelet_pose);

  const auto canonicalized_pose = traffic_simulator::pose::toCanonicalizedLaneletPose(
    pose, makeBoundingBox(), lanelet::Ids{}, true, 1.0, hdmap_utils);

  ASSERT_TRUE(canonicalized_pose.has_value());
  EXPECT_POSE_NEAR(pose, static_cast<geometry_msgs::msg::Pose>(canonicalized_pose.value()), 0.01);
  EXPECT_LANELET_POSE_NEAR(
    static_cast<traffic_simulator::LaneletPose>(canonicalized_pose.value()),
    static_cast<traffic_simulator::LaneletPose>(lanelet_pose), 0.01);
}

/**
 * @note Test function behavior with a non empty unique_route_lanelets vector and a pose that can not be canonicalized.
 */
TEST_F(PoseTest, toCanonicalizedLaneletPose_BoundingBox_route_nonEmptyInvalid)
{
  const geometry_msgs::msg::Pose pose = makePose(makePoint(0.0, 0.0, 0.0));

  EXPECT_EQ(
    traffic_simulator::pose::toCanonicalizedLaneletPose(
      pose, makeBoundingBox(), {195}, true, 1.0, hdmap_utils),
    std::nullopt);
}

/**
 * @note Test function behavior with a non empty unique_route_lanelets vector and a pose that can be canonicalized.
 */
TEST_F(PoseTest, toCanonicalizedLaneletPose_BoundingBox_route_nonEmptyValid)
{
  const auto lanelet_pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 0.0, 0.0, hdmap_utils);

  const geometry_msgs::msg::Pose pose = static_cast<geometry_msgs::msg::Pose>(lanelet_pose);

  const auto canonicalized_pose = traffic_simulator::pose::toCanonicalizedLaneletPose(
    pose, makeBoundingBox(), lanelet::Ids{195}, true, 1.0, hdmap_utils);

  ASSERT_TRUE(canonicalized_pose.has_value());
  EXPECT_POSE_NEAR(pose, static_cast<geometry_msgs::msg::Pose>(canonicalized_pose.value()), 0.01);
  EXPECT_LANELET_POSE_NEAR(
    static_cast<traffic_simulator::LaneletPose>(canonicalized_pose.value()),
    static_cast<traffic_simulator::LaneletPose>(lanelet_pose), 0.01);
}

/**
 * @note Test calculation correctness with a non trivial example.
 */
TEST(pose, transformRelativePoseToGlobal)
{
  const geometry_msgs::msg::Pose pose_relative = makePose(makePoint(4.9969, -28.1026, 0.214));
  const geometry_msgs::msg::Pose pose_global = makePose(makePoint(81601.0571, 50099.0981, 34.595));
  const geometry_msgs::msg::Pose pose_rel_global =
    makePose(makePoint(81606.054, 50070.9955, 34.809));

  EXPECT_POSE_EQ(
    traffic_simulator::pose::transformRelativePoseToGlobal(pose_global, pose_relative),
    pose_rel_global);
}

/**
 * @note Test calculation correctness when only one pose is zeroed, the goal is to obtain a pose equal to the second argument.
 */
TEST(pose, relativePose_poses_zero)
{
  const auto from = makePose(makePoint(0.0, 0.0, 0.0));
  const auto to = makePose(makePoint(10.0, 51.0, 2.0));
  const auto relative = traffic_simulator::pose::relativePose(from, to);

  ASSERT_TRUE(relative.has_value());
  EXPECT_POSE_EQ(relative.value(), to);
}

/**
 * @note Test calculation correctness when both poses are zeroed.
 */
TEST(pose, relativePose_poses_zeros)
{
  const auto from = makePose(makePoint(0.0, 0.0, 0.0));
  const auto to = makePose(makePoint(0.0, 0.0, 0.0));
  const auto relative = traffic_simulator::pose::relativePose(from, to);

  ASSERT_TRUE(relative.has_value());
  EXPECT_POSE_EQ(relative.value(), to);
}

/**
 * @note Test calculation correctness with a non trivial example.
 */
TEST(pose, relativePose_poses_complex)
{
  const auto pose_relative = makePose(makePoint(4.9969, -28.1026, 0.214));
  const auto from = makePose(makePoint(81601.0571, 50099.0981, 34.595));
  const auto to = makePose(makePoint(81606.054, 50070.9955, 34.809));

  const auto relative = traffic_simulator::pose::relativePose(from, to);

  ASSERT_TRUE(relative.has_value());
  EXPECT_POSE_NEAR(pose_relative, relative.value(), 0.01);
}

/**
 * @note Test calculation correctness with the overload.
 */
TEST_F(PoseTest, relativePose_canonicalized_end_position)
{
  const auto pose_relative = makePose(
    makePoint(9.9999, -0.0009, 0.0126),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1));
  const auto from = makePose(
    makePoint(81585.1622, 50176.9202, 34.2595),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(-0.6397).w(0.7686));
  const auto to =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 10.0, 0.0, hdmap_utils);

  const auto relative = traffic_simulator::pose::relativePose(from, to);

  ASSERT_TRUE(relative.has_value());
  EXPECT_POSE_NEAR(pose_relative, relative.value(), 0.01);
}

/**
 * @note Test calculation correctness with the overload.
 */
TEST_F(PoseTest, relativePose_canonicalized_start_position)
{
  const auto pose_relative = makePose(
    makePoint(145244.7916, 786706.3326, 0.0127),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(0).w(1));
  const auto from =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 0.0, 0.0, hdmap_utils);
  const auto to = makePose(
    makePoint(881586.9767, 50167.0862, 34.2722),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0).y(0).z(-0.6397).w(0.7686));
  const auto relative = traffic_simulator::pose::relativePose(from, to);

  ASSERT_TRUE(relative.has_value());
  EXPECT_POSE_NEAR(pose_relative, relative.value(), 0.01);
}

/**
 * @note Test calculation correctness when bounding boxes are disjoint.
 */
TEST_F(PoseTest, boundingBoxRelativePose_disjoint)
{
  const auto pose_relative = makePose(makePoint(0.1108, -20.5955, 0.0));
  const auto from = makePose(makePoint(81600.3967, 50094.4298, 34.6759));
  const auto to = makePose(makePoint(81604.5076, 50071.8343, 40.8482));
  const auto bounding_box = makeBoundingBox();

  const auto relative =
    traffic_simulator::pose::boundingBoxRelativePose(from, bounding_box, to, bounding_box);

  ASSERT_TRUE(relative.has_value());
  EXPECT_POSE_NEAR(pose_relative, relative.value(), 0.01);
}

/**
 * @note Test calculation correctness when bounding boxes share an edge.
 */
TEST_F(PoseTest, boundingBoxRelativePose_commonEdge)
{
  const auto from = makePose(makePoint(81600, 50094, 34));
  const auto to = makePose(makePoint(81601, 50094, 34));
  const auto bounding_box = makeSmallBoundingBox();

  const auto relative =
    traffic_simulator::pose::boundingBoxRelativePose(from, bounding_box, to, bounding_box);

  EXPECT_FALSE(relative.has_value());
}

/**
 * @note Test calculation correctness when bounding boxes intersect.
 */
TEST_F(PoseTest, boundingBoxRelativePose_intersect)
{
  const auto from = makePose(makePoint(81600, 50094, 34));
  const auto to = makePose(makePoint(81600.5, 50094, 34));
  const auto bounding_box = makeSmallBoundingBox();

  const auto relative =
    traffic_simulator::pose::boundingBoxRelativePose(from, bounding_box, to, bounding_box);

  EXPECT_FALSE(relative.has_value());
}

/**
 * @note Test s-value calculation correctness when longitudinal distance between the poses can not be calculated.
 */
TEST_F(PoseTest, relativeLaneletPose_s_invalid)
{
  const auto from =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 0.0, 0.0, hdmap_utils);
  const auto to =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(196, 0.0, 0.0, hdmap_utils);

  const auto relative = traffic_simulator::pose::relativeLaneletPose(from, to, false, hdmap_utils);

  EXPECT_TRUE(std::isnan(relative.s));
}

/**
 * @note Test s-value calculation correctness when longitudinal distance between the poses can be calculated.
 */
TEST_F(PoseTest, relativeLaneletPose_s_valid)
{
  const auto from =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 0.0, 0.0, hdmap_utils);
  const auto to =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(3002163, 0.0, 0.0, hdmap_utils);

  const auto relative = traffic_simulator::pose::relativeLaneletPose(from, to, false, hdmap_utils);

  EXPECT_NEAR(relative.s, 107.74, 0.001);
}

/**
 * @note Test offset-value calculation correctness when lateral distance between the poses can not be calculated.
 */
TEST_F(PoseTest, relativeLaneletPose_offset_invalid)
{
  const auto from =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 0.0, 0.0, hdmap_utils);
  const auto to =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(196, 0.0, 0.0, hdmap_utils);

  const auto relative = traffic_simulator::pose::relativeLaneletPose(from, to, false, hdmap_utils);

  EXPECT_TRUE(std::isnan(relative.offset));
}

/**
 * @note Test offset-value calculation correctness when lateral distance between the poses can be calculated.
 */
TEST_F(PoseTest, relativeLaneletPose_offset_valid)
{
  const auto from =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 0.0, 0.0, hdmap_utils);
  const auto to =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(3002163, 0.0, 1.0, hdmap_utils);

  const auto relative = traffic_simulator::pose::relativeLaneletPose(from, to, true, hdmap_utils);

  EXPECT_EQ(relative.offset, 1.0);
}

/**
 * @note Test s-value calculation correctness when longitudinal distance between the poses can not be calculated.
 */
TEST_F(PoseTest, boundingBoxRelativeLaneletPose_s_invalid)
{
  const auto from =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 0.0, 0.0, hdmap_utils);
  const auto to =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(196, 0.0, 1.0, hdmap_utils);
  const auto bounding_box = makeBoundingBox();

  const auto relative = traffic_simulator::pose::boundingBoxRelativeLaneletPose(
    from, bounding_box, to, bounding_box, false, hdmap_utils);

  EXPECT_TRUE(std::isnan(relative.s));
}

/**
 * @note Test s-value calculation correctness when longitudinal distance between the poses can be calculated.
 */
TEST_F(PoseTest, boundingBoxRelativeLaneletPose_s_valid)
{
  const auto from =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 0.0, 0.0, hdmap_utils);
  const auto to =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(3002163, 0.0, 0.0, hdmap_utils);
  const auto bounding_box = makeBoundingBox();

  const auto relative = traffic_simulator::pose::boundingBoxRelativeLaneletPose(
    from, bounding_box, to, bounding_box, false, hdmap_utils);

  EXPECT_NEAR(relative.s, 103.74, 0.01);
}

/**
 * @note Test offset-value calculation correctness when lateral distance between the poses can not be calculated.
 */
TEST_F(PoseTest, boundingBoxRelativeLaneletPose_offset_invalid)
{
  const auto from =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 0.0, 0.0, hdmap_utils);
  const auto to =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(196, 0.0, 1.0, hdmap_utils);
  const auto bounding_box = makeBoundingBox();

  const auto relative = traffic_simulator::pose::boundingBoxRelativeLaneletPose(
    from, bounding_box, to, bounding_box, false, hdmap_utils);

  EXPECT_TRUE(std::isnan(relative.s));
}

/**
 * @note Test offset-value calculation correctness when lateral distance between the poses can be calculated.
 */
TEST_F(PoseTest, boundingBoxRelativeLaneletPose_offset_valid)
{
  const auto from =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 0.0, -1.0, hdmap_utils);
  const auto to =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(3002163, 0.0, 1.0, hdmap_utils);
  const auto bounding_box = makeBoundingBox();

  const auto relative = traffic_simulator::pose::boundingBoxRelativeLaneletPose(
    from, bounding_box, to, bounding_box, false, hdmap_utils);

  EXPECT_EQ(relative.offset, 0.0);
}

/**
 * @note Test calculation correctness when the pose lies within the lanelet.
 */
TEST_F(PoseTest, isInLanelet_inside)
{
  const auto pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 0.0, 0.0, hdmap_utils);

  EXPECT_TRUE(traffic_simulator::pose::isInLanelet(
    pose, 195, std::numeric_limits<double>::epsilon(), hdmap_utils));
}

/**
 * @note Test calculation correctness when the pose lies outside the front of the lanelet, distance greater than the tolerance.
 */
TEST_F(PoseTest, isInLanelet_outsideFrontFar)
{
  const auto pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(3002163, -10.0, 0.0, hdmap_utils);

  EXPECT_FALSE(traffic_simulator::pose::isInLanelet(pose, 3002163, 1.0, hdmap_utils));
}

/**
 * @note Test calculation correctness when the pose lies outside the front of the lanelet, distance smaller than the tolerance.
 */
TEST_F(PoseTest, isInLanelet_outsideFrontClose)
{
  const auto pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(3002163, -1.0, 0.0, hdmap_utils);

  EXPECT_TRUE(traffic_simulator::pose::isInLanelet(pose, 3002163, 2.0, hdmap_utils));
}

/**
 * @note Test calculation correctness when the pose lies outside the back of the lanelet, distance greater than the tolerance.
 */
TEST_F(PoseTest, isInLanelet_outsideBackFar)
{
  const auto pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 120.0, 0.0, hdmap_utils);

  EXPECT_FALSE(traffic_simulator::pose::isInLanelet(pose, 195, 2, hdmap_utils));
}

/**
 * @note Test calculation correctness when the pose lies outside the back of the lanelet, distance smaller than the tolerance.
 */
TEST_F(PoseTest, isInLanelet_outsideBackClose)
{
  const auto pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 110.0, 0.0, hdmap_utils);

  EXPECT_TRUE(traffic_simulator::pose::isInLanelet(pose, 195, 10.0, hdmap_utils));
}

/**
 * @note Test calculation correctness when there are no following lanelets and the pose lies within the lanelet.
 */
TEST_F(PoseTest, isAtEndOfLanelets_noFollowing_within)
{
  const auto pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(3002171, 31.0, 0.0, hdmap_utils);

  EXPECT_FALSE(traffic_simulator::pose::isAtEndOfLanelets(pose, hdmap_utils));
}

/**
 * @note Test calculation correctness when there is a single following lanelet and the pose lies within the lanelet.
 */
TEST_F(PoseTest, isAtEndOfLanelets_singleFollowing_within)
{
  const auto pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(3002167, 5.0, 0.0, hdmap_utils);

  EXPECT_FALSE(traffic_simulator::pose::isAtEndOfLanelets(pose, hdmap_utils));
}

/**
 * @note Test calculation correctness when there is a single following lanelet and the pose lies after the end of the lanelet.
 */
TEST_F(PoseTest, isAtEndOfLanelets_singleFollowing_outside)
{
  const auto pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(3002167, 20.0, 0.0, hdmap_utils);

  EXPECT_FALSE(traffic_simulator::pose::isAtEndOfLanelets(pose, hdmap_utils));
}

/**
 * @note Test calculation correctness when there are multiple following lanelets and the pose lies within the lanelet.
 */
TEST_F(PoseTest, isAtEndOfLanelets_multipleFollowing_within)
{
  const auto pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 5.0, 0.0, hdmap_utils);

  EXPECT_FALSE(traffic_simulator::pose::isAtEndOfLanelets(pose, hdmap_utils));
}

/**
 * @note Test calculation correctness when there are multiple following lanelets and the pose lies after the end of the lanelet.
 */
TEST_F(PoseTest, isAtEndOfLanelets_multipleFollowing_outside)
{
  const auto pose =
    traffic_simulator::helper::constructCanonicalizedLaneletPose(195, 120.0, 0.0, hdmap_utils);

  EXPECT_FALSE(traffic_simulator::pose::isAtEndOfLanelets(pose, hdmap_utils));
}

/**
 * @note Test function behavior when non existent lanelet::Id has been passed.
 */
TEST_F(PoseTest, laneletLength_invalid)
{
  EXPECT_THROW(
    traffic_simulator::pose::laneletLength(10000000000000000, hdmap_utils), std::runtime_error);
}

/**
 * @note Test calculation correctness when a correct lanelet::Id has been passed.
 */
TEST_F(PoseTest, laneletLength_valid)
{
  EXPECT_NEAR(traffic_simulator::pose::laneletLength(195, hdmap_utils), 107.74, 0.01);
}
