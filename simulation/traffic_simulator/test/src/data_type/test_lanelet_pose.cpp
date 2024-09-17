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

#include <traffic_simulator/data_type/lanelet_pose.hpp>

#include "../helper_functions.hpp"

using CanonicalizedLaneletPose = traffic_simulator::lanelet_pose::CanonicalizedLaneletPose;

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
class CanonicalizedLaneletPoseTest : public testing::Test
{
protected:
  CanonicalizedLaneletPoseTest() : hdmap_utils(makeHdMapUtilsSharedPointer()) {}

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils;
};

/**
 * @note Test constructor behavior with route_lanelets argument present when canonicalization function fails.
 */
TEST_F(CanonicalizedLaneletPoseTest, CanonicalizedLaneletPose_withRoute_invalid)
{
  EXPECT_THROW(
    CanonicalizedLaneletPose(
      traffic_simulator::helper::constructLaneletPose(100000000000, 0.0, 0.0), lanelet::Ids{},
      hdmap_utils),
    std::runtime_error);
}

/**
 * @note Test constructor behavior with route_lanelets argument present when canonicalization function succeeds
 */
TEST_F(CanonicalizedLaneletPoseTest, CanonicalizedLaneletPose_withRoute)
{
  std::shared_ptr<CanonicalizedLaneletPose> pose;
  EXPECT_NO_THROW(
    pose = std::make_shared<CanonicalizedLaneletPose>(
      traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0), lanelet::Ids{120659},
      hdmap_utils));
  EXPECT_LANELET_POSE_EQ(
    static_cast<traffic_simulator::LaneletPose>(*pose),
    traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0));
}

/**
 * @note Test constructor behavior with route_lanelets argument absent when canonicalization function fails
 */
TEST_F(CanonicalizedLaneletPoseTest, CanonicalizedLaneletPose_withoutRoute_invalid)
{
  EXPECT_THROW(
    CanonicalizedLaneletPose(
      traffic_simulator::helper::constructLaneletPose(100000000000, 0.0, 0.0), hdmap_utils),
    std::runtime_error);
}

/**
 * @note Test constructor behavior with route_lanelets argument absent when canonicalization function succeeds
 */
TEST_F(CanonicalizedLaneletPoseTest, CanonicalizedLaneletPose_withoutRoute)
{
  std::shared_ptr<CanonicalizedLaneletPose> pose;
  EXPECT_NO_THROW(
    pose = std::make_shared<CanonicalizedLaneletPose>(
      traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0), hdmap_utils));
  EXPECT_LANELET_POSE_EQ(
    static_cast<traffic_simulator::LaneletPose>(*pose),
    traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0));
}

/**
 * @note Test copy constructor behavior
 */
TEST_F(CanonicalizedLaneletPoseTest, CanonicalizedLaneletPose_copyConstructor)
{
  const CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0), hdmap_utils);
  const CanonicalizedLaneletPose pose_copy(pose);
  EXPECT_LANELET_POSE_EQ(
    static_cast<traffic_simulator::LaneletPose>(pose),
    static_cast<traffic_simulator::LaneletPose>(CanonicalizedLaneletPose(pose)));
}

/**
 * @note Test move constructor behavior
 */
TEST_F(CanonicalizedLaneletPoseTest, CanonicalizedLaneletPose_moveConstructor)
{
  const CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0), hdmap_utils);
  const CanonicalizedLaneletPose pose2(pose);
  const CanonicalizedLaneletPose pose_move = std::move(pose2);
  EXPECT_LANELET_POSE_EQ(
    static_cast<traffic_simulator::LaneletPose>(pose),
    static_cast<traffic_simulator::LaneletPose>(pose_move));
}

/**
 * @note Test copy assignment operator behavior
 */
TEST_F(CanonicalizedLaneletPoseTest, copyAssignment)
{
  const CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0), hdmap_utils);
  CanonicalizedLaneletPose pose_assign(
    traffic_simulator::helper::constructLaneletPose(34468, 0.0, 0.0), hdmap_utils);

  pose_assign = pose;

  EXPECT_LANELET_POSE_EQ(
    static_cast<traffic_simulator::LaneletPose>(pose),
    static_cast<traffic_simulator::LaneletPose>(pose_assign));
}

/**
 * @note Test conversion operator behavior using static_cast<LaneletPose>
 */
TEST_F(CanonicalizedLaneletPoseTest, conversionLaneletPose)
{
  const CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0), hdmap_utils);
  const traffic_simulator::LaneletPose pose_casted =
    static_cast<traffic_simulator::LaneletPose>(pose);

  EXPECT_EQ(pose_casted.lanelet_id, 120659);
  EXPECT_DOUBLE_EQ(pose_casted.s, 0.0);
  EXPECT_DOUBLE_EQ(pose_casted.offset, 0.0);
  EXPECT_VECTOR3_EQ(pose_casted.rpy, geometry_msgs::msg::Vector3());
}

/**
 * @note Test conversion operator behavior using static_cast<Pose>
 */
TEST_F(CanonicalizedLaneletPoseTest, conversionPose)
{
  const CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0), hdmap_utils);

  const geometry_msgs::msg::Pose pose1 =
    makePose(makePoint(3822.3815, 73784.9618, -1.761), makeQuaternionFromYaw(2.060578777273));

  EXPECT_POSE_NEAR(static_cast<geometry_msgs::msg::Pose>(pose), pose1, 0.01);
}

/**
 * @note Test function behavior when alternative poses are present
 */
TEST_F(CanonicalizedLaneletPoseTest, hasAlternativeLaneletPose_true)
{
  const CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(120659, -10.0, 0.0), hdmap_utils);

  EXPECT_TRUE(pose.hasAlternativeLaneletPose());
}

/**
 * @note Test function behavior when alternative poses are absent
 */
TEST_F(CanonicalizedLaneletPoseTest, hasAlternativeLaneletPose_false)
{
  const CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(120659, 10.0, 0.0), hdmap_utils);

  EXPECT_FALSE(pose.hasAlternativeLaneletPose());
}

/**
 * @note Test function behavior when there are no lanelet_poses
 */
TEST_F(CanonicalizedLaneletPoseTest, getAlternativeLaneletPoseBaseOnShortestRouteFrom_empty)
{
  const CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(120659, 20.0, 0.0), hdmap_utils);
  const auto from1 = traffic_simulator::helper::constructLaneletPose(34603, 10.0, 0.0);
  const auto from2 = traffic_simulator::helper::constructLaneletPose(34579, 10.0, 0.0);

  const auto result1 = pose.getAlternativeLaneletPoseBaseOnShortestRouteFrom(from1, hdmap_utils);
  const auto result2 = pose.getAlternativeLaneletPoseBaseOnShortestRouteFrom(from2, hdmap_utils);

  ASSERT_TRUE(result1.has_value());
  ASSERT_TRUE(result2.has_value());

  EXPECT_EQ(result1.value().lanelet_id, 120659);
  EXPECT_EQ(result2.value().lanelet_id, 120659);
}

/**
 * @note Test function behavior when there is only one lanelet_pose
 */
TEST_F(CanonicalizedLaneletPoseTest, getAlternativeLaneletPoseBaseOnShortestRouteFrom_single)
{
  CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(34666, -20.0, 0.0), hdmap_utils);
  const auto from1 = traffic_simulator::helper::constructLaneletPose(34603, 10.0, 0.0);
  const auto from2 = traffic_simulator::helper::constructLaneletPose(34579, 10.0, 0.0);

  const auto result1 = pose.getAlternativeLaneletPoseBaseOnShortestRouteFrom(from1, hdmap_utils);
  const auto result2 = pose.getAlternativeLaneletPoseBaseOnShortestRouteFrom(from2, hdmap_utils);

  ASSERT_TRUE(result1.has_value());
  ASSERT_TRUE(result2.has_value());

  EXPECT_EQ(result1.value().lanelet_id, 34603);
  EXPECT_EQ(result2.value().lanelet_id, 34603);
}

/**
 * @note Test function behavior when there are multiple lanelet_poses
 */
TEST_F(CanonicalizedLaneletPoseTest, getAlternativeLaneletPoseBaseOnShortestRouteFrom_multiple)
{
  CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(120659, -20.0, 0.0), hdmap_utils);
  const auto from1 = traffic_simulator::helper::constructLaneletPose(34603, 10.0, 0.0);
  const auto from2 = traffic_simulator::helper::constructLaneletPose(34579, 10.0, 0.0);

  const auto result1 = pose.getAlternativeLaneletPoseBaseOnShortestRouteFrom(from1, hdmap_utils);
  const auto result2 = pose.getAlternativeLaneletPoseBaseOnShortestRouteFrom(from2, hdmap_utils);

  ASSERT_TRUE(result1.has_value());
  ASSERT_TRUE(result2.has_value());

  EXPECT_EQ(result1.value().lanelet_id, 34603);
  EXPECT_EQ(result2.value().lanelet_id, 34579);
}

/**
 * @note Test accessor function
 */
TEST(CanonicalizedLaneletPose, getConsiderPoseByRoadSlope)
{
  EXPECT_EQ(CanonicalizedLaneletPose::getConsiderPoseByRoadSlope(), false);
}

/**
 * @note Test accessor function
 */
TEST(CanonicalizedLaneletPose, setConsiderPoseByRoadSlope)
{
  EXPECT_EQ(CanonicalizedLaneletPose::getConsiderPoseByRoadSlope(), false);
  CanonicalizedLaneletPose::setConsiderPoseByRoadSlope(true);
  EXPECT_EQ(CanonicalizedLaneletPose::getConsiderPoseByRoadSlope(), true);
}

/**
 * @note Test operator calculation correctness with CanonicalizedLaneletPose of lesser, equal and greater lanelet_id
 */
TEST_F(CanonicalizedLaneletPoseTest, operatorLessEqual)
{
  const CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(120659, 5.0, 0.0), hdmap_utils);
  const CanonicalizedLaneletPose pose_equal(
    traffic_simulator::helper::constructLaneletPose(120659, 5.0, 0.0), hdmap_utils);
  const CanonicalizedLaneletPose pose_less(
    traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0), hdmap_utils);
  const CanonicalizedLaneletPose pose_greater(
    traffic_simulator::helper::constructLaneletPose(120659, 6.0, 0.0), hdmap_utils);

  EXPECT_TRUE(pose_less <= pose);
  EXPECT_TRUE(pose_equal <= pose);
  EXPECT_FALSE(pose_greater <= pose);
}

/**
 * @note Test operator calculation correctness with CanonicalizedLaneletPose of lesser, equal and greater lanelet_id
 */
TEST_F(CanonicalizedLaneletPoseTest, operatorLess)
{
  const CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(120659, 5.0, 0.0), hdmap_utils);
  const CanonicalizedLaneletPose pose_equal(
    traffic_simulator::helper::constructLaneletPose(120659, 5.0, 0.0), hdmap_utils);
  const CanonicalizedLaneletPose pose_less(
    traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0), hdmap_utils);
  const CanonicalizedLaneletPose pose_greater(
    traffic_simulator::helper::constructLaneletPose(120659, 6.0, 0.0), hdmap_utils);

  EXPECT_TRUE(pose_less < pose);
  EXPECT_FALSE(pose_equal < pose);
  EXPECT_FALSE(pose_greater < pose);
}

/**
 * @note Test operator calculation correctness with CanonicalizedLaneletPose of lesser, equal and greater lanelet_id
 */
TEST_F(CanonicalizedLaneletPoseTest, operatorGreaterEqual)
{
  const CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(120659, 5.0, 0.0), hdmap_utils);
  const CanonicalizedLaneletPose pose_equal(
    traffic_simulator::helper::constructLaneletPose(120659, 5.0, 0.0), hdmap_utils);
  const CanonicalizedLaneletPose pose_less(
    traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0), hdmap_utils);
  const CanonicalizedLaneletPose pose_greater(
    traffic_simulator::helper::constructLaneletPose(120659, 6.0, 0.0), hdmap_utils);

  EXPECT_FALSE(pose_less >= pose);
  EXPECT_TRUE(pose_equal >= pose);
  EXPECT_TRUE(pose_greater >= pose);
}

/**
 * @note Test operator calculation correctness with CanonicalizedLaneletPose of lesser, equal and greater lanelet_id
 */
TEST_F(CanonicalizedLaneletPoseTest, operatorGreater)
{
  const CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(120659, 5.0, 0.0), hdmap_utils);
  const CanonicalizedLaneletPose pose_equal(
    traffic_simulator::helper::constructLaneletPose(120659, 5.0, 0.0), hdmap_utils);
  const CanonicalizedLaneletPose pose_less(
    traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0), hdmap_utils);
  const CanonicalizedLaneletPose pose_greater(
    traffic_simulator::helper::constructLaneletPose(120659, 6.0, 0.0), hdmap_utils);

  EXPECT_FALSE(pose_less > pose);
  EXPECT_FALSE(pose_equal > pose);
  EXPECT_TRUE(pose_greater > pose);
}

/**
 * @note Test function behavior when provided two poses occupying the same lanelet_id
 */
TEST_F(CanonicalizedLaneletPoseTest, isSameLaneletId_withPose_same)
{
  const CanonicalizedLaneletPose pose1(
    traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0), hdmap_utils);
  const CanonicalizedLaneletPose pose2(
    traffic_simulator::helper::constructLaneletPose(120659, 1.0, 0.0), hdmap_utils);

  EXPECT_TRUE(traffic_simulator::isSameLaneletId(pose1, pose2));
}

/**
 * @note Test function behavior when provided two poses occupying different lanelet_ids
 */
TEST_F(CanonicalizedLaneletPoseTest, isSameLaneletId_withPose_different)
{
  const CanonicalizedLaneletPose pose1(
    traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0), hdmap_utils);
  CanonicalizedLaneletPose pose2(
    traffic_simulator::helper::constructLaneletPose(34606, 1.0, 0.0), hdmap_utils);

  EXPECT_FALSE(traffic_simulator::isSameLaneletId(pose1, pose2));
}

/**
 * @note Test function behavior when provided with a pose having lanelet_id equal to the lanelet_id argument
 */
TEST_F(CanonicalizedLaneletPoseTest, isSameLaneletId_withLanelet_same)
{
  const CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0), hdmap_utils);

  EXPECT_TRUE(traffic_simulator::isSameLaneletId(pose, 120659));
}

/**
 * @note Test function behavior when provided with a pose having lanelet_id different to the lanelet_id argument
 */
TEST_F(CanonicalizedLaneletPoseTest, isSameLaneletId_withLanelet_different)
{
  const CanonicalizedLaneletPose pose(
    traffic_simulator::helper::constructLaneletPose(120659, 0.0, 0.0), hdmap_utils);

  EXPECT_FALSE(traffic_simulator::isSameLaneletId(pose, 34606));
}
