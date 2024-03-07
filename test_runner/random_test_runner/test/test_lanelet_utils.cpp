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
//
// Co-developed by TIER IV, Inc. and Robotec.AI sp. z o.o.

#include <gtest/gtest.h>

#include <random_test_runner/lanelet_utils.hpp>

#include "expect_eq_macros.hpp"
#include "test_utils.hpp"

constexpr double EPS = 1e-6;

TEST(LaneletUtils, initialize)
{
  const std::string path =
    ament_index_cpp::get_package_share_directory("random_test_runner") + "/map/lanelet2_map.osm";
  EXPECT_NO_THROW(LaneletUtils utils(path));
}

TEST(LaneletUtils, computeDistance)
{
  traffic_simulator_msgs::msg::LaneletPose from = makeLaneletPose(34621, 0.0),
                                           to = makeLaneletPose(34621, 1.0);
  EXPECT_NEAR(1.0, getLaneletUtils().computeDistance(from, to), EPS);
}

TEST(LaneletUtils, computeDistance_reverse)
{
  traffic_simulator_msgs::msg::LaneletPose from = makeLaneletPose(34621, 21.3),
                                           to = makeLaneletPose(34621, 17.7);
  EXPECT_NEAR(getLaneletUtils().computeDistance(from, to), 3.6, EPS);
}

TEST(LaneletUtils, computeDistance_differentLanelet)
{
  traffic_simulator_msgs::msg::LaneletPose from = makeLaneletPose(34594, 15.0),
                                           to = makeLaneletPose(34621, 5.0);
  EXPECT_NEAR(getLaneletUtils().computeDistance(from, to), 10.032117179351, 1e-2);
}

TEST(LaneletUtils, getOppositeLanelet)
{
  traffic_simulator_msgs::msg::LaneletPose pose = makeLaneletPose(34621, 5.0);
  std::optional<traffic_simulator_msgs::msg::LaneletPose> opposite =
    getLaneletUtils().getOppositeLaneLet(pose);
  EXPECT_TRUE(opposite);
  EXPECT_LANELET_POSE_NEAR(opposite.value(), makeLaneletPose(34981, 3.611), 1e-2);
}

TEST(LaneletUtils, getOppositeLanelet_notExisting)
{
  traffic_simulator_msgs::msg::LaneletPose pose = makeLaneletPose(1, 0.0);
  std::optional<traffic_simulator_msgs::msg::LaneletPose> opposite =
    getLaneletUtils().getOppositeLaneLet(pose);
  EXPECT_FALSE(opposite);
}

TEST(LaneletUtils, getLanesWithinDistance_incorrectDistances)
{
  traffic_simulator_msgs::msg::LaneletPose pose = makeLaneletPose(34621, 5.0);
  EXPECT_THROW(getLaneletUtils().getLanesWithinDistance(pose, 1.0, 0.0), std::runtime_error);
}
#include <algorithm>
TEST(LaneletUtils, getLanesWithinDistance_straightRoad)
{
  traffic_simulator_msgs::msg::LaneletPose pose = makeLaneletPose(34462, 15.0);
  EXPECT_NO_THROW(getLaneletUtils().getLanesWithinDistance(pose, 0.0, 10.0));
  std::vector<LaneletPart> lanes = getLaneletUtils().getLanesWithinDistance(pose, 0.0, 10.0);
  EXPECT_EQ(lanes.size(), size_t(4));
  EXPECT_EQ(lanes[0].lanelet_id, 34462);
  EXPECT_EQ(lanes[1].lanelet_id, 34462);
  EXPECT_EQ(lanes[2].lanelet_id, 34513);
  EXPECT_EQ(lanes[3].lanelet_id, 34513);
}

TEST(LaneletUtils, getLanesWithinDistance_intersection)
{
  traffic_simulator_msgs::msg::LaneletPose pose = makeLaneletPose(34621, 5.0);
  EXPECT_NO_THROW(getLaneletUtils().getLanesWithinDistance(pose, 0.0, 10.0));
  std::vector<LaneletPart> lanes = getLaneletUtils().getLanesWithinDistance(pose, 0.0, 10.0);
  EXPECT_EQ(lanes.size(), size_t(13));
  EXPECT_EQ(lanes[0].lanelet_id, 34585);
  EXPECT_EQ(lanes[1].lanelet_id, 34594);
  EXPECT_EQ(lanes[2].lanelet_id, 34621);
  EXPECT_EQ(lanes[3].lanelet_id, 34621);
  EXPECT_EQ(lanes[4].lanelet_id, 34636);
  EXPECT_EQ(lanes[5].lanelet_id, 34642);
  EXPECT_EQ(lanes[6].lanelet_id, 34645);
  EXPECT_EQ(lanes[7].lanelet_id, 34651);
  EXPECT_EQ(lanes[8].lanelet_id, 34976);
  EXPECT_EQ(lanes[9].lanelet_id, 34981);
  EXPECT_EQ(lanes[10].lanelet_id, 34981);
  EXPECT_EQ(lanes[11].lanelet_id, 35001);
  EXPECT_EQ(lanes[12].lanelet_id, 35051);
}

TEST(LaneletUtils, getLaneletIds)
{
  EXPECT_NO_THROW(getLaneletUtils().getLaneletIds());
  const auto ids = getLaneletUtils().getLaneletIds();
  EXPECT_EQ(ids.size(), size_t(83));
  unsigned int idx = 0;
  EXPECT_EQ(ids[idx++], 35051);
  EXPECT_EQ(ids[idx++], 35036);
  EXPECT_EQ(ids[idx++], 35001);
  EXPECT_EQ(ids[idx++], 34850);
  EXPECT_EQ(ids[idx++], 34786);
  EXPECT_EQ(ids[idx++], 34783);
  EXPECT_EQ(ids[idx++], 35031);
  EXPECT_EQ(ids[idx++], 34777);
  EXPECT_EQ(ids[idx++], 34774);
  EXPECT_EQ(ids[idx++], 34768);
  EXPECT_EQ(ids[idx++], 35016);
  EXPECT_EQ(ids[idx++], 34762);
  EXPECT_EQ(ids[idx++], 34603);
  EXPECT_EQ(ids[idx++], 120659);
  EXPECT_EQ(ids[idx++], 34426);
  EXPECT_EQ(ids[idx++], 34591);
  EXPECT_EQ(ids[idx++], 34795);
  EXPECT_EQ(ids[idx++], 34414);
  EXPECT_EQ(ids[idx++], 34579);
  EXPECT_EQ(ids[idx++], 34756);
  EXPECT_EQ(ids[idx++], 34576);
  EXPECT_EQ(ids[idx++], 34780);
  EXPECT_EQ(ids[idx++], 34399);
  EXPECT_EQ(ids[idx++], 34753);
  EXPECT_EQ(ids[idx++], 34570);
  EXPECT_EQ(ids[idx++], 34564);
  EXPECT_EQ(ids[idx++], 34741);
  EXPECT_EQ(ids[idx++], 34976);
  EXPECT_EQ(ids[idx++], 34468);
  EXPECT_EQ(ids[idx++], 35026);
  EXPECT_EQ(ids[idx++], 34645);
  EXPECT_EQ(ids[idx++], 34465);
  EXPECT_EQ(ids[idx++], 34642);
  EXPECT_EQ(ids[idx++], 34585);
  EXPECT_EQ(ids[idx++], 34789);
  EXPECT_EQ(ids[idx++], 34408);
  EXPECT_EQ(ids[idx++], 34498);
  EXPECT_EQ(ids[idx++], 34675);
  EXPECT_EQ(ids[idx++], 35046);
  EXPECT_EQ(ids[idx++], 34792);
  EXPECT_EQ(ids[idx++], 34411);
  EXPECT_EQ(ids[idx++], 34392);
  EXPECT_EQ(ids[idx++], 34510);
  EXPECT_EQ(ids[idx++], 34438);
  EXPECT_EQ(ids[idx++], 34615);
  EXPECT_EQ(ids[idx++], 34495);
  EXPECT_EQ(ids[idx++], 34672);
  EXPECT_EQ(ids[idx++], 34621);
  EXPECT_EQ(ids[idx++], 34594);
  EXPECT_EQ(ids[idx++], 34507);
  EXPECT_EQ(ids[idx++], 34684);
  EXPECT_EQ(ids[idx++], 34420);
  EXPECT_EQ(ids[idx++], 34423);
  EXPECT_EQ(ids[idx++], 34981);
  EXPECT_EQ(ids[idx++], 34600);
  EXPECT_EQ(ids[idx++], 34462);
  EXPECT_EQ(ids[idx++], 34385);
  EXPECT_EQ(ids[idx++], 34639);
  EXPECT_EQ(ids[idx++], 35021);
  EXPECT_EQ(ids[idx++], 34513);
  EXPECT_EQ(ids[idx++], 34690);
  EXPECT_EQ(ids[idx++], 34441);
  EXPECT_EQ(ids[idx++], 34606);
  EXPECT_EQ(ids[idx++], 34624);
  EXPECT_EQ(ids[idx++], 34630);
  EXPECT_EQ(ids[idx++], 34633);
  EXPECT_EQ(ids[idx++], 34636);
  EXPECT_EQ(ids[idx++], 34648);
  EXPECT_EQ(ids[idx++], 34651);
  EXPECT_EQ(ids[idx++], 34654);
  EXPECT_EQ(ids[idx++], 34666);
  EXPECT_EQ(ids[idx++], 34678);
  EXPECT_EQ(ids[idx++], 120660);
  EXPECT_EQ(ids[idx++], 34681);
  EXPECT_EQ(ids[idx++], 120545);
  EXPECT_EQ(ids[idx++], 34693);
  EXPECT_EQ(ids[idx++], 34696);
  EXPECT_EQ(ids[idx++], 34705);
  EXPECT_EQ(ids[idx++], 34708);
  EXPECT_EQ(ids[idx++], 34744);
  EXPECT_EQ(ids[idx++], 34750);
  EXPECT_EQ(ids[idx++], 34378);
  EXPECT_EQ(ids[idx++], 34759);
}

TEST(LaneletUtils, toMapPose)
{
  const traffic_simulator_msgs::msg::LaneletPose pose = makeLaneletPose(34621, 10.0);
  EXPECT_NO_THROW(getLaneletUtils().toMapPose(pose, false));
  EXPECT_POSE_NEAR(
    getLaneletUtils().toMapPose(pose, false).pose,
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                  .x(3747.3511648127)
                  .y(73735.0699484234)
                  .z(0.3034051453))
      .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                     .x(0.0)
                     .y(0.0)
                     .z(-0.9719821398)
                     .w(0.2350547166)),
    EPS);
}

TEST(LaneletUtils, toMapPoseWithFillingPitch)
{
  const traffic_simulator_msgs::msg::LaneletPose pose = makeLaneletPose(34621, 10.0);
  EXPECT_NO_THROW(getLaneletUtils().toMapPose(pose, true));
  /// @note orientation data is output as of #1103
  EXPECT_POSE_NEAR(
    getLaneletUtils().toMapPose(pose, true).pose,
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                  .x(3747.3511648127)
                  .y(73735.0699484234)
                  .z(0.3034051453))
      .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                     .x(-0.0091567745565503053)
                     .y(-0.0022143853886961245)
                     .z(-0.97193900717756765)
                     .w(0.23504428583514828)),
    EPS);
}

TEST(LaneletUtils, getRoute_turn)
{
  EXPECT_NO_THROW(getLaneletUtils().getRoute(34681, 34513));
  const std::vector<int64_t> route = getLaneletUtils().getRoute(34681, 34513);
  EXPECT_EQ(route.size(), size_t(3));
  EXPECT_EQ(route[0], 34681);
  EXPECT_EQ(route[1], 34684);
  EXPECT_EQ(route[2], 34513);
}

TEST(LaneletUtils, getRoute_intersection)
{
  EXPECT_NO_THROW(getLaneletUtils().getRoute(34600, 34621));
  const std::vector<int64_t> route = getLaneletUtils().getRoute(34600, 34621);
  EXPECT_EQ(route.size(), size_t(3));
  EXPECT_EQ(route[0], 34600);
  EXPECT_EQ(route[1], 34594);
  EXPECT_EQ(route[2], 34621);
}

TEST(LaneletUtils, getLaneletLength)
{
  EXPECT_NEAR(getLaneletUtils().getLaneletLength(34621), 49.9125380565, EPS);
  EXPECT_NEAR(getLaneletUtils().getLaneletLength(34507), 55.4273243719, EPS);
  EXPECT_NEAR(getLaneletUtils().getLaneletLength(34570), 19.5778003979, EPS);
}

TEST(LaneletUtils, isInLanelet)
{
  // lanelet 34621
  EXPECT_TRUE(getLaneletUtils().isInLanelet(34621, 10.0));
  EXPECT_FALSE(getLaneletUtils().isInLanelet(34621, -1.0));
  EXPECT_FALSE(getLaneletUtils().isInLanelet(34621, 50.0));
  // lanelet 34507
  EXPECT_TRUE(getLaneletUtils().isInLanelet(34507, 10.0));
  EXPECT_FALSE(getLaneletUtils().isInLanelet(34507, -1.0));
  EXPECT_FALSE(getLaneletUtils().isInLanelet(34507, 56.0));
  // lanelet 34570
  EXPECT_TRUE(getLaneletUtils().isInLanelet(34570, 10.0));
  EXPECT_FALSE(getLaneletUtils().isInLanelet(34570, -1.0));
  EXPECT_FALSE(getLaneletUtils().isInLanelet(34570, 20.0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
