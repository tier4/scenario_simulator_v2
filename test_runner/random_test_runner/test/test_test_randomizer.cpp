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

#include <iostream>
#include <random_test_runner/test_randomizer.hpp>
#include <rclcpp/rclcpp.hpp>

#include "expect_eq_macros.hpp"
#include "test_utils.hpp"

constexpr double EPS = 1e-6;

TEST(TestRandomizer, initialize)
{
  TestSuiteParameters suite_params;
  suite_params.ego_goal_lanelet_id = 1;
  suite_params.ego_goal_partial_randomization = true;
  TestCaseParameters case_params;
  case_params.seed = 1;
  TestRandomizer randomizer(
    rclcpp::get_logger("test_randomizer"), suite_params, case_params, getLaneletUtilsPtr());
}

TEST(TestRandomizer, generate_10NPC)
{
  TestSuiteParameters suite_params;
  suite_params.ego_goal_lanelet_id = 34621;
  suite_params.ego_goal_partial_randomization = true;
  TestCaseParameters case_params;
  case_params.seed = 1;
  rclcpp::Logger logger = rclcpp::get_logger("test_randomizer");
  logger.set_level(rclcpp::Logger::Level::Unset);
  TestRandomizer randomizer(logger, suite_params, case_params, getLaneletUtilsPtr());

  TestDescription description = randomizer.generate();
  // ego data
  EXPECT_LANELET_POSE_NEAR(
    description.ego_start_position, makeLaneletPose(34705, 6.2940393113), EPS);
  EXPECT_LANELET_POSE_NEAR(
    description.ego_goal_position, makeLaneletPose(34642, 9.6945373700), EPS);
  EXPECT_POSE_NEAR(
    description.ego_goal_pose,
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                  .x(3759.7298049304)
                  .y(73741.1526960728)
                  .z(0.0698702227))
      .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                     .x(0.0)
                     .y(0.0)
                     .z(-0.9854984261)
                     .w(0.1696845662)),
    EPS);
  // npc data
  EXPECT_EQ(description.npcs_descriptions.size(), size_t(10));
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[0],
    makeNPCDescription("npc0", 0.7308464889, makeLaneletPose(34624, 7.5543207194)), EPS)
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[1],
    makeNPCDescription("npc1", 2.1743651011, makeLaneletPose(34642, 5.2207237465)), EPS);
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[2],
    makeNPCDescription("npc2", 1.8113704071, makeLaneletPose(34786, 3.7184477027)), EPS);
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[3],
    makeNPCDescription("npc3", 0.5684689900, makeLaneletPose(34615, 21.0757887426)), EPS);
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[4],
    makeNPCDescription("npc4", 1.6430120231, makeLaneletPose(34684, 5.5834935067)), EPS);
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[5],
    makeNPCDescription("npc5", 0.9952537087, makeLaneletPose(34690, 18.9720777813)), EPS);
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[6],
    makeNPCDescription("npc6", 2.5068937586, makeLaneletPose(34762, 6.9548768523)), EPS);
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[7],
    makeNPCDescription("npc7", 0.5976369610, makeLaneletPose(34976, 2.5577956602)), EPS);
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[8],
    makeNPCDescription("npc8", 2.1763201035, makeLaneletPose(34621, 9.4524364223)), EPS);
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[9],
    makeNPCDescription("npc9", 0.9938772341, makeLaneletPose(34708, 7.0635798983)), EPS);
}

TEST(TestRandomizer, generate_8NPC)
{
  TestSuiteParameters suite_params;
  suite_params.ego_goal_lanelet_id = 34468;
  suite_params.ego_goal_partial_randomization = true;
  suite_params.npcs_count = 8;
  TestCaseParameters case_params;
  case_params.seed = 1234;
  TestRandomizer randomizer(
    rclcpp::get_logger("test_randomizer"), suite_params, case_params, getLaneletUtilsPtr());

  TestDescription description = randomizer.generate();
  // ego data
  EXPECT_LANELET_POSE_NEAR(
    description.ego_start_position, makeLaneletPose(34648, 11.3744011441), EPS);
  EXPECT_LANELET_POSE_NEAR(
    description.ego_goal_position, makeLaneletPose(34507, 55.3754803281), EPS);
  EXPECT_POSE_NEAR(
    description.ego_goal_pose,
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(geometry_msgs::build<geometry_msgs::msg::Point>()
                  .x(3799.5102437521)
                  .y(73815.1830249432)
                  .z(-2.9975350010))
      .orientation(geometry_msgs::build<geometry_msgs::msg::Quaternion>()
                     .x(0.0)
                     .y(0.0)
                     .z(0.2345315792)
                     .w(0.9721085013)),
    EPS);
  // npc data
  EXPECT_EQ(description.npcs_descriptions.size(), size_t(8));
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[0],
    makeNPCDescription("npc0", 1.1814815242, makeLaneletPose(34648, 1.0719996393)), EPS)
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[1],
    makeNPCDescription("npc1", 2.5379073352, makeLaneletPose(34441, 1.8085656480)), EPS);
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[2],
    makeNPCDescription("npc2", 1.3945431898, makeLaneletPose(34783, 13.6066129043)), EPS);
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[3],
    makeNPCDescription("npc3", 1.3275385670, makeLaneletPose(34411, 6.2502502346)), EPS);
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[4],
    makeNPCDescription("npc4", 1.9029904729, makeLaneletPose(34606, 14.5456329040)), EPS);
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[5],
    makeNPCDescription("npc5", 0.6884530926, makeLaneletPose(34507, 31.5435000028)), EPS);
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[6],
    makeNPCDescription("npc6", 1.4844558761, makeLaneletPose(34783, 1.8243487931)), EPS);
  EXPECT_NPC_DESCRIPTION_NEAR(
    description.npcs_descriptions[7],
    makeNPCDescription("npc7", 2.4718253580, makeLaneletPose(34603, 13.0118273897)), EPS);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
