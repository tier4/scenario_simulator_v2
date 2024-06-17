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

#include <traffic_simulator/behavior/route_planner.hpp>

#include "../catalogs.hpp"
#include "../expect_eq_macros.hpp"
#include "../helper_functions.hpp"

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class RoutePlannerTest : public testing::Test
{
protected:
  RoutePlannerTest() : hdmap_utils_ptr(makeHdMapUtilsSharedPointer()), planner(hdmap_utils_ptr) {}

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr;
  traffic_simulator::RoutePlanner planner;
};

/**
 * @note Test functionality used by other units.
 * Test accessor getGoalPoses.
 */
TEST_F(RoutePlannerTest, getGoalPoses)
{
  const auto in_poses = std::vector<traffic_simulator::CanonicalizedLaneletPose>{
    makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659),
    makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120660),
    makeCanonicalizedLaneletPose(hdmap_utils_ptr, 34468)};

  planner.setWaypoints(in_poses);

  const auto out_poses = planner.getGoalPoses();

  EXPECT_EQ(in_poses.size(), out_poses.size());
  for (auto it_in = in_poses.begin(), it_out = out_poses.end();
       it_in != in_poses.end() && it_out != out_poses.end(); ++it_in, ++it_out) {
    EXPECT_LANELET_POSE_EQ(
      static_cast<traffic_simulator::LaneletPose>(*it_in),
      static_cast<traffic_simulator::LaneletPose>(*it_out));
  }
}

/**
 * @note Test functionality used by other units.
 * Test performing transformations by getGoalPosesInWorldFrame accessor.
 */
TEST_F(RoutePlannerTest, getGoalPosesInWorldFrame)
{
  const auto in_poses = std::vector<traffic_simulator::CanonicalizedLaneletPose>{
    makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659),
    makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120660),
    makeCanonicalizedLaneletPose(hdmap_utils_ptr, 34468)};

  planner.setWaypoints(in_poses);

  const auto out_poses = planner.getGoalPosesInWorldFrame();

  EXPECT_EQ(in_poses.size(), out_poses.size());
  for (size_t i = 0; i < in_poses.size(); i++) {
    EXPECT_POSE_EQ(static_cast<geometry_msgs::msg::Pose>(in_poses[i]), out_poses[i]);
  }
}

/**
 * @note Test functionality used by other units.
 * Test routing correctness with an entity pose and target pose spaced apart for more than horizon.
 */
TEST_F(RoutePlannerTest, getRouteLanelets_horizon)
{
  const lanelet::Id id_target = 34579;

  planner.setWaypoints({makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_target)});
  auto route =
    planner.getRouteLanelets(makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), 1000.0);

  EXPECT_TRUE(std::find(route.begin(), route.end(), id_target) != route.end());
}

/**
 * @note Test functionality used by other units.
 * Test routing correctness with an entity pose and target pose spaced apart for less than horizon.
 */
TEST_F(RoutePlannerTest, getRouteLanelets_noHorizon)
{
  lanelet::Id id_target = 34579;

  planner.setWaypoints({makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_target)});
  const auto route =
    planner.getRouteLanelets(makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), 100.0);

  EXPECT_FALSE(std::find(route.begin(), route.end(), id_target) != route.end());
}

/**
 * @note Test functionality used by other units.
 * Test routing correctness with an entity pose and empty waypoints vector
 * - the goal is to test function behavior when empty vector is passed.
 */
TEST_F(RoutePlannerTest, getRouteLanelets_empty)
{
  const lanelet::Ids following_ids({120659, 120660, 34468, 34465, 34462});

  planner.setWaypoints({});
  const auto route =
    planner.getRouteLanelets(makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), 100.0);

  EXPECT_EQ(route.size(), following_ids.size());
  for (size_t i = 0; i < route.size(); i++) {
    EXPECT_EQ(following_ids[i], route[i]);
  }
}
