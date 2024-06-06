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
#include "../entity_helper_functions.hpp"
#include "../expect_eq_macros.hpp"

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/**
 * @note Test functionality used by other units.
 * Test accessor getGoalPoses.
 */
TEST(RoutePlanner, getGoalPoses)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  traffic_simulator::RoutePlanner planner(hdmap_utils_ptr);
  lanelet::Id id_0 = 120659;
  lanelet::Id id_1 = 120660;
  lanelet::Id id_2 = 34468;
  auto pose_0 = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_0);
  auto pose_1 = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_1);
  auto pose_2 = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_2);

  std::vector<traffic_simulator::CanonicalizedLaneletPose> in_poses;
  in_poses.push_back(pose_0);
  in_poses.push_back(pose_1);
  in_poses.push_back(pose_2);

  planner.setWaypoints(in_poses);

  auto out_poses = planner.getGoalPoses();

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
TEST(RoutePlanner, getGoalPosesInWorldFrame)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  traffic_simulator::RoutePlanner planner(hdmap_utils_ptr);
  lanelet::Id id_0 = 120659;
  lanelet::Id id_1 = 120660;
  lanelet::Id id_2 = 34468;
  auto pose_0 = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_0);
  auto pose_1 = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_1);
  auto pose_2 = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_2);

  std::vector<traffic_simulator::CanonicalizedLaneletPose> in_poses;
  in_poses.push_back(pose_0);
  in_poses.push_back(pose_1);
  in_poses.push_back(pose_2);

  planner.setWaypoints(in_poses);

  auto out_poses = planner.getGoalPosesInWorldFrame();

  EXPECT_EQ(in_poses.size(), out_poses.size());
  for (size_t i = 0; i < in_poses.size(); i++) {
    EXPECT_POSE_EQ(static_cast<geometry_msgs::msg::Pose>(in_poses[i]), out_poses[i]);
  }
}

/**
 * @note Test functionality used by other units.
 * Test routing correctness with an entity pose and target pose spaced apart for more than horizon.
 */
TEST(RoutePlanner, getRouteLanelets_horizon)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  traffic_simulator::RoutePlanner planner(hdmap_utils_ptr);

  lanelet::Id id_start = 120659;
  lanelet::Id id_target = 34579;
  auto pose_start = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_start);
  auto pose_target = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_target);

  const double distance = 1000;

  planner.setWaypoints({pose_target});
  auto route = planner.getRouteLanelets(pose_start, distance);

  EXPECT_TRUE(std::find(route.begin(), route.end(), id_target) != route.end());
}

/**
 * @note Test functionality used by other units.
 * Test routing correctness with an entity pose and target pose spaced apart for less than horizon.
 */
TEST(RoutePlanner, getRouteLanelets_noHorizon)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  traffic_simulator::RoutePlanner planner(hdmap_utils_ptr);

  lanelet::Id id_start = 120659;
  lanelet::Id id_target = 34579;
  auto pose_start = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_start);
  auto pose_target = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_target);

  const double distance = 100;

  planner.setWaypoints({pose_target});
  auto route = planner.getRouteLanelets(pose_start, distance);

  EXPECT_FALSE(std::find(route.begin(), route.end(), id_target) != route.end());
}

/**
 * @note Test functionality used by other units.
 * Test routing correctness with an entity pose and empty waypoints vector
 * - the goal is to test function behavior when empty vector is passed.
 */
TEST(RoutePlanner, getRouteLanelets_empty)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  traffic_simulator::RoutePlanner planner(hdmap_utils_ptr);

  lanelet::Id id_0 = 120659;
  auto pose_0 = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_0);

  const double distance = 100;
  lanelet::Ids following_ids({120659, 120660, 34468, 34465, 34462});

  planner.setWaypoints({});
  auto route = planner.getRouteLanelets(pose_0, distance);

  EXPECT_EQ(route.size(), following_ids.size());
  for (size_t i = 0; i < route.size(); i++) {
    EXPECT_EQ(following_ids[i], route[i]);
  }
}
