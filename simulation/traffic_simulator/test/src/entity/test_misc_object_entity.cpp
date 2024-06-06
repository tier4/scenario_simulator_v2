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
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/entity/misc_object_entity.hpp>
#include <traffic_simulator/helper/helper.hpp>

#include "../catalogs.hpp"
#include "../entity_helper_functions.hpp"
#include "../expect_eq_macros.hpp"

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/**
 * @note Test basic functionality. Test current action obtaining when NPC logic is not started.
 */
TEST(MiscObjectEntity, getCurrentAction_npcNotStarted)
{
  lanelet::Id id = 120659;
  const double initial_speed = 0.0;
  std::string entity_name("blob");
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto non_canonicalized_status =
    makeEntityStatus(hdmap_utils_ptr, pose, bbox, initial_speed, entity_name);
  non_canonicalized_status.action_status.current_action = "purposelessly_existing";
  traffic_simulator::entity_status::CanonicalizedEntityStatus status(
    non_canonicalized_status, hdmap_utils_ptr);
  traffic_simulator_msgs::msg::MiscObjectParameters params{};
  traffic_simulator::entity::MiscObjectEntity blob(entity_name, status, hdmap_utils_ptr, params);

  EXPECT_FALSE(blob.isNpcLogicStarted());
  EXPECT_TRUE(blob.getCurrentAction() == "waiting");
}

/**
 * @note Test basic functionality. Test current action obtaining when NPC logic is started.
 */
TEST(MiscObjectEntity, getCurrentAction_npcStarted)
{
  lanelet::Id id = 120659;
  const double initial_speed = 0.0;
  std::string entity_name("blob");
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto non_canonicalized_status =
    makeEntityStatus(hdmap_utils_ptr, pose, bbox, initial_speed, entity_name);
  non_canonicalized_status.action_status.current_action = "purposelessly_existing";
  traffic_simulator::entity_status::CanonicalizedEntityStatus status(
    non_canonicalized_status, hdmap_utils_ptr);
  traffic_simulator_msgs::msg::MiscObjectParameters params{};
  traffic_simulator::entity::MiscObjectEntity blob(entity_name, status, hdmap_utils_ptr, params);

  blob.startNpcLogic();
  EXPECT_TRUE(blob.isNpcLogicStarted());
  EXPECT_TRUE(blob.getCurrentAction() == "purposelessly_existing");
}

/**
 * @note Test function behavior when absolute speed change is requested - the goal is to test throwing error.
 */
TEST(MiscObjectEntity, requestSpeedChange_absolute)
{
  lanelet::Id id = 120659;
  const double initial_speed = 0.0;
  std::string entity_name("blob");
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status =
    makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, initial_speed, entity_name);
  traffic_simulator_msgs::msg::MiscObjectParameters params{};
  traffic_simulator::entity::MiscObjectEntity blob(entity_name, status, hdmap_utils_ptr, params);

  const double target_speed = 10.0;
  const bool continuous = false;
  EXPECT_THROW(blob.requestSpeedChange(target_speed, continuous), common::Error);
}

/**
 * @note Test function behavior when relative speed change is requested - the goal is to test throwing error.
 */
TEST(MiscObjectEntity, requestSpeedChange_relative)
{
  lanelet::Id id = 120659;
  const double initial_speed = 0.0;
  std::string entity_name("blob");
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status =
    makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, initial_speed, entity_name);
  traffic_simulator_msgs::msg::MiscObjectParameters params{};
  traffic_simulator::entity::MiscObjectEntity blob(entity_name, status, hdmap_utils_ptr, params);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  blob.setOtherStatus(others);

  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  bool continuous = true;

  EXPECT_THROW(blob.requestSpeedChange(relative_taget_speed, continuous), common::Error);
}

/**
 * @note Test function behavior when relative speed change with transition type is requested
 * - the goal is to test throwing error.
 */
TEST(MiscObjectEntity, requestSpeedChange_absoluteTransition)
{
  lanelet::Id id = 120659;
  const double initial_speed = 0.0;
  std::string entity_name("blob");
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status =
    makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, initial_speed, entity_name);
  traffic_simulator_msgs::msg::MiscObjectParameters params{};
  traffic_simulator::entity::MiscObjectEntity blob(entity_name, status, hdmap_utils_ptr, params);

  const double target_speed = 10.0;
  auto transition = traffic_simulator::speed_change::Transition::AUTO;
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::NONE, 0.0);
  const bool continuous = false;

  EXPECT_THROW(
    blob.requestSpeedChange(target_speed, transition, constraint, continuous), common::Error);
}

/**
 * @note Test function behavior when route assigning is requested with lanelet pose
 * - the goal is to test throwing error.
 */
TEST(MiscObjectEntity, requestAssignRoute_laneletPose)
{
  lanelet::Id id_0 = 120659;
  lanelet::Id id_1 = 120660;
  const double initial_speed = 0.0;
  std::string entity_name("blob");
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_0);
  auto bbox = makeBoundingBox();
  auto status =
    makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, initial_speed, entity_name);
  traffic_simulator_msgs::msg::MiscObjectParameters params{};
  traffic_simulator::entity::MiscObjectEntity blob(entity_name, status, hdmap_utils_ptr, params);

  auto target_pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_1);

  std::vector<traffic_simulator::lanelet_pose::CanonicalizedLaneletPose> target_poses;
  target_poses.push_back(target_pose);

  EXPECT_THROW(blob.requestAssignRoute(target_poses), common::Error);
}

/**
 * @note Test function behavior when route assigning is requested with pose
 * - the goal is to test throwing error.
 */
TEST(MiscObjectEntity, requestAssignRoute_pose)
{
  lanelet::Id id = 120659;
  const double initial_speed = 0.0;
  std::string entity_name("blob");
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status =
    makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, initial_speed, entity_name);
  traffic_simulator_msgs::msg::MiscObjectParameters params{};
  traffic_simulator::entity::MiscObjectEntity blob(entity_name, status, hdmap_utils_ptr, params);

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 3759.34;
  target_pose.position.y = 73791.38;

  std::vector<geometry_msgs::msg::Pose> target_poses;
  target_poses.push_back(target_pose);

  EXPECT_THROW(blob.requestAssignRoute(target_poses), common::Error);
}

/**
 * @note Test function behavior when position acquiring is requested with lanelet pose
 * - the goal is to test throwing error.
 */
TEST(MiscObjectEntity, requestAcquirePosition_laneletPose)
{
  lanelet::Id id_0 = 120659;
  lanelet::Id id_1 = 120660;
  const double initial_speed = 0.0;
  std::string entity_name("blob");
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_0);
  auto bbox = makeBoundingBox();
  auto status =
    makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, initial_speed, entity_name);
  traffic_simulator_msgs::msg::MiscObjectParameters params{};
  traffic_simulator::entity::MiscObjectEntity blob(entity_name, status, hdmap_utils_ptr, params);

  auto target_pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_1);

  EXPECT_THROW(blob.requestAcquirePosition(target_pose), common::Error);
}

/**
 * @note Test function behavior when position acquiring is requested with pose
 * - the goal is to test throwing error.
 */
TEST(MiscObjectEntity, requestAcquirePosition_pose)
{
  lanelet::Id id = 120659;
  const double initial_speed = 0.0;
  std::string entity_name("blob");
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status =
    makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, initial_speed, entity_name);
  traffic_simulator_msgs::msg::MiscObjectParameters params{};
  traffic_simulator::entity::MiscObjectEntity blob(entity_name, status, hdmap_utils_ptr, params);

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 3759.34;
  target_pose.position.y = 73791.38;

  EXPECT_THROW(blob.requestAcquirePosition(target_pose), common::Error);
}

/**
 * @note Test function behavior when called with any argument - the goal is to test error throwing.
 */
TEST(MiscObjectEntity, getRouteLanelets)
{
  lanelet::Id id = 120659;
  const double initial_speed = 0.0;
  std::string entity_name("blob");
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status =
    makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, initial_speed, entity_name);
  traffic_simulator_msgs::msg::MiscObjectParameters params{};
  traffic_simulator::entity::MiscObjectEntity blob(entity_name, status, hdmap_utils_ptr, params);

  const double horizon = 100.0;
  EXPECT_THROW(blob.getRouteLanelets(horizon), common::Error);
}
