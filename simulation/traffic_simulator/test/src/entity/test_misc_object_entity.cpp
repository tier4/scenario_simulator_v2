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
#include "../expect_eq_macros.hpp"
#include "../helper_functions.hpp"

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class MiscObjectEntityTest : public testing::Test
{
protected:
  MiscObjectEntityTest() : hdmap_utils_ptr(makeHdMapUtilsSharedPointer()), entity_name("blob") {}

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr;
  const std::string entity_name;
};
/**
 * @note Test basic functionality. Test current action obtaining when NPC logic is not started.
 */
TEST_F(MiscObjectEntityTest, getCurrentAction_npcNotStarted)
{
  auto non_canonicalized_status = makeEntityStatus(
    hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(), 0.0,
    entity_name);
  non_canonicalized_status.action_status.current_action = "current_action_name";

  const auto blob = traffic_simulator::entity::MiscObjectEntity(
    entity_name,
    traffic_simulator::entity_status::CanonicalizedEntityStatus(
      non_canonicalized_status, hdmap_utils_ptr),
    hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{});

  EXPECT_FALSE(blob.isNpcLogicStarted());
  EXPECT_EQ(blob.getCurrentAction(), "waiting");
}

/**
 * @note Test function behavior when absolute speed change is requested - the goal is to test throwing error.
 */
TEST_F(MiscObjectEntityTest, requestSpeedChange_absolute)
{
  EXPECT_THROW(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(),
        0.0, entity_name),
      hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .requestSpeedChange(10.0, false),
    common::SemanticError);
}

/**
 * @note Test function behavior when relative speed change is requested - the goal is to test throwing error.
 */
TEST_F(MiscObjectEntityTest, requestSpeedChange_relative)
{
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659);
  auto bbox = makeBoundingBox();

  auto blob = traffic_simulator::entity::MiscObjectEntity(
    entity_name, makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, 0.0, entity_name),
    hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{});

  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace(
    "bob_entity", makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, 17.0, "bob"));
  blob.setOtherStatus(others);

  EXPECT_THROW(
    blob.requestSpeedChange(
      traffic_simulator::speed_change::RelativeTargetSpeed(
        "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, 3.0),
      true),
    common::SemanticError);
}

/**
 * @note Test function behavior when relative speed change with transition type is requested
 * - the goal is to test throwing error.
 */
TEST_F(MiscObjectEntityTest, requestSpeedChange_absoluteTransition)
{
  EXPECT_THROW(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(),
        0.0, entity_name),
      hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .requestSpeedChange(
        10.0, traffic_simulator::speed_change::Transition::AUTO,
        traffic_simulator::speed_change::Constraint(
          traffic_simulator::speed_change::Constraint::Type::NONE, 0.0),
        false),
    common::SemanticError);
}

/**
 * @note Test function behavior when route assigning is requested with lanelet pose
 * - the goal is to test throwing error.
 */
TEST_F(MiscObjectEntityTest, requestAssignRoute_laneletPose)
{
  EXPECT_THROW(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(),
        0.0, entity_name),
      hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .requestAssignRoute(std::vector<traffic_simulator::lanelet_pose::CanonicalizedLaneletPose>{
        makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120660)}),
    common::SemanticError);
}

/**
 * @note Test function behavior when route assigning is requested with pose
 * - the goal is to test throwing error.
 */
TEST_F(MiscObjectEntityTest, requestAssignRoute_pose)
{
  EXPECT_THROW(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(),
        0.0, entity_name),
      hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .requestAssignRoute(
        std::vector<geometry_msgs::msg::Pose>{geometry_msgs::build<geometry_msgs::msg::Pose>()
                                                .position(makePoint(3759.34, 73791.38))
                                                .orientation(makeQuaternionFromYaw(0.0))}),
    common::SemanticError);
}

/**
 * @note Test function behavior when position acquiring is requested with lanelet pose
 * - the goal is to test throwing error.
 */
TEST_F(MiscObjectEntityTest, requestAcquirePosition_laneletPose)
{
  EXPECT_THROW(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(),
        0.0, entity_name),
      hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .requestAcquirePosition(makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120660)),
    common::SemanticError);
}

/**
 * @note Test function behavior when position acquiring is requested with pose
 * - the goal is to test throwing error.
 */
TEST_F(MiscObjectEntityTest, requestAcquirePosition_pose)
{
  EXPECT_THROW(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(),
        0.0, entity_name),
      hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .requestAcquirePosition(geometry_msgs::build<geometry_msgs::msg::Pose>()
                                .position(makePoint(3759.34, 73791.38))
                                .orientation(makeQuaternionFromYaw(0.0))),
    common::SemanticError);
}

/**
 * @note Test function behavior when called with any argument - the goal is to test error throwing.
 */
TEST_F(MiscObjectEntityTest, getRouteLanelets)
{
  EXPECT_THROW(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(),
        0.0, entity_name),
      hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .getRouteLanelets(100.0),
    common::SemanticError);
}