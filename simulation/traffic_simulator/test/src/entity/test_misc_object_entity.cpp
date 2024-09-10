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
#include <traffic_simulator/utils/pose.hpp>

#include "../catalogs.hpp"
#include "../expect_eq_macros.hpp"
#include "../helper_functions.hpp"

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class MiscObjectEntityTest_HdMapUtils : public testing::Test
{
protected:
  MiscObjectEntityTest_HdMapUtils()
  : hdmap_utils_ptr(makeHdMapUtilsSharedPointer()), entity_name("misc_object_entity")
  {
  }

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr;
  const std::string entity_name;
};

class MiscObjectEntityTest_FullObject : public MiscObjectEntityTest_HdMapUtils
{
protected:
  MiscObjectEntityTest_FullObject()
  : id(120659),
    pose(makeCanonicalizedLaneletPose(hdmap_utils_ptr, id)),
    bbox(makeBoundingBox()),
    status(makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, 0.0, entity_name)),
    misc_object(
      entity_name, status, hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{}),
    entity_base(&misc_object)
  {
  }

  const lanelet::Id id;
  traffic_simulator::lanelet_pose::CanonicalizedLaneletPose pose;
  traffic_simulator_msgs::msg::BoundingBox bbox;
  traffic_simulator::entity_status::CanonicalizedEntityStatus status;
  traffic_simulator::entity::MiscObjectEntity misc_object;
  traffic_simulator::entity::EntityBase * entity_base;
};

/**
 * @note Test basic functionality. Test current action obtaining when NPC logic is not started.
 */
TEST_F(MiscObjectEntityTest_HdMapUtils, getCurrentAction_npcNotStarted)
{
  auto non_canonicalized_status = makeEntityStatus(
    hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(), 0.0,
    entity_name, traffic_simulator_msgs::msg::EntityType::MISC_OBJECT);
  non_canonicalized_status.action_status.current_action = "current_action_name";

  const auto blob = traffic_simulator::entity::MiscObjectEntity(
    entity_name,
    traffic_simulator::entity_status::CanonicalizedEntityStatus(
      non_canonicalized_status, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659)),
    hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{});

  EXPECT_EQ(blob.getCurrentAction(), "current_action_name");
}

/**
 * @note Test function behavior when absolute speed change is requested - the goal is to test throwing error.
 */
TEST_F(MiscObjectEntityTest_HdMapUtils, requestSpeedChange_absolute)
{
  EXPECT_THROW(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(),
        0.0, entity_name, traffic_simulator_msgs::msg::EntityType::MISC_OBJECT),
      hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .requestSpeedChange(10.0, false),
    common::SemanticError);
}

/**
 * @note Test function behavior when relative speed change is requested - the goal is to test throwing error.
 */
TEST_F(MiscObjectEntityTest_HdMapUtils, requestSpeedChange_relative)
{
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659);
  auto bbox = makeBoundingBox();

  auto blob = traffic_simulator::entity::MiscObjectEntity(
    entity_name,
    makeCanonicalizedEntityStatus(
      hdmap_utils_ptr, pose, bbox, 0.0, entity_name,
      traffic_simulator_msgs::msg::EntityType::MISC_OBJECT),
    hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{});

  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace(
    "other_entity", makeCanonicalizedEntityStatus(
                      hdmap_utils_ptr, pose, bbox, 17.0, "other_entity_name",
                      traffic_simulator_msgs::msg::EntityType::MISC_OBJECT));
  blob.setOtherStatus(others);

  EXPECT_THROW(
    blob.requestSpeedChange(
      traffic_simulator::speed_change::RelativeTargetSpeed(
        "other_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, 3.0),
      true),
    common::SemanticError);
}

/**
 * @note Test function behavior when relative speed change with transition type is requested
 * - the goal is to test throwing error.
 */
TEST_F(MiscObjectEntityTest_HdMapUtils, requestSpeedChange_absoluteTransition)
{
  EXPECT_THROW(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(),
        0.0, entity_name, traffic_simulator_msgs::msg::EntityType::MISC_OBJECT),
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
TEST_F(MiscObjectEntityTest_HdMapUtils, requestAssignRoute_laneletPose)
{
  EXPECT_THROW(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(),
        0.0, entity_name, traffic_simulator_msgs::msg::EntityType::MISC_OBJECT),
      hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .requestAssignRoute(std::vector<traffic_simulator::lanelet_pose::CanonicalizedLaneletPose>{
        makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120660)}),
    common::SemanticError);
}

/**
 * @note Test function behavior when route assigning is requested with pose
 * - the goal is to test throwing error.
 */
TEST_F(MiscObjectEntityTest_HdMapUtils, requestAssignRoute_pose)
{
  EXPECT_THROW(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(),
        0.0, entity_name, traffic_simulator_msgs::msg::EntityType::MISC_OBJECT),
      hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .requestAssignRoute(
        std::vector<geometry_msgs::msg::Pose>{makePose(makePoint(3759.34, 73791.38))}),
    common::SemanticError);
}

/**
 * @note Test function behavior when position acquiring is requested with lanelet pose
 * - the goal is to test throwing error.
 */
TEST_F(MiscObjectEntityTest_HdMapUtils, requestAcquirePosition_laneletPose)
{
  EXPECT_THROW(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(),
        0.0, entity_name, traffic_simulator_msgs::msg::EntityType::MISC_OBJECT),
      hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .requestAcquirePosition(makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120660)),
    common::SemanticError);
}

/**
 * @note Test function behavior when position acquiring is requested with pose
 * - the goal is to test throwing error.
 */
TEST_F(MiscObjectEntityTest_HdMapUtils, requestAcquirePosition_pose)
{
  EXPECT_THROW(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(),
        0.0, entity_name, traffic_simulator_msgs::msg::EntityType::MISC_OBJECT),
      hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .requestAcquirePosition(makePose(makePoint(3759.34, 73791.38))),
    common::SemanticError);
}

/**
 * @note Test function behavior when called with any argument - the goal is to test error throwing.
 */
TEST_F(MiscObjectEntityTest_HdMapUtils, getRouteLanelets)
{
  EXPECT_THROW(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 120659), makeBoundingBox(),
        0.0, entity_name, traffic_simulator_msgs::msg::EntityType::MISC_OBJECT),
      hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .getRouteLanelets(100.0),
    common::SemanticError);
}

/**
 * @note Test basic functionality; test whether the function does nothing.
 */
TEST_F(MiscObjectEntityTest_FullObject, appendDebugMarker)
{
  visualization_msgs::msg::MarkerArray markers{};

  {
    auto markers_copy = markers;
    misc_object.appendDebugMarker(markers);
    EXPECT_EQ(markers.markers.size(), markers_copy.markers.size());
  }

  markers.markers.push_back([] {
    auto marker = visualization_msgs::msg::Marker{};
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.pose.position.x = 10.0;
    marker.pose.position.y = 10.0;
    return marker;
  }());

  {
    auto markers_copy = markers;
    misc_object.appendDebugMarker(markers);
    EXPECT_EQ(markers.markers.size(), markers_copy.markers.size());
  }
}

/**
 * @note Test basic functionality; test whether the function throws an error.
 */
TEST_F(MiscObjectEntityTest_FullObject, asFieldOperatorApplication)
{
  EXPECT_THROW(misc_object.asFieldOperatorApplication(), common::Error);
}

/**
 * @note Test functionality used by other units; test correctness of 2d polygon calculations.
 */
TEST_F(MiscObjectEntityTest_FullObject, get2DPolygon)
{
  const auto polygon = misc_object.get2DPolygon();

  std::vector<geometry_msgs::msg::Point> ref_poly{
    makePoint(-1.0, -1.0), makePoint(-1.0, 1.0), makePoint(3.0, 1.0), makePoint(3.0, -1.0),
    makePoint(-1.0, -1.0)};

  EXPECT_EQ(polygon.size(), ref_poly.size());
  EXPECT_POINT_EQ(polygon.at(0), ref_poly.at(0));
  EXPECT_POINT_EQ(polygon.at(1), ref_poly.at(1));
  EXPECT_POINT_EQ(polygon.at(2), ref_poly.at(2));
  EXPECT_POINT_EQ(polygon.at(3), ref_poly.at(3));
  EXPECT_POINT_EQ(polygon.at(4), ref_poly.at(4));
}

/**
 * @note Test basic functionality; test activating an out of range job with
 * an entity that has a positive speed and a speed range specified in the job = [0, 0]
 */
TEST_F(MiscObjectEntityTest_FullObject, activateOutOfRangeJob_speed)
{
  misc_object.setLinearVelocity(1.0);
  misc_object.activateOutOfRangeJob(0.0, 0.0, -100.0, 100.0, -100.0, 100.0);

  constexpr double current_time = 0.0;
  constexpr double step_time = 0.0;
  EXPECT_NO_THROW(misc_object.EntityBase::onUpdate(current_time, step_time));
  EXPECT_THROW(misc_object.onPostUpdate(current_time, step_time), common::Error);
}

/**
 * @note Test basic functionality; test activating an out of range job
 * with an entity that has a positive acceleration
 * and an acceleration range specified in the job = [0, 0].
 */
TEST_F(MiscObjectEntityTest_FullObject, activateOutOfRangeJob_acceleration)
{
  misc_object.setLinearAcceleration(1.0);
  misc_object.activateOutOfRangeJob(-100.0, 100.0, 0.0, 0.0, -100.0, 100.0);

  constexpr double current_time = 0.0;
  constexpr double step_time = 0.0;
  EXPECT_NO_THROW(misc_object.EntityBase::onUpdate(current_time, step_time));
  EXPECT_THROW(misc_object.onPostUpdate(current_time, step_time), common::Error);
}

/**
 * @note Test basic functionality; test activating an out of range job
 * with an entity that has a positive jerk
 * and a jerk range specified in the job = [0, 0].
 */
TEST_F(MiscObjectEntityTest_FullObject, activateOutOfRangeJob_jerk)
{
  misc_object.setLinearJerk(1.0);
  misc_object.activateOutOfRangeJob(-100.0, 100.0, -100.0, 100.0, 0.0, 0.0);

  constexpr double current_time = 0.0;
  constexpr double step_time = 0.0;
  EXPECT_NO_THROW(misc_object.EntityBase::onUpdate(current_time, step_time));
  EXPECT_THROW(misc_object.onPostUpdate(current_time, step_time), common::Error);
}

/**
 * @note Test basic functionality; test wrapper function with invalid relative target lanelet pose.
 */
TEST_F(MiscObjectEntityTest_FullObject, requestLaneChange_relativeTargetLaneletPose)
{
  const std::string target_name = "target_name";

  auto other_status =
    std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus>{};
  other_status.emplace(
    target_name,
    makeCanonicalizedEntityStatus(hdmap_utils_ptr, makePose(makePoint(3810.0, 73745.0)), bbox));

  entity_base->setOtherStatus(other_status);

  EXPECT_THROW(entity_base->requestLaneChange(
    traffic_simulator::lane_change::RelativeTarget(
      target_name, traffic_simulator::lane_change::Direction::STRAIGHT, 3, 1.0),
    traffic_simulator::lane_change::TrajectoryShape::LINEAR,
    traffic_simulator::lane_change::Constraint(
      traffic_simulator::lane_change::Constraint::Type::TIME, 30.0,
      traffic_simulator::lane_change::Constraint::Policy::BEST_EFFORT));
               , common::Error);
}

/**
 * @note Test basic functionality; test wrapper function with invalid relative target name.
 */
TEST_F(MiscObjectEntityTest_FullObject, requestLaneChange_relativeTargetName)
{
  const std::string target_name = "target_name";

  auto other_status =
    std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus>{};
  other_status.emplace(
    target_name,
    makeCanonicalizedEntityStatus(
      hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 34468, 5.0), bbox));

  entity_base->setOtherStatus(other_status);
  EXPECT_THROW(
    entity_base->requestLaneChange(
      traffic_simulator::lane_change::RelativeTarget(
        target_name + "_wrong", traffic_simulator::lane_change::Direction::STRAIGHT, 3, 1.0),
      traffic_simulator::lane_change::TrajectoryShape::LINEAR,
      traffic_simulator::lane_change::Constraint(
        traffic_simulator::lane_change::Constraint::Type::TIME, 30.0,
        traffic_simulator::lane_change::Constraint::Policy::BEST_EFFORT)),
    common::Error);
}

/**
 * @note Test basic functionality; test wrapper function with invalid relative target lane change
 * - the goal is to request a lane change in the location where the lane change is impossible.
 */
TEST_F(MiscObjectEntityTest_FullObject, requestLaneChange_relativeTargetInvalid)
{
  const std::string target_name = "target_name";

  auto other_status =
    std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus>{};
  other_status.emplace(
    target_name,
    makeCanonicalizedEntityStatus(
      hdmap_utils_ptr, makeCanonicalizedLaneletPose(hdmap_utils_ptr, 34468, 5.0), bbox));

  entity_base->setOtherStatus(other_status);
  EXPECT_THROW(
    entity_base->requestLaneChange(
      traffic_simulator::lane_change::RelativeTarget(
        target_name, traffic_simulator::lane_change::Direction::RIGHT,
        std::numeric_limits<std::uint8_t>::max(), 1.0),
      traffic_simulator::lane_change::TrajectoryShape::LINEAR,
      traffic_simulator::lane_change::Constraint(
        traffic_simulator::lane_change::Constraint::Type::TIME, 30.0,
        traffic_simulator::lane_change::Constraint::Policy::BEST_EFFORT)),
    common::Error);
}

/**
 * @note Test function behavior when called with any argument - the goal is to test error throwing.
 */
TEST_F(MiscObjectEntityTest_FullObject, requestFollowTrajectory)
{
  EXPECT_THROW(
    misc_object.requestFollowTrajectory(
      std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>()),
    common::Error);
}

/**
 * @note Test function behavior when called with any argument - the goal is to test error throwing.
 */
TEST_F(MiscObjectEntityTest_FullObject, requestWalkStraight)
{
  EXPECT_THROW(misc_object.requestWalkStraight(), common::Error);
}

/**
 * @note test basic functionality; test updating stand still duration
 * when NPC logic is started and velocity is greater than 0.
 */
TEST_F(MiscObjectEntityTest_FullObject, updateStandStillDuration_startedMoving)
{
  misc_object.setLinearVelocity(3.0);

  EXPECT_EQ(0.0, misc_object.updateStandStillDuration(0.1));
}

/**
 * @note Test basic functionality; test updating traveled distance correctness
 * with NPC logic started and velocity greater than 0.
 */
TEST_F(MiscObjectEntityTest_FullObject, updateTraveledDistance_startedMoving)
{
  constexpr double velocity = 3.0;
  constexpr double step_time = 0.1;
  misc_object.setLinearVelocity(velocity);

  EXPECT_EQ(1.0 * step_time * velocity, misc_object.updateTraveledDistance(step_time));
  EXPECT_EQ(2.0 * step_time * velocity, misc_object.updateTraveledDistance(step_time));
  EXPECT_EQ(3.0 * step_time * velocity, misc_object.updateTraveledDistance(step_time));
  EXPECT_EQ(4.0 * step_time * velocity, misc_object.updateTraveledDistance(step_time));
}

/**
 * @note Test basic functionality; test stopping correctness - the goal
 * is to check whether the entity status is changed to stopped (no velocity etc.).
 */
TEST_F(MiscObjectEntityTest_FullObject, stopAtCurrentPosition)
{
  constexpr double velocity = 3.0;
  misc_object.setLinearVelocity(velocity);
  EXPECT_EQ(misc_object.getCurrentTwist().linear.x, velocity);

  misc_object.stopAtCurrentPosition();
  EXPECT_EQ(misc_object.getCurrentTwist().linear.x, 0.0);
}

/**
 * @note Test functionality used by other units; test lanelet pose obtaining
 * with a matching distance smaller than a distance from an entity to the lanelet
 * (both crosswalk and road) and status_.type.type != PEDESTRIAN.
 */
TEST_F(
  MiscObjectEntityTest_HdMapUtils, getCanonicalizedLaneletPose_notOnRoadAndCrosswalkNotPedestrian)
{
  EXPECT_FALSE(traffic_simulator::entity::MiscObjectEntity(
                 entity_name,
                 makeCanonicalizedEntityStatus(
                   hdmap_utils_ptr, makePose(makePoint(3810.0, 73745.0)), makeBoundingBox(), 1.0,
                   0.0, entity_name, traffic_simulator_msgs::msg::EntityType::MISC_OBJECT),
                 hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
                 .getCanonicalizedLaneletPose(5.0)
                 .has_value());
}

/**
 * @note Test functionality used by other units; test lanelet pose obtaining
 * with a matching distance greater than a distance from an entity to the lanelet
 * (both crosswalk and road) and status_.type.type != PEDESTRIAN.
 */
TEST_F(MiscObjectEntityTest_HdMapUtils, getCanonicalizedLaneletPose_onRoadAndCrosswalkNotPedestrian)
{
  EXPECT_TRUE(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr,
        makePose(makePoint(3766.1, 73738.2), makeQuaternionFromYaw((120.0) * M_PI / 180.0)),
        makeBoundingBox(), 1.0, 0.0, entity_name,
        traffic_simulator_msgs::msg::EntityType::MISC_OBJECT),
      hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .getCanonicalizedLaneletPose(1.0)
      .has_value());
}

/**
 * @note Test functionality used by other units; test lanelet pose obtaining
 * with a matching distance greater than a distance from an entity to the crosswalk lanelet,
 * but smaller than to the road lanelet and status_.type.type != PEDESTRIAN.
 */
TEST_F(
  MiscObjectEntityTest_HdMapUtils, getCanonicalizedLaneletPose_onCrosswalkNotOnRoadNotPedestrian)
{
  EXPECT_FALSE(
    traffic_simulator::entity::MiscObjectEntity(
      entity_name,
      makeCanonicalizedEntityStatus(
        hdmap_utils_ptr,
        makePose(makePoint(3764.5, 73737.5), makeQuaternionFromYaw((120.0) * M_PI / 180.0)),
        makeBoundingBox(), 1.0, 0.0, entity_name,
        traffic_simulator_msgs::msg::EntityType::MISC_OBJECT),
      hdmap_utils_ptr, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .getCanonicalizedLaneletPose(1.0)
      .has_value());
}
