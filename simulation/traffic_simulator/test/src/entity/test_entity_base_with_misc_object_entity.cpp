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
#include <visualization_msgs/msg/marker.hpp>

#include "../catalogs.hpp"
#include "../expect_eq_macros.hpp"
#include "../helper_functions.hpp"

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class EntityBaseWithMiscObjectTest : public testing::Test
{
protected:
  EntityBaseWithMiscObjectTest()
  : id(120659),
    hdmap_utils(makeHdMapUtilsSharedPointer()),
    pose(makeCanonicalizedLaneletPose(hdmap_utils, id)),
    bbox(makeBoundingBox()),
    status(makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox)),
    dummy("dummy_entity", status, hdmap_utils, traffic_simulator_msgs::msg::MiscObjectParameters{}),
    dummy_base(&dummy)
  {
  }

  const lanelet::Id id;
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils;
  traffic_simulator::lanelet_pose::CanonicalizedLaneletPose pose;
  traffic_simulator_msgs::msg::BoundingBox bbox;
  traffic_simulator::entity_status::CanonicalizedEntityStatus status;
  traffic_simulator::entity::MiscObjectEntity dummy;
  traffic_simulator::entity::EntityBase * dummy_base;
};

/**
 * @note Test basic functionality; test whether the function does nothing.
 */
TEST_F(EntityBaseWithMiscObjectTest, appendDebugMarker)
{
  visualization_msgs::msg::MarkerArray markers{};

  {
    auto markers_copy = markers;
    dummy.appendDebugMarker(markers);
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
    dummy.appendDebugMarker(markers);
    EXPECT_EQ(markers.markers.size(), markers_copy.markers.size());
  }
}

/**
 * @note Test basic functionality; test whether the function throws an error.
 */
TEST_F(EntityBaseWithMiscObjectTest, asFieldOperatorApplication)
{
  EXPECT_THROW(dummy.asFieldOperatorApplication(), common::Error);
}

/**
 * @note Test functionality used by other units; test correctness of 2d polygon calculations.
 */
TEST_F(EntityBaseWithMiscObjectTest, get2DPolygon)
{
  const auto polygon = dummy.get2DPolygon();

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
 * @note Test basic functionality; test whether the NPC logic is started correctly.
 */
TEST_F(EntityBaseWithMiscObjectTest, startNpcLogic)
{
  EXPECT_FALSE(dummy.isNpcLogicStarted());
  dummy.startNpcLogic(0.0);
  EXPECT_TRUE(dummy.isNpcLogicStarted());
}

/**
 * @note Test basic functionality; test activating an out of range job with
 * an entity that has a positive speed and a speed range specified in the job = [0, 0]
 */
TEST_F(EntityBaseWithMiscObjectTest, activateOutOfRangeJob_speed)
{
  constexpr double velocity = 1.0;
  dummy.setLinearVelocity(velocity);
  dummy.activateOutOfRangeJob(0.0, 0.0, -100.0, 100.0, -100.0, 100.0);

  constexpr double current_time = 0.0;
  constexpr double step_time = 0.0;
  EXPECT_NO_THROW(dummy.EntityBase::onUpdate(current_time, step_time));
  EXPECT_THROW(dummy.onPostUpdate(current_time, step_time), common::Error);
}

/**
 * @note Test basic functionality; test activating an out of range job
 * with an entity that has a positive acceleration
 * and an acceleration range specified in the job = [0, 0].
 */
TEST_F(EntityBaseWithMiscObjectTest, activateOutOfRangeJob_acceleration)
{
  constexpr double acceleration = 1.0;
  dummy.setLinearAcceleration(acceleration);
  dummy.activateOutOfRangeJob(-100.0, 100.0, 0.0, 0.0, -100.0, 100.0);

  constexpr double current_time = 0.0;
  constexpr double step_time = 0.0;
  EXPECT_NO_THROW(dummy.EntityBase::onUpdate(current_time, step_time));
  EXPECT_THROW(dummy.onPostUpdate(current_time, step_time), common::Error);
}

/**
 * @note Test basic functionality; test activating an out of range job
 * with an entity that has a positive jerk
 * and a jerk range specified in the job = [0, 0].
 */
TEST_F(EntityBaseWithMiscObjectTest, activateOutOfRangeJob_jerk)
{
  constexpr double jerk = 1.0;
  dummy.setLinearJerk(jerk);
  dummy.activateOutOfRangeJob(-100.0, 100.0, -100.0, 100.0, 0.0, 0.0);

  constexpr double current_time = 0.0;
  constexpr double step_time = 0.0;
  EXPECT_NO_THROW(dummy.EntityBase::onUpdate(current_time, step_time));
  EXPECT_THROW(dummy.onPostUpdate(current_time, step_time), common::Error);
}

/**
 * @note Test basic functionality; test wrapper function with invalid relative target lanelet pose.
 */
TEST_F(EntityBaseWithMiscObjectTest, requestLaneChange_relativeTargetLaneletPose)
{
  const std::string target_name = "target_name";

  auto other_status =
    std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus>{};
  other_status.emplace(
    target_name,
    makeCanonicalizedEntityStatus(hdmap_utils, makePose(makePoint(3810.0, 73745.0)), bbox));

  dummy_base->setOtherStatus(other_status);

  EXPECT_THROW(dummy_base->requestLaneChange(
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
TEST_F(EntityBaseWithMiscObjectTest, requestLaneChange_relativeTargetName)
{
  const std::string target_name = "target_name";

  auto other_status =
    std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus>{};
  other_status.emplace(
    target_name, makeCanonicalizedEntityStatus(
                   hdmap_utils, makeCanonicalizedLaneletPose(hdmap_utils, 34468, 5.0), bbox));

  dummy_base->setOtherStatus(other_status);
  EXPECT_THROW(
    dummy_base->requestLaneChange(
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
TEST_F(EntityBaseWithMiscObjectTest, requestLaneChange_relativeTargetInvalid)
{
  const std::string target_name = "target_name";

  auto other_status =
    std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus>{};
  other_status.emplace(
    target_name, makeCanonicalizedEntityStatus(
                   hdmap_utils, makeCanonicalizedLaneletPose(hdmap_utils, 34468, 5.0), bbox));

  dummy_base->setOtherStatus(other_status);
  EXPECT_THROW(
    dummy_base->requestLaneChange(
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
TEST_F(EntityBaseWithMiscObjectTest, requestFollowTrajectory)
{
  EXPECT_THROW(
    dummy.requestFollowTrajectory(
      std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>()),
    common::Error);
}

/**
 * @note Test function behavior when called with any argument - the goal is to test error throwing.
 */
TEST_F(EntityBaseWithMiscObjectTest, requestWalkStraight)
{
  EXPECT_THROW(dummy.requestWalkStraight(), common::Error);
}

/**
 * @note test basic functionality; test updating stand still duration
 * when NPC logic is started and velocity is greater than 0.
 */
TEST_F(EntityBaseWithMiscObjectTest, updateStandStillDuration_startedMoving)
{
  dummy.startNpcLogic(0.0);
  dummy.setLinearVelocity(3.0);

  EXPECT_EQ(0.0, dummy.updateStandStillDuration(0.1));
}

/**
 * @note Test basic functionality; test updating stand still duration
 * when NPC logic is not started.
 */
TEST_F(EntityBaseWithMiscObjectTest, updateStandStillDuration_notStarted)
{
  dummy.setLinearVelocity(3.0);
  EXPECT_EQ(0.0, dummy.updateStandStillDuration(0.1));

  dummy.setLinearVelocity(0.0);
  EXPECT_EQ(0.0, dummy.updateStandStillDuration(0.1));
}

/**
 * @note Test basic functionality; test updating traveled distance correctness
 * with NPC logic started and velocity greater than 0.
 */
TEST_F(EntityBaseWithMiscObjectTest, updateTraveledDistance_startedMoving)
{
  constexpr double velocity = 3.0;
  constexpr double step_time = 0.1;
  dummy.startNpcLogic(0.0);
  dummy.setLinearVelocity(velocity);

  EXPECT_EQ(1.0 * step_time * velocity, dummy.updateTraveledDistance(step_time));
  EXPECT_EQ(2.0 * step_time * velocity, dummy.updateTraveledDistance(step_time));
  EXPECT_EQ(3.0 * step_time * velocity, dummy.updateTraveledDistance(step_time));
  EXPECT_EQ(4.0 * step_time * velocity, dummy.updateTraveledDistance(step_time));
}

/**
 * @note Test basic functionality; test updating traveled distance correctness with NPC not started.
 */
TEST_F(EntityBaseWithMiscObjectTest, updateTraveledDistance_notStarted)
{
  constexpr double step_time = 0.1;
  dummy.setLinearVelocity(3.0);
  EXPECT_EQ(0.0, dummy.updateTraveledDistance(step_time));

  dummy.setLinearVelocity(0.0);
  EXPECT_EQ(0.0, dummy.updateTraveledDistance(step_time));
}

/**
 * @note Test basic functionality; test stopping correctness - the goal
 * is to check whether the entity status is changed to stopped (no velocity etc.).
 */
TEST_F(EntityBaseWithMiscObjectTest, stopAtCurrentPosition)
{
  constexpr double velocity = 3.0;
  dummy.setLinearVelocity(velocity);
  EXPECT_EQ(dummy.getCurrentTwist().linear.x, velocity);

  dummy.stopAtCurrentPosition();
  EXPECT_EQ(dummy.getCurrentTwist().linear.x, 0.0);
}

/**
 * @note Test functionality used by other units; test lanelet pose obtaining
 * with a matching distance smaller than a distance from an entity to the lanelet
 * (both crosswalk and road) and status_.type.type != PEDESTRIAN.
 */
TEST(EntityBaseWithMiscObject, getLaneletPose_notOnRoadAndCrosswalkNotPedestrian)
{
  auto hdmap_utils = makeHdMapUtilsSharedPointer();

  EXPECT_FALSE(traffic_simulator::entity::MiscObjectEntity(
                 "dummy_entity",
                 traffic_simulator::CanonicalizedEntityStatus(
                   makeEntityStatus(
                     hdmap_utils, makePose(makePoint(3810.0, 73745.0)), makeBoundingBox(), 0.0,
                     "dummy_entity", traffic_simulator_msgs::msg::EntityType::MISC_OBJECT),
                   hdmap_utils),
                 hdmap_utils, traffic_simulator_msgs::msg::MiscObjectParameters{})
                 .getLaneletPose(5.0)
                 .has_value());
}

/**
 * @note Test functionality used by other units; test lanelet pose obtaining
 * with a matching distance greater than a distance from an entity to the lanelet
 * (both crosswalk and road) and status_.type.type != PEDESTRIAN.
 */
TEST(EntityBaseWithMiscObject, getLaneletPose_onRoadAndCrosswalkNotPedestrian)
{
  auto hdmap_utils = makeHdMapUtilsSharedPointer();

  EXPECT_TRUE(
    traffic_simulator::entity::MiscObjectEntity(
      "dummy_entity",
      traffic_simulator::CanonicalizedEntityStatus(
        makeEntityStatus(
          hdmap_utils,
          makePose(makePoint(3766.1, 73738.2), makeQuaternionFromYaw((120.0) * M_PI / 180.0)),
          makeBoundingBox(), 0.0, "dummy_entity",
          traffic_simulator_msgs::msg::EntityType::MISC_OBJECT),
        hdmap_utils),
      hdmap_utils, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .getLaneletPose(1.0)
      .has_value());
}

/**
 * @note Test functionality used by other units; test lanelet pose obtaining
 * with a matching distance greater than a distance from an entity to the crosswalk lanelet,
 * but smaller than to the road lanelet and status_.type.type != PEDESTRIAN.
 */
TEST(EntityBaseWithMiscObject, getLaneletPose_onCrosswalkNotOnRoadNotPedestrian)
{
  auto hdmap_utils = makeHdMapUtilsSharedPointer();

  EXPECT_FALSE(
    traffic_simulator::entity::MiscObjectEntity(
      "dummy_entity",
      traffic_simulator::CanonicalizedEntityStatus(
        makeEntityStatus(
          hdmap_utils,
          makePose(makePoint(3764.5, 73737.5), makeQuaternionFromYaw((120.0) * M_PI / 180.0)),
          makeBoundingBox(), 0.0, "dummy_entity",
          traffic_simulator_msgs::msg::EntityType::MISC_OBJECT),
        hdmap_utils),
      hdmap_utils, traffic_simulator_msgs::msg::MiscObjectParameters{})
      .getLaneletPose(1.0)
      .has_value());
}

/**
 * @note Test functionality used by other units; test relative pose calculations
 * correctness with a transformation argument passed.
 */
TEST_F(EntityBaseWithMiscObjectTest, getMapPoseFromRelativePose_relative)
{
  constexpr double s = 5.0;
  EXPECT_POSE_NEAR(
    dummy.getMapPoseFromRelativePose(makePose(makePoint(s, 0.0))),
    static_cast<geometry_msgs::msg::Pose>(makeCanonicalizedLaneletPose(hdmap_utils, id, s)), 0.1);
}
