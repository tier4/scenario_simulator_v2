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

  visualization_msgs::msg::Marker marker{};
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.pose.position.x = 10.0;
  marker.pose.position.y = 10.0;

  markers.markers.push_back(marker);

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

  std::vector<geometry_msgs::msg::Point> ref_poly{};

  ref_poly.push_back(makePoint(-1.0, -1.0));
  ref_poly.push_back(makePoint(-1.0, 1.0));
  ref_poly.push_back(makePoint(3.0, 1.0));
  ref_poly.push_back(makePoint(3.0, -1.0));
  ref_poly.push_back(ref_poly.front());

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
  double min_velocity = 0.0;
  double max_velocity = 0.0;
  double min_acceleration = -100.0;
  double max_acceleration = 100.0;
  double min_jerk = -100.0;
  double max_jerk = 100.0;
  double velocity = 1.0;
  dummy.setLinearVelocity(velocity);
  dummy.activateOutOfRangeJob(
    min_velocity, max_velocity, min_acceleration, max_acceleration, min_jerk, max_jerk);
  double current_time = 0.0;
  double step_time = 0.0;
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
  double min_velocity = -100.0;
  double max_velocity = 100.0;
  double min_acceleration = 0.0;
  double max_acceleration = 0.0;
  double min_jerk = -100.0;
  double max_jerk = 100.0;
  double acceleration = 1.0;
  dummy.setLinearAcceleration(acceleration);
  dummy.activateOutOfRangeJob(
    min_velocity, max_velocity, min_acceleration, max_acceleration, min_jerk, max_jerk);
  double current_time = 0.0;
  double step_time = 0.0;
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
  double min_velocity = -100.0;
  double max_velocity = 100.0;
  double min_acceleration = -100.0;
  double max_acceleration = 100.0;
  double min_jerk = 0.0;
  double max_jerk = 0.0;
  double jerk = 1.0;
  dummy.setLinearJerk(jerk);
  dummy.activateOutOfRangeJob(
    min_velocity, max_velocity, min_acceleration, max_acceleration, min_jerk, max_jerk);
  double current_time = 0.0;
  double step_time = 0.0;
  EXPECT_NO_THROW(dummy.EntityBase::onUpdate(current_time, step_time));
  EXPECT_THROW(dummy.onPostUpdate(current_time, step_time), common::Error);
}

/**
 * @note Test basic functionality; test wrapper function with invalid relative target lanelet pose.
 */
TEST_F(EntityBaseWithMiscObjectTest, requestLaneChange_relativeTargetLaneletPose)
{
  const std::string target_name = "target_name";
  geometry_msgs::msg::Pose target_pose;
  target_pose.position = makePoint(3810.0, 73745.0);  // outside of road

  auto target_status = makeCanonicalizedEntityStatus(hdmap_utils, target_pose, bbox);

  const double target_offset = 1.0;
  traffic_simulator::lane_change::RelativeTarget target(
    target_name, traffic_simulator::lane_change::Direction::STRAIGHT, 3, target_offset);

  traffic_simulator::lane_change::TrajectoryShape trajectory_shape =
    traffic_simulator::lane_change::TrajectoryShape::LINEAR;

  traffic_simulator::lane_change::Constraint constraint(
    traffic_simulator::lane_change::Constraint::Type::TIME, 30.0,
    traffic_simulator::lane_change::Constraint::Policy::BEST_EFFORT);

  std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus> other_status;
  other_status.emplace(target_name, target_status);

  dummy_base->setOtherStatus(other_status);

  EXPECT_THROW(dummy_base->requestLaneChange(target, trajectory_shape, constraint);, common::Error);
}

/**
 * @note Test basic functionality; test wrapper function with invalid relative target name.
 */
TEST_F(EntityBaseWithMiscObjectTest, requestLaneChange_relativeTargetName)
{
  const lanelet::Id target_id = 34468;
  const std::string target_name = "target_name";
  auto target_pose = makeCanonicalizedLaneletPose(hdmap_utils, target_id, 5.0);
  auto target_status = makeCanonicalizedEntityStatus(hdmap_utils, target_pose, bbox);

  const double target_offset = 1.0;
  traffic_simulator::lane_change::RelativeTarget target(
    target_name + "_wrong", traffic_simulator::lane_change::Direction::STRAIGHT, 3, target_offset);

  traffic_simulator::lane_change::TrajectoryShape trajectory_shape =
    traffic_simulator::lane_change::TrajectoryShape::LINEAR;

  traffic_simulator::lane_change::Constraint constraint(
    traffic_simulator::lane_change::Constraint::Type::TIME, 30.0,
    traffic_simulator::lane_change::Constraint::Policy::BEST_EFFORT);

  std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus> other_status;
  other_status.emplace(target_name, target_status);

  dummy_base->setOtherStatus(other_status);
  EXPECT_THROW(dummy_base->requestLaneChange(target, trajectory_shape, constraint);, common::Error);
}

/**
 * @note Test basic functionality; test wrapper function with invalid relative target lane change
 * - the goal is to request a lane change in the location where the lane change is impossible.
 */
TEST_F(EntityBaseWithMiscObjectTest, requestLaneChange_relativeTargetInvalid)
{
  const lanelet::Id target_id = 34468;
  const std::string target_name = "target_name";
  auto target_pose = makeCanonicalizedLaneletPose(hdmap_utils, target_id, 5.0);
  auto target_status = makeCanonicalizedEntityStatus(hdmap_utils, target_pose, bbox);

  const double target_offset = 1.0;
  traffic_simulator::lane_change::RelativeTarget target(
    target_name, traffic_simulator::lane_change::Direction::RIGHT,
    std::numeric_limits<std::uint8_t>::max(), target_offset);

  traffic_simulator::lane_change::TrajectoryShape trajectory_shape =
    traffic_simulator::lane_change::TrajectoryShape::LINEAR;

  traffic_simulator::lane_change::Constraint constraint(
    traffic_simulator::lane_change::Constraint::Type::TIME, 30.0,
    traffic_simulator::lane_change::Constraint::Policy::BEST_EFFORT);

  std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus> other_status;
  other_status.emplace(target_name, target_status);

  dummy_base->setOtherStatus(other_status);
  EXPECT_THROW(dummy_base->requestLaneChange(target, trajectory_shape, constraint), common::Error);
}

/**
 * @note Test function behavior when called with any argument - the goal is to test error throwing.
 */
TEST_F(EntityBaseWithMiscObjectTest, requestFollowTrajectory)
{
  auto ptr = std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>();
  EXPECT_THROW(dummy.requestFollowTrajectory(ptr), common::Error);
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
  double velocity = 3.0;
  double step_time = 0.1;
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
  double velocity = 3.0;
  double step_time = 0.1;
  dummy.setLinearVelocity(velocity);
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
  double velocity = 3.0;
  dummy.setLinearVelocity(velocity);
  auto curr_twist = dummy.getCurrentTwist();
  EXPECT_EQ(curr_twist.linear.x, velocity);

  dummy.stopAtCurrentPosition();
  curr_twist = dummy.getCurrentTwist();
  EXPECT_EQ(curr_twist.linear.x, 0.0);
}

/**
 * @note Test functionality used by other units; test lanelet pose obtaining
 * with a matching distance smaller than a distance from an entity to the lanelet
 * (both crosswalk and road) and status_.type.type = PEDESTRIAN.
 */
TEST(EntityBaseWithMiscObject, getLaneletPose_notOnRoadAndCrosswalkPedestrian)
{
  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto bbox = makeBoundingBox();

  geometry_msgs::msg::Pose pose;
  pose.position.x = 3810.0;
  pose.position.y = 73745.0;

  auto status_base = makeEntityStatus(hdmap_utils, pose, bbox);
  status_base.type.type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
  traffic_simulator::CanonicalizedEntityStatus status(status_base, hdmap_utils);

  auto dummy = traffic_simulator::entity::MiscObjectEntity(
    "dummy_entity", status, hdmap_utils, traffic_simulator_msgs::msg::MiscObjectParameters{});

  auto lanelet_pose = dummy.getLaneletPose(5.0);
  EXPECT_FALSE(lanelet_pose);
}

/**
 * @note Test functionality used by other units; test lanelet pose obtaining
 * with a matching distance smaller than a distance from an entity to the lanelet
 * (both crosswalk and road) and status_.type.type != PEDESTRIAN.
 */
TEST(EntityBaseWithMiscObject, getLaneletPose_notOnRoadAndCrosswalkNotPedestrian)
{
  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto bbox = makeBoundingBox();

  geometry_msgs::msg::Pose pose;
  pose.position.x = 3810.0;
  pose.position.y = 73745.0;

  auto status_base = makeEntityStatus(hdmap_utils, pose, bbox);
  status_base.type.type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
  traffic_simulator::CanonicalizedEntityStatus status(status_base, hdmap_utils);

  auto dummy = traffic_simulator::entity::MiscObjectEntity(
    "dummy_entity", status, hdmap_utils, traffic_simulator_msgs::msg::MiscObjectParameters{});

  auto lanelet_pose = dummy.getLaneletPose(5.0);
  EXPECT_FALSE(lanelet_pose);
}

/**
 * @note Test functionality used by other units; test lanelet pose obtaining
 * with a matching distance greater than a distance from an entity to the lanelet
 * (both crosswalk and road) and status_.type.type != PEDESTRIAN.
 */
TEST(EntityBaseWithMiscObject, getLaneletPose_onRoadAndCrosswalkNotPedestrian)
{
  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto bbox = makeBoundingBox();

  geometry_msgs::msg::Pose pose;
  pose.position.x = 3766.1;
  pose.position.y = 73738.2;
  pose.orientation = makeQuaternionFromYaw((120.0) * M_PI / 180.0);

  auto status_base = makeEntityStatus(hdmap_utils, pose, bbox);
  status_base.type.type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
  traffic_simulator::CanonicalizedEntityStatus status(status_base, hdmap_utils);

  auto dummy = traffic_simulator::entity::MiscObjectEntity(
    "dummy_entity", status, hdmap_utils, traffic_simulator_msgs::msg::MiscObjectParameters{});

  auto lanelet_pose = dummy.getLaneletPose(1.0);
  EXPECT_TRUE(lanelet_pose);
}

/**
 * @note Test functionality used by other units; test lanelet pose obtaining
 * with a matching distance greater than a distance from an entity to the crosswalk lanelet,
 * but smaller than to the road lanelet and status_.type.type != PEDESTRIAN.
 */
TEST(EntityBaseWithMiscObject, getLaneletPose_onCrosswalkNotOnRoadNotPedestrian)
{
  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto bbox = makeBoundingBox();

  geometry_msgs::msg::Pose pose;
  pose.position.x = 3764.5;
  pose.position.y = 73737.5;
  pose.orientation = makeQuaternionFromYaw((120.0) * M_PI / 180.0);

  auto status_base = makeEntityStatus(hdmap_utils, pose, bbox);
  status_base.type.type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
  traffic_simulator::CanonicalizedEntityStatus status(status_base, hdmap_utils);

  auto dummy = traffic_simulator::entity::MiscObjectEntity(
    "dummy_entity", status, hdmap_utils, traffic_simulator_msgs::msg::MiscObjectParameters{});

  auto lanelet_pose = dummy.getLaneletPose(1.0);
  EXPECT_FALSE(lanelet_pose);
}

/**
 * @note Test functionality used by other units; test relative pose calculations
 * correctness with a transformation argument passed.
 */
TEST_F(EntityBaseWithMiscObjectTest, getMapPoseFromRelativePose_relative)
{
  const double s = 5.0;
  constexpr double eps = 0.1;

  geometry_msgs::msg::Pose relative_pose;
  relative_pose.position.x = s;

  auto result_pose = dummy.getMapPoseFromRelativePose(relative_pose);

  auto ref_pose = makeCanonicalizedLaneletPose(hdmap_utils, id, s);
  EXPECT_POSE_NEAR(result_pose, static_cast<geometry_msgs::msg::Pose>(ref_pose), eps);
}
