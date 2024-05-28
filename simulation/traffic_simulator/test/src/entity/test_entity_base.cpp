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
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/helper/helper.hpp>

#include "../catalogs.hpp"
#include "../entity_helper_functions.hpp"
#include "../expect_eq_macros.hpp"
#include "dummy_entity.hpp"

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

class EntityBaseTest : public testing::Test
{
protected:
  EntityBaseTest()
  : id(120659),
    hdmap_utils(makeHdMapUtilsSharedPointer()),
    pose(makeCanonicalizedLaneletPose(hdmap_utils, id)),
    bbox(makeBoundingBox()),
    status(makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox)),
    dummy("dummy_entity", status, hdmap_utils),
    dummy_base(&dummy)
  {
  }

  const lanelet::Id id;
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils;
  traffic_simulator::lanelet_pose::CanonicalizedLaneletPose pose;
  traffic_simulator_msgs::msg::BoundingBox bbox;
  traffic_simulator::entity_status::CanonicalizedEntityStatus status;
  DummyEntity dummy;
  traffic_simulator::entity::EntityBase * dummy_base;
};

TEST_F(EntityBaseTest, appendDebugMarker)
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

TEST_F(EntityBaseTest, asFieldOperatorApplication)
{
  EXPECT_THROW(dummy.asFieldOperatorApplication(), common::Error);
}

TEST_F(EntityBaseTest, get2DPolygon)
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

TEST_F(EntityBaseTest, startNpcLogic)
{
  EXPECT_FALSE(dummy.isNpcLogicStarted());
  dummy.startNpcLogic();
  EXPECT_TRUE(dummy.isNpcLogicStarted());
}

TEST_F(EntityBaseTest, activateOutOfRangeJob_speed)
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
  EXPECT_NO_THROW(dummy.onUpdate(current_time, step_time));
  EXPECT_THROW(dummy.onPostUpdate(current_time, step_time), common::Error);
}

TEST_F(EntityBaseTest, activateOutOfRangeJob_acceleration)
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
  EXPECT_NO_THROW(dummy.onUpdate(current_time, step_time));
  EXPECT_THROW(dummy.onPostUpdate(current_time, step_time), common::Error);
}

TEST_F(EntityBaseTest, activateOutOfRangeJob_jerk)
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
  EXPECT_NO_THROW(dummy.onUpdate(current_time, step_time));
  EXPECT_THROW(dummy.onPostUpdate(current_time, step_time), common::Error);
}

TEST_F(EntityBaseTest, onUpdate)
{
  bool first_cleanup = false;
  bool first_update = false;
  bool second_cleanup = false;
  bool second_update = false;

  auto first_update_func = [&first_update](const double) { return first_update = true; };
  auto first_cleanup_func = [&first_cleanup]() { first_cleanup = true; };
  auto second_update_func = [&second_update](const double) { return second_update = true; };
  auto second_cleanup_func = [&second_cleanup]() { second_cleanup = true; };

  auto type_first = traffic_simulator::job::Type::LINEAR_VELOCITY;
  auto type_second = traffic_simulator::job::Type::LINEAR_ACCELERATION;
  auto first_event = traffic_simulator::job::Event::PRE_UPDATE;
  auto second_event = traffic_simulator::job::Event::POST_UPDATE;
  auto is_exclusive = true;

  dummy.appendToJobList(
    first_update_func, first_cleanup_func, type_first, is_exclusive, first_event);
  dummy.appendToJobList(
    second_update_func, second_cleanup_func, type_second, is_exclusive, second_event);

  double current_time = 0.0;
  double step_time = 0.0;
  dummy.onUpdate(current_time, step_time);

  EXPECT_TRUE(first_cleanup);
  EXPECT_TRUE(first_update);
  EXPECT_FALSE(second_cleanup);
  EXPECT_FALSE(second_update);
}

TEST_F(EntityBaseTest, onPostUpdate)
{
  bool first_cleanup = false;
  bool first_update = false;
  bool second_cleanup = false;
  bool second_update = false;

  auto first_update_func = [&first_update](const double) { return first_update = true; };
  auto first_cleanup_func = [&first_cleanup]() { first_cleanup = true; };
  auto second_update_func = [&second_update](const double) { return second_update = true; };
  auto second_cleanup_func = [&second_cleanup]() { second_cleanup = true; };

  auto type_first = traffic_simulator::job::Type::LINEAR_VELOCITY;
  auto type_second = traffic_simulator::job::Type::LINEAR_ACCELERATION;
  auto first_event = traffic_simulator::job::Event::PRE_UPDATE;
  auto second_event = traffic_simulator::job::Event::POST_UPDATE;
  auto is_exclusive = true;

  dummy.appendToJobList(
    first_update_func, first_cleanup_func, type_first, is_exclusive, first_event);
  dummy.appendToJobList(
    second_update_func, second_cleanup_func, type_second, is_exclusive, second_event);

  double current_time = 0.0;
  double step_time = 0.0;
  dummy.onPostUpdate(current_time, step_time);

  EXPECT_FALSE(first_cleanup);
  EXPECT_FALSE(first_update);
  EXPECT_TRUE(second_cleanup);
  EXPECT_TRUE(second_update);
}

TEST_F(EntityBaseTest, resetDynamicConstraints)
{
  auto default_constraints = dummy.getDefaultDynamicConstraints();
  dummy.resetDynamicConstraints();
  auto current_constraints = dummy.getDynamicConstraints();

  EXPECT_DYNAMIC_CONSTRAINTS_EQ(default_constraints, current_constraints);
}

TEST_F(EntityBaseTest, requestLaneChange_absoluteTarget)
{
  traffic_simulator::lane_change::AbsoluteTarget target(id, 1.0);

  traffic_simulator::lane_change::TrajectoryShape trajectory_shape =
    traffic_simulator::lane_change::TrajectoryShape::LINEAR;

  traffic_simulator::lane_change::Constraint constraint(
    traffic_simulator::lane_change::Constraint::Type::TIME, 30.0,
    traffic_simulator::lane_change::Constraint::Policy::BEST_EFFORT);

  dummy_base->requestLaneChange(target, trajectory_shape, constraint);
  auto result_param = dummy.getLaneChangeParameter();

  EXPECT_LANE_CHANGE_ABSOLUTE_TARGET_EQ(result_param.target, target);
  EXPECT_EQ(result_param.trajectory_shape, trajectory_shape);
  EXPECT_LANE_CHANGE_CONSTRAINT_EQ(result_param.constraint, constraint);
}

TEST_F(EntityBaseTest, requestLaneChange_relativeTarget)
{
  const lanelet::Id target_id = 34468;
  const std::string target_name = "target_name";
  auto target_pose = makeCanonicalizedLaneletPose(hdmap_utils, target_id, 5.0);
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

  dummy_base->requestLaneChange(target, trajectory_shape, constraint);
  auto result_param = dummy.getLaneChangeParameter();

  traffic_simulator::lane_change::AbsoluteTarget ref_target(target_id, target_offset);

  EXPECT_LANE_CHANGE_ABSOLUTE_TARGET_EQ(result_param.target, ref_target);
  EXPECT_EQ(result_param.trajectory_shape, trajectory_shape);
  EXPECT_LANE_CHANGE_CONSTRAINT_EQ(result_param.constraint, constraint);
}

TEST_F(EntityBaseTest, requestLaneChange_relativeTargetLaneletPose)
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

TEST_F(EntityBaseTest, requestLaneChange_relativeTargetName)
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

TEST_F(EntityBaseTest, requestLaneChange_relativeTargetInvalid)
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

TEST_F(EntityBaseTest, setDynamicConstraints)
{
  traffic_simulator_msgs::msg::DynamicConstraints custom_constraints{};

  custom_constraints.max_speed = 5.0;
  custom_constraints.max_acceleration = 7.0;
  custom_constraints.max_deceleration = 11.0;
  custom_constraints.max_acceleration_rate = 13.0;
  custom_constraints.max_deceleration_rate = 17.0;
  dummy.setDynamicConstraints(custom_constraints);
  auto result_constraints = dummy.getDynamicConstraints();

  EXPECT_DYNAMIC_CONSTRAINTS_EQ(custom_constraints, result_constraints);
}

TEST_F(EntityBaseTest, setOtherStatus)
{
  const std::string name0 = "other_entity0", name1 = "other_entity1";

  std::unordered_map<std::string, traffic_simulator::CanonicalizedEntityStatus> other_status;
  {
    auto pose = makeCanonicalizedLaneletPose(hdmap_utils, id, 5.0);
    auto status = makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox, 0.0, name0);
    other_status.emplace(name0, status);
  }
  {
    geometry_msgs::msg::Pose pose;
    pose.position = makePoint(3810.0, 73745.0);  // outside of road
    auto status = makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox, 0.0, name1);
    other_status.emplace(name1, status);
  }

  dummy_base->setOtherStatus(other_status);

  const auto & result_status = dummy.getOtherStatus();

  EXPECT_EQ(other_status.size(), result_status.size());
  EXPECT_EQ(
    static_cast<traffic_simulator_msgs::msg::EntityStatus>(other_status.at(name0)),
    static_cast<traffic_simulator_msgs::msg::EntityStatus>(result_status.at(name0)));
  EXPECT_EQ(
    static_cast<traffic_simulator_msgs::msg::EntityStatus>(other_status.at(name1)),
    static_cast<traffic_simulator_msgs::msg::EntityStatus>(result_status.at(name1)));
}

TEST_F(EntityBaseTest, setStatus)
{
  dummy.setEntityType(traffic_simulator_msgs::msg::EntityType::VEHICLE);

  auto new_type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
  auto new_subtype = traffic_simulator_msgs::msg::EntitySubtype::CAR;
  double new_time = 1.0;
  const std::string new_name = "dummy_entity_new";
  traffic_simulator_msgs::msg::BoundingBox new_bounding_box;
  new_bounding_box.center.x = 2.0;
  new_bounding_box.dimensions.x = 3.0;
  new_bounding_box.dimensions.y = 1.5;
  traffic_simulator_msgs::msg::ActionStatus new_action_status;
  new_action_status.twist.linear.x = 100.0;
  new_action_status.linear_jerk = 20.0;
  new_action_status.current_action = "new_current_status";
  geometry_msgs::msg::Pose new_pose;
  new_pose.position = makePoint(3810.0, 73745.0);  // outside of road
  bool new_lanelet_pose_valid = false;

  traffic_simulator_msgs::msg::EntityStatus new_status;
  {
    new_status.type.type = new_type;
    new_status.subtype.value = new_subtype;
    new_status.time = new_time;
    new_status.name = new_name;
    new_status.bounding_box = new_bounding_box;
    new_status.action_status = new_action_status;
    new_status.pose = new_pose;
    new_status.lanelet_pose_valid = new_lanelet_pose_valid;
  }

  auto ref_status = static_cast<traffic_simulator_msgs::msg::EntityStatus>(dummy_base->getStatus());
  {
    ref_status.time = new_time;
    ref_status.action_status = [&]() -> traffic_simulator_msgs::msg::ActionStatus {
      auto tmp_action = ref_status.action_status.current_action;
      auto ret_status = new_action_status;
      ret_status.current_action = tmp_action;
      return ret_status;
    }();
    ref_status.pose = new_pose;
    ref_status.lanelet_pose = traffic_simulator_msgs::msg::LaneletPose();
    ref_status.lanelet_pose_valid = new_lanelet_pose_valid;
  }

  dummy_base->setStatus(
    traffic_simulator::entity_status::CanonicalizedEntityStatus(new_status, hdmap_utils));

  EXPECT_EQ(
    static_cast<traffic_simulator_msgs::msg::EntityStatus>(dummy_base->getStatus()),
    static_cast<traffic_simulator_msgs::msg::EntityStatus>(ref_status));
}

TEST_F(EntityBaseTest, requestFollowTrajectory)
{
  auto ptr = std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>();
  EXPECT_THROW(dummy.requestFollowTrajectory(ptr), common::Error);
}

TEST_F(EntityBaseTest, requestWalkStraight)
{
  EXPECT_THROW(dummy.requestWalkStraight(), common::Error);
}

TEST_F(EntityBaseTest, updateStandStillDuration_startedMoving)
{
  dummy.startNpcLogic();
  dummy.setLinearVelocity(3.0);

  EXPECT_EQ(0.0, dummy.updateStandStillDuration(0.1));
}

TEST_F(EntityBaseTest, updateStandStillDuration_notStarted)
{
  dummy.setLinearVelocity(3.0);
  EXPECT_EQ(0.0, dummy.updateStandStillDuration(0.1));

  dummy.setLinearVelocity(0.0);
  EXPECT_EQ(0.0, dummy.updateStandStillDuration(0.1));
}

TEST_F(EntityBaseTest, updateTraveledDistance_startedMoving)
{
  double velocity = 3.0;
  double step_time = 0.1;
  dummy.startNpcLogic();
  dummy.setLinearVelocity(velocity);

  EXPECT_EQ(1.0 * step_time * velocity, dummy.updateTraveledDistance(step_time));
  EXPECT_EQ(2.0 * step_time * velocity, dummy.updateTraveledDistance(step_time));
  EXPECT_EQ(3.0 * step_time * velocity, dummy.updateTraveledDistance(step_time));
  EXPECT_EQ(4.0 * step_time * velocity, dummy.updateTraveledDistance(step_time));
}

TEST_F(EntityBaseTest, updateTraveledDistance_notStarted)
{
  double velocity = 3.0;
  double step_time = 0.1;
  dummy.setLinearVelocity(velocity);
  EXPECT_EQ(0.0, dummy.updateTraveledDistance(step_time));

  dummy.setLinearVelocity(0.0);
  EXPECT_EQ(0.0, dummy.updateTraveledDistance(step_time));
}

TEST_F(EntityBaseTest, stopAtCurrentPosition)
{
  double velocity = 3.0;
  dummy.setLinearVelocity(velocity);
  auto curr_twist = dummy.getCurrentTwist();
  EXPECT_EQ(curr_twist.linear.x, velocity);

  dummy.stopAtCurrentPosition();
  curr_twist = dummy.getCurrentTwist();
  EXPECT_EQ(curr_twist.linear.x, 0.0);
}

TEST(EntityBase, getDistanceToLeftLaneBound_one)
{
  const double lane_width = 3.0;
  const double entity_center_offset = -0.5;
  lanelet::Id id = 120659;

  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils, id);
  auto bbox = makeBoundingBox(entity_center_offset);
  auto status = makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);

  auto distance_result = dummy.getDistanceToLeftLaneBound(id);
  double distance_actual = (lane_width - bbox.dimensions.y) / 2.0 - entity_center_offset;
  EXPECT_NEAR(distance_result, distance_actual, 0.1);
}

TEST(EntityBase, getDistanceToRightLaneBound_one)
{
  const double lane_width = 3.0;
  const double entity_center_offset = -0.5;
  lanelet::Id id = 120659;

  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils, id);
  auto bbox = makeBoundingBox(entity_center_offset);
  auto status = makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);

  auto distance_result = dummy.getDistanceToRightLaneBound(id);
  double distance_actual = (lane_width - bbox.dimensions.y) / 2.0 + entity_center_offset;
  EXPECT_NEAR(distance_result, distance_actual, 0.1);
}

TEST(EntityBase, getDistanceToLaneBound_one)
{
  const double lane_width = 3.0;
  const double entity_center_offset = -0.5;
  lanelet::Id id = 120659;

  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils, id);
  auto bbox = makeBoundingBox(entity_center_offset);
  auto status = makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);

  auto distance_result = dummy.getDistanceToLaneBound(id);
  double distance_actual = std::min(
    (lane_width - bbox.dimensions.y) / 2.0 - entity_center_offset,
    (lane_width - bbox.dimensions.y) / 2.0 + entity_center_offset);
  EXPECT_NEAR(distance_result, distance_actual, 0.1);
}

TEST(EntityBase, getDistanceToLeftLaneBound)
{
  const double lane_width = 3.0;
  const double entity_center_offset = -0.5;
  lanelet::Id id_previous = 34666;
  lanelet::Id id = 120659;
  lanelet::Id id_next = 120660;

  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils, id);
  auto bbox = makeBoundingBox(entity_center_offset);
  auto status = makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);
  dummy.setRouteLanelets({id_previous, id, id_next});

  auto distance_result = dummy.getDistanceToLeftLaneBound();
  double distance_actual = (lane_width - bbox.dimensions.y) / 2.0 - entity_center_offset;
  EXPECT_NEAR(distance_result, distance_actual, 0.1);
}

TEST(EntityBase, getDistanceToLeftLaneBound_many)
{
  const double entity_center_offset = -0.5;
  lanelet::Id id_0 = 120659;
  lanelet::Id id_1 = 120659;

  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils, id_0);
  auto bbox = makeBoundingBox(entity_center_offset);
  auto status = makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);

  lanelet::Ids ids{id_0, id_1};
  auto distance_result = dummy.getDistanceToLeftLaneBound(ids);
  auto distance_actual = dummy.getDistanceToLeftLaneBound(id_0);
  EXPECT_NEAR(distance_result, distance_actual, 0.1);
}

TEST(EntityBase, getLaneletPose_notOnRoadAndCrosswalkPedestrian)
{
  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto bbox = makeBoundingBox();

  geometry_msgs::msg::Pose pose;
  pose.position.x = 3810.0;
  pose.position.y = 73745.0;

  auto status_base = makeEntityStatus(hdmap_utils, pose, bbox);
  status_base.type.type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
  traffic_simulator::CanonicalizedEntityStatus status(status_base, hdmap_utils);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);
  dummy.setEntityType(status_base.type.type);

  auto lanelet_pose = dummy.getLaneletPose(5.0);
  EXPECT_FALSE(lanelet_pose);
}

TEST(EntityBase, getLaneletPose_onRoadAndCrosswalkPedestrian)
{
  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto bbox = makeBoundingBox();

  geometry_msgs::msg::Pose pose;
  pose.position.x = 3766.1;
  pose.position.y = 73738.2;
  pose.orientation = makeQuaternionFromYaw((30.0) * M_PI / 180.0);

  auto status_base = makeEntityStatus(hdmap_utils, pose, bbox);
  status_base.type.type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
  traffic_simulator::CanonicalizedEntityStatus status(status_base, hdmap_utils);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);
  dummy.setEntityType(status_base.type.type);

  auto lanelet_pose = dummy.getLaneletPose(1.0);
  EXPECT_TRUE(lanelet_pose);
}

TEST(EntityBase, getLaneletPose_onCrosswalkNotOnRoadPedestrian)
{
  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto bbox = makeBoundingBox();

  geometry_msgs::msg::Pose pose;
  pose.position.x = 3764.5;
  pose.position.y = 73737.5;
  pose.orientation = makeQuaternionFromYaw((30.0) * M_PI / 180.0);

  auto status_base = makeEntityStatus(hdmap_utils, pose, bbox);
  status_base.type.type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
  traffic_simulator::CanonicalizedEntityStatus status(status_base, hdmap_utils);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);
  dummy.setEntityType(status_base.type.type);

  auto lanelet_pose = dummy.getLaneletPose(1.0);
  EXPECT_TRUE(lanelet_pose);
}

TEST(EntityBase, getLaneletPose_notOnRoadAndCrosswalkNotPedestrian)
{
  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto bbox = makeBoundingBox();

  geometry_msgs::msg::Pose pose;
  pose.position.x = 3810.0;
  pose.position.y = 73745.0;

  auto status_base = makeEntityStatus(hdmap_utils, pose, bbox);
  status_base.type.type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
  traffic_simulator::CanonicalizedEntityStatus status(status_base, hdmap_utils);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);
  dummy.setEntityType(status_base.type.type);

  auto lanelet_pose = dummy.getLaneletPose(5.0);
  EXPECT_FALSE(lanelet_pose);
}

TEST(EntityBase, getLaneletPose_onRoadAndCrosswalkNotPedestrian)
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

  DummyEntity dummy("dummy_entity", status, hdmap_utils);
  dummy.setEntityType(status_base.type.type);

  auto lanelet_pose = dummy.getLaneletPose(1.0);
  EXPECT_TRUE(lanelet_pose);
}

TEST(EntityBase, getLaneletPose_onCrosswalkNotOnRoadNotPedestrian)
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

  DummyEntity dummy("dummy_entity", status, hdmap_utils);
  dummy.setEntityType(status_base.type.type);

  auto lanelet_pose = dummy.getLaneletPose(1.0);
  EXPECT_FALSE(lanelet_pose);
}

TEST_F(EntityBaseTest, getMapPoseFromRelativePose_relative)
{
  const double s = 5.0;
  constexpr double eps = 0.1;

  geometry_msgs::msg::Pose relative_pose;
  relative_pose.position.x = s;

  auto result_pose = dummy.getMapPoseFromRelativePose(relative_pose);

  auto ref_pose = makeCanonicalizedLaneletPose(hdmap_utils, id, s);
  EXPECT_POSE_NEAR(result_pose, static_cast<geometry_msgs::msg::Pose>(ref_pose), eps);
}
