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

TEST(EntityBase, appendDebugMarker)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
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

TEST(EntityBase, asFieldOperatorApplication)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  EXPECT_THROW(dummy.asFieldOperatorApplication(), common::Error);
}

TEST(EntityBase, get2DPolygon)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  const auto polygon = dummy.get2DPolygon();

  std::vector<geometry_msgs::msg::Point> ref_poly{};
  {
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
}

TEST(EntityBase, startNpcLogic)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  EXPECT_FALSE(dummy.isNpcLogicStarted());
  dummy.startNpcLogic();
  EXPECT_TRUE(dummy.isNpcLogicStarted());
}

TEST(EntityBase, activateOutOfRangeJob_speed)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

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

TEST(EntityBase, activateOutOfRangeJob_acceleration)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

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

TEST(EntityBase, activateOutOfRangeJob_jerk)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

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

TEST(EntityBase, onUpdate)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

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

  dummy._appendToJobList(
    first_update_func, first_cleanup_func, type_first, is_exclusive, first_event);
  dummy._appendToJobList(
    second_update_func, second_cleanup_func, type_second, is_exclusive, second_event);

  double current_time = 0.0;
  double step_time = 0.0;
  dummy.onUpdate(current_time, step_time);

  EXPECT_TRUE(first_cleanup);
  EXPECT_TRUE(first_update);
  EXPECT_FALSE(second_cleanup);
  EXPECT_FALSE(second_update);
}

TEST(EntityBase, onPostUpdate)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

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

  dummy._appendToJobList(
    first_update_func, first_cleanup_func, type_first, is_exclusive, first_event);
  dummy._appendToJobList(
    second_update_func, second_cleanup_func, type_second, is_exclusive, second_event);

  double current_time = 0.0;
  double step_time = 0.0;
  dummy.onPostUpdate(current_time, step_time);

  EXPECT_FALSE(first_cleanup);
  EXPECT_FALSE(first_update);
  EXPECT_TRUE(second_cleanup);
  EXPECT_TRUE(second_update);
}

TEST(EntityBase, resetDynamicConstraints)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  auto default_constraints = dummy.getDefaultDynamicConstraints();
  dummy.resetDynamicConstraints();
  auto current_constraints = dummy.getDynamicConstraints();

  EXPECT_DYNAMIC_CONSTRAINTS_EQ(default_constraints, current_constraints);
}

TEST(EntityBase, requestLaneChange_absoluteTarget)
{
  const lanelet::Id id = 120659;
  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);
  traffic_simulator::entity::EntityBase * dummy_base = &dummy;

  traffic_simulator::lane_change::AbsoluteTarget target(id, 1.0);

  traffic_simulator::lane_change::TrajectoryShape trajectory_shape =
    traffic_simulator::lane_change::TrajectoryShape::LINEAR;

  traffic_simulator::lane_change::Constraint constraint(
    traffic_simulator::lane_change::Constraint::Type::TIME, 30.0,
    traffic_simulator::lane_change::Constraint::Policy::BEST_EFFORT);

  dummy_base->requestLaneChange(target, trajectory_shape, constraint);
  auto result_param = dummy._getLaneChangeParameter();

  EXPECT_LANE_CHANGE_ABSOLUTE_TARGET_EQ(result_param.target, target);
  EXPECT_EQ(result_param.trajectory_shape, trajectory_shape);
  EXPECT_LANE_CHANGE_CONSTRAINT_EQ(result_param.constraint, constraint);
}

TEST(EntityBase, requestLaneChange_relativeTarget)
{
  const lanelet::Id id = 120659;
  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox);

  const lanelet::Id target_id = 34468;
  const std::string target_name = "target_name";
  auto target_pose = makeCanonicalizedLaneletPose(hdmap_utils, target_id, 5.0);
  auto target_status = makeCanonicalizedEntityStatus(hdmap_utils, target_pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);
  traffic_simulator::entity::EntityBase * dummy_base = &dummy;

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
  auto result_param = dummy._getLaneChangeParameter();

  traffic_simulator::lane_change::AbsoluteTarget ref_target(target_id, target_offset);

  EXPECT_LANE_CHANGE_ABSOLUTE_TARGET_EQ(result_param.target, ref_target);
  EXPECT_EQ(result_param.trajectory_shape, trajectory_shape);
  EXPECT_LANE_CHANGE_CONSTRAINT_EQ(result_param.constraint, constraint);
}

TEST(EntityBase, requestLaneChange_relativeTargetLaneletPose)
{
  const lanelet::Id id = 120659;
  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox);

  const std::string target_name = "target_name";
  geometry_msgs::msg::Pose target_pose;
  target_pose.position = makePoint(3810.0, 73745.0);  // outside of road

  auto target_status = makeCanonicalizedEntityStatus(hdmap_utils, target_pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);
  traffic_simulator::entity::EntityBase * dummy_base = &dummy;

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

TEST(EntityBase, requestLaneChange_relativeTargetnName)
{
  const lanelet::Id id = 120659;
  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox);

  const lanelet::Id target_id = 34468;
  const std::string target_name = "target_name";
  auto target_pose = makeCanonicalizedLaneletPose(hdmap_utils, target_id, 5.0);
  auto target_status = makeCanonicalizedEntityStatus(hdmap_utils, target_pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);
  traffic_simulator::entity::EntityBase * dummy_base = &dummy;

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

TEST(EntityBase, requestLaneChange_relativeTargetInvalid)
{
  const lanelet::Id id = 120659;
  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox);

  const lanelet::Id target_id = 34468;
  const std::string target_name = "target_name";
  auto target_pose = makeCanonicalizedLaneletPose(hdmap_utils, target_id, 5.0);
  auto target_status = makeCanonicalizedEntityStatus(hdmap_utils, target_pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);
  traffic_simulator::entity::EntityBase * dummy_base = &dummy;

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

TEST(EntityBase, setDynamicConstraints)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

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

TEST(EntityBase, setOtherStatus)
{
  const lanelet::Id id = 120659;

  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);
  traffic_simulator::entity::EntityBase * dummy_base = &dummy;

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

  const auto & result_status = dummy._getOtherStatus();

  EXPECT_EQ(other_status.size(), result_status.size());
  EXPECT_EQ(
    static_cast<traffic_simulator_msgs::msg::EntityStatus>(other_status.at(name0)),
    static_cast<traffic_simulator_msgs::msg::EntityStatus>(result_status.at(name0)));
  EXPECT_EQ(
    static_cast<traffic_simulator_msgs::msg::EntityStatus>(other_status.at(name1)),
    static_cast<traffic_simulator_msgs::msg::EntityStatus>(result_status.at(name1)));
}

TEST(EntityBase, setStatus)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  const auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy._setEntityType(traffic_simulator_msgs::msg::EntityType::VEHICLE);
  traffic_simulator::entity::EntityBase * dummy_base = &dummy;

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
    traffic_simulator::entity_status::CanonicalizedEntityStatus(new_status, hdmap_utils_ptr));

  EXPECT_EQ(
    static_cast<traffic_simulator_msgs::msg::EntityStatus>(dummy_base->getStatus()),
    static_cast<traffic_simulator_msgs::msg::EntityStatus>(ref_status));
}

TEST(EntityBase, requestFollowTrajectory)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  auto ptr = std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>();
  EXPECT_THROW(dummy.requestFollowTrajectory(ptr), common::Error);
}

TEST(EntityBase, requestWalkStraight)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  EXPECT_THROW(dummy.requestWalkStraight(), common::Error);
}

TEST(EntityBase, updateStandStillDuration_startedMoving)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  dummy.startNpcLogic();
  dummy.setLinearVelocity(3.0);

  EXPECT_EQ(0.0, dummy.updateStandStillDuration(0.1));
}

TEST(EntityBase, updateStandStillDuration_notStarted)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  dummy.setLinearVelocity(3.0);
  EXPECT_EQ(0.0, dummy.updateStandStillDuration(0.1));

  dummy.setLinearVelocity(0.0);
  EXPECT_EQ(0.0, dummy.updateStandStillDuration(0.1));
}

TEST(EntityBase, updateTraveledDistance_startedMoving)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  double velocity = 3.0;
  double step_time = 0.1;
  dummy.startNpcLogic();
  dummy.setLinearVelocity(velocity);

  EXPECT_EQ(1 * step_time * velocity, dummy.updateTraveledDistance(step_time));
  EXPECT_EQ(2 * step_time * velocity, dummy.updateTraveledDistance(step_time));
  EXPECT_EQ(3 * step_time * velocity, dummy.updateTraveledDistance(step_time));
  EXPECT_EQ(4 * step_time * velocity, dummy.updateTraveledDistance(step_time));
}

TEST(EntityBase, updateTraveledDistance_notStarted)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  double velocity = 3.0;
  double step_time = 0.1;
  dummy.setLinearVelocity(velocity);
  EXPECT_EQ(0.0, dummy.updateTraveledDistance(step_time));

  dummy.setLinearVelocity(0.0);
  EXPECT_EQ(0.0, dummy.updateTraveledDistance(step_time));
}

TEST(EntityBase, stopAtCurrentPosition)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

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

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox(entity_center_offset);
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  auto distance_result = dummy.getDistanceToLeftLaneBound(id);
  double distance_actual = (lane_width - bbox.dimensions.y) / 2 - entity_center_offset;
  EXPECT_NEAR(distance_result, distance_actual, 0.1);
}

TEST(EntityBase, getDistanceToRightLaneBound_one)
{
  const double lane_width = 3.0;
  const double entity_center_offset = -0.5;
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox(entity_center_offset);
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  auto distance_result = dummy.getDistanceToRightLaneBound(id);
  double distance_actual = (lane_width - bbox.dimensions.y) / 2 + entity_center_offset;
  EXPECT_NEAR(distance_result, distance_actual, 0.1);
}

TEST(EntityBase, getDistanceToLaneBound_one)
{
  const double lane_width = 3.0;
  const double entity_center_offset = -0.5;
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox(entity_center_offset);
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  auto distance_result = dummy.getDistanceToLaneBound(id);
  double distance_actual = std::min(
    (lane_width - bbox.dimensions.y) / 2 - entity_center_offset,
    (lane_width - bbox.dimensions.y) / 2 + entity_center_offset);
  EXPECT_NEAR(distance_result, distance_actual, 0.1);
}

TEST(EntityBase, getDistanceToLeftLaneBound)
{
  const double lane_width = 3.0;
  const double entity_center_offset = -0.5;
  lanelet::Id id_previous = 34666;
  lanelet::Id id = 120659;
  lanelet::Id id_next = 120660;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox(entity_center_offset);
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setRouteLanelets({id_previous, id, id_next});

  auto distance_result = dummy.getDistanceToLeftLaneBound();
  double distance_actual = (lane_width - bbox.dimensions.y) / 2 - entity_center_offset;
  EXPECT_NEAR(distance_result, distance_actual, 0.1);
}

TEST(EntityBase, getDistanceToRightLaneBound)
{
  // see issue 1 at the end of this file
  const double lane_width = 3.0;
  const double entity_center_offset = -0.5;
  lanelet::Id id_previous = 34666;
  lanelet::Id id = 120659;
  lanelet::Id id_next = 120660;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox(entity_center_offset);
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setRouteLanelets({id_previous, id, id_next});

  auto distance_result = dummy.getDistanceToRightLaneBound();
  double distance_actual = (lane_width - bbox.dimensions.y) / 2 + entity_center_offset;
  EXPECT_NEAR(distance_result, distance_actual, 0.1);
}

TEST(EntityBase, getDistanceToLaneBound)
{
  // see issue 1 at the end of this file
  const double lane_width = 3.0;
  const double entity_center_offset = -0.5;
  lanelet::Id id_previous = 34666;
  lanelet::Id id = 120659;
  lanelet::Id id_next = 120660;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox(entity_center_offset);
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setRouteLanelets({id_previous, id, id_next});

  auto distance_result = dummy.getDistanceToLaneBound();
  double distance_actual = std::min(
    (lane_width - bbox.dimensions.y) / 2 - entity_center_offset,
    (lane_width - bbox.dimensions.y) / 2 + entity_center_offset);
  EXPECT_NEAR(distance_result, distance_actual, 0.1);
}

TEST(EntityBase, getDistanceToLeftLaneBound_empty)
{
  // fix is to be implemented, 2nd issue
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  auto ids = std::vector<lanelet::Id>{};

  // auto left = dummy.getDistanceToLeftLaneBound(ids);

  throw std::runtime_error("Fix not implemented");
}

TEST(EntityBase, getDistanceToRightLaneBound_empty)
{
  // fix is to be implemented, 2nd issue
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  auto ids = std::vector<lanelet::Id>{};

  // auto right = dummy.getDistanceToRightLaneBound(ids);
  throw std::runtime_error("Fix not implemented");
}

TEST(EntityBase, getDistanceToLaneBound_empty)
{
  // fix is to be implemented, 2nd issue
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  auto ids = std::vector<lanelet::Id>{};

  // auto distance = dummy.getDistanceToLaneBound(ids);

  throw std::runtime_error("Fix not implemented");
}

TEST(EntityBase, getDistanceToLeftLaneBound_many)
{
  const double entity_center_offset = -0.5;
  lanelet::Id id_0 = 120659;
  lanelet::Id id_1 = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_0);
  auto bbox = makeBoundingBox(entity_center_offset);
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  lanelet::Ids ids{id_0, id_1};
  auto distance_result = dummy.getDistanceToLeftLaneBound(ids);
  auto distance_actual = dummy.getDistanceToLeftLaneBound(id_0);
  EXPECT_NEAR(distance_result, distance_actual, 0.1);
}

TEST(EntityBase, getDistanceToRightLaneBound_many)
{
  // typo, 1st issue
  const double entity_center_offset = -0.5;
  lanelet::Id id_0 = 120659;
  lanelet::Id id_1 = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_0);
  auto bbox = makeBoundingBox(entity_center_offset);
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  lanelet::Ids ids{id_0, id_1};
  auto distance_result = dummy.getDistanceToRightLaneBound(ids);
  auto distance_actual = dummy.getDistanceToRightLaneBound(id_0);
  EXPECT_NEAR(distance_result, distance_actual, 0.1);
}

TEST(EntityBase, getDistanceToLaneBound_many)
{
  // typo, 1st issue
  const double entity_center_offset = -0.5;
  lanelet::Id id_0 = 120659;
  lanelet::Id id_1 = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id_0);
  auto bbox = makeBoundingBox(entity_center_offset);
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  lanelet::Ids ids{id_0, id_1};
  auto distance_result = dummy.getDistanceToLaneBound(ids);
  auto distance_actual = dummy.getDistanceToLaneBound(id_0);
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
  dummy._setEntityType(status_base.type.type);

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
  dummy._setEntityType(status_base.type.type);

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
  dummy._setEntityType(status_base.type.type);

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
  dummy._setEntityType(status_base.type.type);

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
  dummy._setEntityType(status_base.type.type);

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
  dummy._setEntityType(status_base.type.type);

  auto lanelet_pose = dummy.getLaneletPose(1.0);
  EXPECT_FALSE(lanelet_pose);
}

TEST(EntityBase, getMapPoseFromRelativePose_relative)
{
  const lanelet::Id id = 120659;
  const double s = 5.0;
  constexpr double eps = 0.1;

  auto hdmap_utils = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils, id, 0.0);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils);

  geometry_msgs::msg::Pose relative_pose;
  relative_pose.position.x = s;

  auto result_pose = dummy.getMapPoseFromRelativePose(relative_pose);

  auto ref_pose = makeCanonicalizedLaneletPose(hdmap_utils, id, s);
  EXPECT_POSE_NEAR(result_pose, static_cast<geometry_msgs::msg::Pose>(ref_pose), eps);
}

TEST(EntityBase, requestSpeedChange_targetSpeedAbsoluteNotContinuous)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());

  const double target_speed = 3.0;
  const bool continuous = false;

  dummy.requestSpeedChange(target_speed, continuous);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);

  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedAbsoluteContinuous)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());

  const double target_speed = 3.0;
  const bool continuous = true;

  dummy.requestSpeedChange(target_speed, continuous);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);

  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedAbsoluteReached)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());

  const double target_speed = 3.0;
  const bool continuous = false;

  dummy.setLinearVelocity(target_speed);
  dummy.requestSpeedChange(target_speed, continuous);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeNotContinuousInvalidTarget)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "invalid_name", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  bool continuous = false;

  EXPECT_THROW(dummy.requestSpeedChange(relative_taget_speed, continuous), common::Error);

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);

  EXPECT_FALSE(dummy._getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeNotContinuous)
{
  // "requestSpeedChange" is unable to change the "target_speed", explained in the 4th issue
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  const double target_speed = bob_speed + delta_speed;
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  bool continuous = false;
  EXPECT_NO_THROW(dummy.requestSpeedChange(relative_taget_speed, continuous));

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);

  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(dummy._getTargetSpeed().value(), target_speed);

  dummy.setLinearVelocity(target_speed);

  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeContinuousInvalidTarget)
{
  // "requestSpeedChange" will not throw when "continuous" == true, explained in the 6th issue
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "invalid_name", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  bool continuous = true;

  EXPECT_THROW(dummy.requestSpeedChange(relative_taget_speed, continuous), common::Error);

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);

  EXPECT_FALSE(dummy._getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeContinuous)
{
  // "requestSpeedChange" is unable to change the "target_speed", explained in the 4th issue
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  const double target_speed = bob_speed + delta_speed;
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  bool continuous = true;

  EXPECT_NO_THROW(dummy.requestSpeedChange(relative_taget_speed, continuous));

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);

  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeConstraintNoneReached)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  const double target_speed = bob_speed + delta_speed;
  dummy.setLinearVelocity(target_speed);

  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::NONE, 0.0);
  auto transition = traffic_simulator::speed_change::Transition::AUTO;
  bool continuous = false;

  EXPECT_NO_THROW(
    dummy.requestSpeedChange(relative_taget_speed, transition, constraint, continuous));

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeConstraintNone)
{
  // "requestSpeedChange" is unable to change the "target_speed", explained in the 4th issue
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  const double target_speed = bob_speed + delta_speed;
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::NONE, 0.0);
  auto transition = traffic_simulator::speed_change::Transition::AUTO;
  bool continuous = false;

  EXPECT_NO_THROW(
    dummy.requestSpeedChange(relative_taget_speed, transition, constraint, continuous));

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeConstraintAccelerationTransitionStep)
{
  // internal call of "requestSpeedChange" is being issued with manual and forceful speed change, 5th issue
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  const double target_speed = bob_speed + delta_speed;
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 0.0);
  auto transition = traffic_simulator::speed_change::Transition::STEP;
  bool continuous = false;

  EXPECT_NO_THROW(
    dummy.requestSpeedChange(relative_taget_speed, transition, constraint, continuous));

  EXPECT_EQ(dummy.getCurrentTwist().linear.x, target_speed);
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeConstraintTimeContinuous)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::TIME, 10.0);
  auto transition = traffic_simulator::speed_change::Transition::STEP;
  bool continuous = true;

  EXPECT_THROW(
    dummy.requestSpeedChange(relative_taget_speed, transition, constraint, continuous),
    common::Error);

  EXPECT_FALSE(dummy._getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeReached)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  const double target_speed = bob_speed + delta_speed;
  dummy.setLinearVelocity(target_speed);

  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  const double constraint_value = 10.0;
  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, constraint_value);
  auto transition = traffic_simulator::speed_change::Transition::LINEAR;
  bool continuous = false;

  EXPECT_NO_THROW(
    dummy.requestSpeedChange(relative_taget_speed, transition, constraint, continuous));

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeConstraintTimeTransitionStep)
{
  // internal call of "requestSpeedChange" is being issued with manual and forceful speed change, 5th issue
  lanelet::Id id = 120659;

  const double initial_speed = 25.0;
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setLinearVelocity(initial_speed);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  const double target_speed = bob_speed + delta_speed;
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  const double constraint_value = 10.0;
  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::TIME, constraint_value);
  auto transition = traffic_simulator::speed_change::Transition::STEP;
  bool continuous = false;

  EXPECT_NO_THROW(
    dummy.requestSpeedChange(relative_taget_speed, transition, constraint, continuous));

  EXPECT_EQ(dummy.getCurrentTwist().linear.x, target_speed);
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeConstraintTimeTransitionNotStepNoTime)
{
  // internal call of "requestSpeedChange" is being issued with manual and forceful speed change, 5th issue
  lanelet::Id id = 120659;

  const double initial_speed = 10.0;
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setLinearVelocity(initial_speed);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  const double target_speed = bob_speed + delta_speed;
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  const double constraint_value = 0.0;
  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::TIME, constraint_value);
  auto transition = traffic_simulator::speed_change::Transition::LINEAR;
  bool continuous = false;

  EXPECT_NO_THROW(
    dummy.requestSpeedChange(relative_taget_speed, transition, constraint, continuous));

  EXPECT_EQ(dummy.getCurrentTwist().linear.x, target_speed);
}

TEST(EntityBase, requestSpeedChange_targetSpeedAbsoluteConstraintNoneReached)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  const double target_speed = 20.0;
  dummy.setLinearVelocity(target_speed);

  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::NONE, 0.0);
  auto transition = traffic_simulator::speed_change::Transition::AUTO;
  bool continuous = false;

  EXPECT_NO_THROW(dummy.requestSpeedChange(target_speed, transition, constraint, continuous));

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedAbsoluteConstraintNone)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  const double target_speed = 20.0;
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::NONE, 0.0);
  auto transition = traffic_simulator::speed_change::Transition::AUTO;
  bool continuous = false;

  EXPECT_NO_THROW(dummy.requestSpeedChange(target_speed, transition, constraint, continuous));

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedAbsoluteConstraintAccelerationTransitionStep)
{
  // internal call of "requestSpeedChange" is being issued with manual and forceful speed change, 5th issue
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  const double target_speed = 20.0;
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, 0.0);
  auto transition = traffic_simulator::speed_change::Transition::STEP;
  bool continuous = false;

  EXPECT_NO_THROW(dummy.requestSpeedChange(target_speed, transition, constraint, continuous));

  EXPECT_EQ(dummy.getCurrentTwist().linear.x, target_speed);
}

TEST(EntityBase, requestSpeedChange_targetSpeedAbsoluteConstraintTimeTransitionStep)
{
  // internal call of "requestSpeedChange" is being issued with manual and forceful speed change, 5th issue
  lanelet::Id id = 120659;

  const double initial_speed = 25.0;
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setLinearVelocity(initial_speed);

  const double target_speed = 20.0;
  const double constraint_value = 10.0;
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::TIME, constraint_value);
  auto transition = traffic_simulator::speed_change::Transition::STEP;
  bool continuous = false;

  EXPECT_NO_THROW(dummy.requestSpeedChange(target_speed, transition, constraint, continuous));

  EXPECT_EQ(dummy.getCurrentTwist().linear.x, target_speed);
}

TEST(EntityBase, requestSpeedChange_targetSpeedAbsoluteConstraintTimeTransitionNotStepNoTime)
{
  // internal call of "requestSpeedChange" is being issued with manual and forceful speed change, 5th issue
  lanelet::Id id = 120659;

  const double initial_speed = 10.0;
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setLinearVelocity(initial_speed);

  const double target_speed = 20.0;
  const double constraint_value = 0.0;
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::TIME, constraint_value);
  auto transition = traffic_simulator::speed_change::Transition::LINEAR;
  bool continuous = false;

  EXPECT_NO_THROW(dummy.requestSpeedChange(target_speed, transition, constraint, continuous));

  EXPECT_EQ(dummy.getCurrentTwist().linear.x, target_speed);
}

TEST(EntityBase, requestSpeedChange_targetSpeedAbsoluteConstraintAccelerationTransitionLinear)
{
  lanelet::Id id = 120659;

  const double initial_speed = 10.0;
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setLinearVelocity(initial_speed);

  const double target_speed = 20.0;
  const double constraint_value = 5.0;
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, constraint_value);
  auto transition = traffic_simulator::speed_change::Transition::LINEAR;
  bool continuous = false;

  EXPECT_NO_THROW(dummy.requestSpeedChange(target_speed, transition, constraint, continuous));

  EXPECT_EQ(dummy.getCurrentTwist().linear.x, initial_speed);

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());

  auto default_constraints = dummy.getDefaultDynamicConstraints();
  auto current_constraints = dummy.getDynamicConstraints();
  EXPECT_DYNAMIC_CONSTRAINTS_EQ(default_constraints, current_constraints);
}

TEST(
  EntityBase, requestSpeedChange_targetSpeedAbsoluteConstraintAccelerationTransitionAutoAccelerate)
{
  lanelet::Id id = 120659;

  const double initial_speed = 10.0;
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setLinearVelocity(initial_speed);

  const double target_speed = 20.0;
  const double constraint_value = 5.0;
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, constraint_value);
  auto transition = traffic_simulator::speed_change::Transition::AUTO;
  bool continuous = false;

  EXPECT_NO_THROW(dummy.requestSpeedChange(target_speed, transition, constraint, continuous));

  EXPECT_EQ(dummy.getCurrentTwist().linear.x, initial_speed);

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());

  auto default_constraints = dummy.getDefaultDynamicConstraints();
  auto current_constraints = dummy.getDynamicConstraints();
  EXPECT_DYNAMIC_CONSTRAINTS_EQ(default_constraints, current_constraints);
}

TEST(
  EntityBase, requestSpeedChange_targetSpeedAbsoluteConstraintAccelerationTransitionAutoDecelerate)
{
  lanelet::Id id = 120659;

  const double initial_speed = 30.0;
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setLinearVelocity(initial_speed);

  const double target_speed = 20.0;
  const double constraint_value = 5.0;
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, constraint_value);
  auto transition = traffic_simulator::speed_change::Transition::AUTO;
  bool continuous = false;

  EXPECT_NO_THROW(dummy.requestSpeedChange(target_speed, transition, constraint, continuous));

  EXPECT_EQ(dummy.getCurrentTwist().linear.x, initial_speed);

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());

  auto default_constraints = dummy.getDefaultDynamicConstraints();
  auto current_constraints = dummy.getDynamicConstraints();
  EXPECT_DYNAMIC_CONSTRAINTS_EQ(default_constraints, current_constraints);
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeConstraintAccelerationTransitionLinear)
{
  lanelet::Id id = 120659;

  const double initial_speed = 10.0;
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setLinearVelocity(initial_speed);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  const double target_speed = bob_speed + delta_speed;
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  const double constraint_value = 0.0;
  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, constraint_value);
  auto transition = traffic_simulator::speed_change::Transition::LINEAR;
  bool continuous = false;

  EXPECT_NO_THROW(dummy.requestSpeedChange(target_speed, transition, constraint, continuous));

  EXPECT_EQ(dummy.getCurrentTwist().linear.x, initial_speed);

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());

  auto default_constraints = dummy.getDefaultDynamicConstraints();
  auto current_constraints = dummy.getDynamicConstraints();
  EXPECT_DYNAMIC_CONSTRAINTS_EQ(default_constraints, current_constraints);
}

TEST(
  EntityBase, requestSpeedChange_targetSpeedRelativeConstraintAccelerationTransitionAutoAccelerate)
{
  // "requestSpeedChange" is unable to change the "target_speed", explained in the 4th issue
  lanelet::Id id = 120659;

  const double initial_speed = 10.0;
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setLinearVelocity(initial_speed);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  const double target_speed = bob_speed + delta_speed;
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  const double constraint_value = 0.0;
  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, constraint_value);
  auto transition = traffic_simulator::speed_change::Transition::AUTO;
  bool continuous = false;

  EXPECT_NO_THROW(
    dummy.requestSpeedChange(relative_taget_speed, transition, constraint, continuous));

  EXPECT_EQ(dummy.getCurrentTwist().linear.x, initial_speed);

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());

  auto default_constraints = dummy.getDefaultDynamicConstraints();
  auto current_constraints = dummy.getDynamicConstraints();
  EXPECT_DYNAMIC_CONSTRAINTS_EQ(default_constraints, current_constraints);
}

TEST(
  EntityBase, requestSpeedChange_targetSpeedRelativeConstraintAccelerationTransitionAutoDecelerate)
{
  // "requestSpeedChange" is unable to change the "target_speed", explained in the 4th issue
  lanelet::Id id = 120659;

  const double initial_speed = 30.0;
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setLinearVelocity(initial_speed);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  const double target_speed = bob_speed + delta_speed;
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  const double constraint_value = 0.0;
  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::LONGITUDINAL_ACCELERATION, constraint_value);
  auto transition = traffic_simulator::speed_change::Transition::AUTO;
  bool continuous = false;

  EXPECT_NO_THROW(
    dummy.requestSpeedChange(relative_taget_speed, transition, constraint, continuous));

  EXPECT_EQ(dummy.getCurrentTwist().linear.x, initial_speed);

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());

  auto default_constraints = dummy.getDefaultDynamicConstraints();
  auto current_constraints = dummy.getDynamicConstraints();
  EXPECT_DYNAMIC_CONSTRAINTS_EQ(default_constraints, current_constraints);
}

TEST(EntityBase, requestSpeedChange_targetSpeedAbsoluteConstraintTimeTransitionLinear)
{
  lanelet::Id id = 120659;

  const double initial_speed = 25.0;
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setLinearVelocity(initial_speed);

  const double target_speed = 20.0;
  const double constraint_value = 10.0;
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::TIME, constraint_value);
  auto transition = traffic_simulator::speed_change::Transition::LINEAR;
  bool continuous = false;

  EXPECT_NO_THROW(dummy.requestSpeedChange(target_speed, transition, constraint, continuous));

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());

  auto default_constraints = dummy.getDefaultDynamicConstraints();
  auto current_constraints = dummy.getDynamicConstraints();
  EXPECT_DYNAMIC_CONSTRAINTS_EQ(default_constraints, current_constraints);
}

TEST(EntityBase, requestSpeedChange_targetSpeedAbsoluteConstraintTimeTransitionAuto)
{
  lanelet::Id id = 120659;

  const double initial_speed = 10.0;
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setLinearVelocity(initial_speed);

  const double target_speed = 20.0;
  const double constraint_value = 10.0;
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::TIME, constraint_value);
  auto transition = traffic_simulator::speed_change::Transition::AUTO;
  bool continuous = false;

  EXPECT_NO_THROW(dummy.requestSpeedChange(target_speed, transition, constraint, continuous));

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());

  auto default_constraints = dummy.getDefaultDynamicConstraints();
  auto current_constraints = dummy.getDynamicConstraints();
  EXPECT_DYNAMIC_CONSTRAINTS_EQ(default_constraints, current_constraints);
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeConstraintTimeTransitionLinear)
{
  lanelet::Id id = 120659;

  const double initial_speed = 25.0;
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setLinearVelocity(initial_speed);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  const double target_speed = bob_speed + delta_speed;
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  const double constraint_value = 10.0;
  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::TIME, constraint_value);
  auto transition = traffic_simulator::speed_change::Transition::LINEAR;
  bool continuous = false;

  EXPECT_NO_THROW(
    dummy.requestSpeedChange(relative_taget_speed, transition, constraint, continuous));

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());

  auto default_constraints = dummy.getDefaultDynamicConstraints();
  auto current_constraints = dummy.getDynamicConstraints();
  EXPECT_DYNAMIC_CONSTRAINTS_EQ(default_constraints, current_constraints);
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeConstraintTimeTransitionAuto)
{
  lanelet::Id id = 120659;

  const double initial_speed = 10.0;
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);
  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  dummy.setLinearVelocity(initial_speed);

  const double bob_speed = 17.0;
  const double delta_speed = 3.0;
  const double target_speed = bob_speed + delta_speed;
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  const double constraint_value = 10.0;
  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  traffic_simulator::speed_change::Constraint constraint(
    traffic_simulator::speed_change::Constraint::Type::TIME, constraint_value);
  auto transition = traffic_simulator::speed_change::Transition::AUTO;
  bool continuous = false;

  EXPECT_NO_THROW(
    dummy.requestSpeedChange(relative_taget_speed, transition, constraint, continuous));

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy._getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy._getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy._getTargetSpeed().has_value());

  auto default_constraints = dummy.getDefaultDynamicConstraints();
  auto current_constraints = dummy.getDynamicConstraints();
  EXPECT_DYNAMIC_CONSTRAINTS_EQ(default_constraints, current_constraints);
}

/*
ISSUES:
1: 182: "getDistanceToRightLaneBound" typo
2: 183: sigsegv on empty "distances" vector
3: 595, 581: removal of these lines might be considered.
  It looks like a job to change the "target_speed" is added after
  the "target_speed" is already set for the desired value.
  The job will do nothing, if continuous == false and the target_speed is already set
4: 644: this line should be moved down, just like in 604, 630, 587.
  "target_speed" often will not be set because of this line.
5: 385, 437, 512, 545: it is unclear, why is a request to change the speed being issued,
  and then, immediately, the speed is forcefully changed.
  The job assigned to change the target speed will return early anyway,
  after checking "isTargetSpeedReached" (if continuous == false)
6: 551, 618: "requestSpeedChange" throws on invalid entity name if-and-only-if continuous is false.
  Order of checking in the if statement might be swapped to fix this.
7: 49, 59: it should be considered to make the "cancelRequest" and "appendDebugMarker" functions
  purely virtual, or make them throw.
8: 565: "requestSpeedChange" throws if "continuous" == true with "RelativeTargetSpeed",
  whereas it does not when called with absolute ratget speed, in line 456.
*/
