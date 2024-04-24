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

auto makePoint(const double x, const double y, const double z = 0.0) -> geometry_msgs::msg::Point
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

auto makeQuaternionFromYaw(const double yaw) -> geometry_msgs::msg::Quaternion
{
  geometry_msgs::msg::Vector3 v;
  v.z = yaw;
  return quaternion_operation::convertEulerAngleToQuaternion(v);
}

TEST(EntityBase, asFieldOperatorApplication)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  EXPECT_THROW(dummy.asFieldOperatorApplication(), std::runtime_error);
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
  EXPECT_THROW(dummy.onPostUpdate(current_time, step_time), std::runtime_error);
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
  EXPECT_THROW(dummy.onPostUpdate(current_time, step_time), std::runtime_error);
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
  EXPECT_THROW(dummy.onPostUpdate(current_time, step_time), std::runtime_error);
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

TEST(EntityBase, setDynamicConstraints)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  traffic_simulator_msgs::msg::DynamicConstraints default_constraints{};

  default_constraints.max_speed = 5.0;
  default_constraints.max_acceleration = 7.0;
  default_constraints.max_deceleration = 11.0;
  default_constraints.max_acceleration_rate = 13.0;
  default_constraints.max_deceleration_rate = 17.0;
  dummy.setDynamicConstraints(default_constraints);
  auto current_constraints = dummy.getDynamicConstraints();

  EXPECT_DYNAMIC_CONSTRAINTS_EQ(default_constraints, current_constraints);
}

TEST(EntityBase, requestFollowTrajectory)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  auto ptr = std::make_shared<traffic_simulator_msgs::msg::PolylineTrajectory>();
  EXPECT_THROW(dummy.requestFollowTrajectory(ptr), std::runtime_error);
}

TEST(EntityBase, requestWalkStraight)
{
  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  EXPECT_THROW(dummy.requestWalkStraight(), std::runtime_error);
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
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  /* 
  auto distance = dummy.getDistanceToLeftLaneBound();
  sigsegv if invoked
  tested variables depend on the fix which is to be implemented
  */
  throw std::runtime_error("Fix not implemented");
}

TEST(EntityBase, getDistanceToRightLaneBound)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  /*
  auto distance = dummy.getDistanceToRightLaneBound();
  sigsegv if invoked
  tested variables depend on the fix which is to be implemented
  */
  throw std::runtime_error("Fix not implemented");
}

TEST(EntityBase, getDistanceToLaneBound)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);

  /*
  auto distance = dummy.getDistanceToLaneBound();
  sigsegv if invoked
  tested variables depend on the fix which is to be implemented
  */
  throw std::runtime_error("Fix not implemented");
}

TEST(EntityBase, getDistanceToLeftLaneBound_empty)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  auto ids = std::vector<lanelet::Id>{};

  /*
  auto left = dummy.getDistanceToLeftLaneBound(ids);
  sigsegv if invoked
  tested variables depend on the fix which is to be implemented
  */
  throw std::runtime_error("Fix not implemented");
}

TEST(EntityBase, getDistanceToRightLaneBound_empty)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  auto ids = std::vector<lanelet::Id>{};

  /*
  auto right = dummy.getDistanceToRightLaneBound(ids);
  sigsegv if invoked
  tested variables depend on the fix which is to be implemented
  */
  throw std::runtime_error("Fix not implemented");
}

TEST(EntityBase, getDistanceToLaneBound_empty)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  auto ids = std::vector<lanelet::Id>{};

  /*
  auto distance = dummy.getDistanceToLaneBound(ids);
  sigsegv if invoked
  tested variables depend on the fix which is to be implemented
  */
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

TEST(EntityBase, requestSpeedChange_targetSpeedAbsoluteNotContinuous)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  EXPECT_FALSE(dummy.getTargetSpeed().has_value());

  const double target_speed = 3.0;
  const bool continuous = false;

  dummy.requestSpeedChange(target_speed, continuous);
  EXPECT_TRUE(dummy.getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy.getTargetSpeed().value());

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy.getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy.getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);

  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy.getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedAbsoluteContinuous)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  EXPECT_FALSE(dummy.getTargetSpeed().has_value());

  const double target_speed = 3.0;
  const bool continuous = true;

  dummy.requestSpeedChange(target_speed, continuous);
  EXPECT_TRUE(dummy.getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy.getTargetSpeed().value());

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy.getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy.getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);

  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy.getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy.getTargetSpeed().value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedAbsoluteReached)
{
  lanelet::Id id = 120659;

  auto hdmap_utils_ptr = makeHdMapUtilsSharedPointer();
  auto pose = makeCanonicalizedLaneletPose(hdmap_utils_ptr, id);
  auto bbox = makeBoundingBox();
  auto status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox);

  DummyEntity dummy("dummy_entity", status, hdmap_utils_ptr);
  EXPECT_FALSE(dummy.getTargetSpeed().has_value());

  const double target_speed = 3.0;
  const bool continuous = false;

  dummy.setLinearVelocity(target_speed);
  dummy.requestSpeedChange(target_speed, continuous);
  EXPECT_FALSE(dummy.getTargetSpeed().has_value());
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

  EXPECT_THROW(dummy.requestSpeedChange(relative_taget_speed, continuous), std::runtime_error);

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);

  EXPECT_FALSE(dummy.getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeNotContinuous)
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
  auto bob_status = makeCanonicalizedEntityStatus(hdmap_utils_ptr, pose, bbox, bob_speed, "bob");
  std::unordered_map<std::string, traffic_simulator::entity_status::CanonicalizedEntityStatus>
    others;
  others.emplace("bob_entity", bob_status);
  dummy.setOtherStatus(others);

  traffic_simulator::speed_change::RelativeTargetSpeed relative_taget_speed(
    "bob_entity", traffic_simulator::speed_change::RelativeTargetSpeed::Type::DELTA, delta_speed);
  bool continuous = false;
  EXPECT_NO_THROW(dummy.requestSpeedChange(relative_taget_speed, continuous));
  // the job cannot change target_speed. entity_base.cpp, 644 should be moved 2 lines down
  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);

  EXPECT_TRUE(dummy.getTargetSpeed().has_value());
  EXPECT_EQ(dummy.getTargetSpeed().value(), target_speed);

  dummy.setLinearVelocity(target_speed);

  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy.getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeContinuousInvalidTarget)
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
  bool continuous = true;

  EXPECT_THROW(dummy.requestSpeedChange(relative_taget_speed, continuous), std::runtime_error);
  // inconsistency: entity_base.cpp, 618 will only check and throw on "invalid_name" if continuous == false
  // this differs form functionality tested in requestSpeedChange_targetSpeedRelativeNotContinuousInvalidTarget
  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);

  EXPECT_FALSE(dummy.getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeContinuous)
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
  EXPECT_TRUE(dummy.getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy.getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);

  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy.getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy.getTargetSpeed().value());
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
  EXPECT_FALSE(dummy.getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeConstraintNone)
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
  EXPECT_TRUE(dummy.getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy.getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy.getTargetSpeed().has_value());
}

TEST(EntityBase, requestSpeedChange_targetSpeedRelativeConstraintAccelerationTransitionStep)
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

  const double current_time = 5.0;
  const double step_time = 7.0;
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_TRUE(dummy.getTargetSpeed().has_value());
  EXPECT_EQ(target_speed, dummy.getTargetSpeed().value());

  dummy.setLinearVelocity(target_speed);
  dummy.onPostUpdate(current_time, step_time);
  EXPECT_FALSE(dummy.getTargetSpeed().has_value());
}

/*
182: "getDistanceToRightLaneBound" typo
183: sigsegv on empty "distances" vector
595, 630: why do these lines exist? The job seemingly will do nothing
644: this line should be moved down, just like in 604, 630, 587
385, 437, 512, 545: why is a request to change the speed being issued,
  and then, immidiately, the speed is forcefully changed? 
  The job assigned to change the target speed will return early anyway,
  after checking "isTargetSpeedReached"
551, 618: "requestSpeedChange" throws on invalid entity name if-and-only-if continuous is false.
  Order of checking in the if statement should be swapped in my opinion.
*/