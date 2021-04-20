// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <quaternion_operation/quaternion_operation.h>

#include <boost/algorithm/clamp.hpp>
#include <memory>
#include <openscenario_msgs/msg/vehicle_parameters.hpp>
#include <string>
#include <traffic_simulator/entity/exception.hpp>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
VehicleEntity::VehicleEntity(
  const std::string & name, const openscenario_msgs::msg::EntityStatus & initial_state,
  const openscenario_msgs::msg::VehicleParameters & params)
: EntityBase(params.name, name, initial_state),
  parameters(params),
  tree_ptr_(std::make_shared<entity_behavior::vehicle::BehaviorTree>())
{
  entity_type_.type = openscenario_msgs::msg::EntityType::VEHICLE;
  tree_ptr_->setValueToBlackBoard("vehicle_parameters", parameters);
}

VehicleEntity::VehicleEntity(
  const std::string & name, const openscenario_msgs::msg::VehicleParameters & params)
: EntityBase(params.name, name),
  parameters(params),
  tree_ptr_(std::make_shared<entity_behavior::vehicle::BehaviorTree>())
{
  entity_type_.type = openscenario_msgs::msg::EntityType::VEHICLE;
  tree_ptr_->setValueToBlackBoard("vehicle_parameters", parameters);
}

void VehicleEntity::requestAssignRoute(
  const std::vector<openscenario_msgs::msg::LaneletPose> & waypoints)
{
  if (status_ and status_->lanelet_pose_valid) {
    route_planner_ptr_->getRouteLanelets(status_->lanelet_pose, waypoints);
  }
}

void VehicleEntity::requestAcquirePosition(const openscenario_msgs::msg::LaneletPose & lanelet_pose)
{
  if (status_ and status_->lanelet_pose_valid) {
    route_planner_ptr_->getRouteLanelets(status_->lanelet_pose, lanelet_pose);
  }
}

void VehicleEntity::requestLaneChange(const std::int64_t to_lanelet_id)
{
  tree_ptr_->setRequest("lane_change");
  tree_ptr_->setValueToBlackBoard("to_lanelet_id", to_lanelet_id);
}

void VehicleEntity::cancelRequest() { tree_ptr_->setRequest("none"); }

void VehicleEntity::setTargetSpeed(double target_speed, bool continuous)
{
  target_speed_ = target_speed;
  tree_ptr_->setValueToBlackBoard("target_speed", target_speed_);
  if (continuous) {
    target_speed_ = boost::none;
  }
}

void VehicleEntity::onUpdate(double current_time, double step_time)
{
  if (!status_) {
    return;
  }
  if (current_time < 0) {
    updateEntityStatusTimestamp(current_time);
  } else {
    tree_ptr_->setValueToBlackBoard("other_entity_status", other_status_);
    tree_ptr_->setValueToBlackBoard("entity_type_list", entity_type_list_);
    tree_ptr_->setValueToBlackBoard("entity_status", status_.get());
    if (status_->lanelet_pose_valid) {
      tree_ptr_->setValueToBlackBoard(
        "route_lanelets", route_planner_ptr_->getRouteLanelets(status_->lanelet_pose));
    } else {
      std::vector<std::int64_t> empty = {};
      tree_ptr_->setValueToBlackBoard("route_lanelets", empty);
    }
    action_status_ = tree_ptr_->tick(current_time, step_time);
    while (getCurrentAction() == "root") {
      action_status_ = tree_ptr_->tick(current_time, step_time);
    }
    auto status_updated = tree_ptr_->getUpdatedStatus();
    if (status_updated.lanelet_pose_valid) {
      auto following_lanelets =
        hdmap_utils_ptr_->getFollowingLanelets(status_updated.lanelet_pose.lanelet_id);
      auto l = hdmap_utils_ptr_->getLaneletLength(status_updated.lanelet_pose.lanelet_id);
      if (following_lanelets.size() == 1 && l <= status_updated.lanelet_pose.s) {
        stopAtEndOfRoad();
        return;
      }
    }
    if (target_speed_) {
      if (status_updated.action_status.twist.linear.x >= target_speed_.get()) {
        target_speed_ = boost::none;
        tree_ptr_->setValueToBlackBoard("target_speed", target_speed_);
      }
    }
    if (!status_) {
      linear_jerk_ = 0;
    } else {
      linear_jerk_ =
        (status_updated.action_status.accel.linear.x - status_->action_status.accel.linear.x) /
        step_time;
    }
    setStatus(status_updated);
    updateStandStillDuration(step_time);
  }
}
}  // namespace entity
}  // namespace traffic_simulator
