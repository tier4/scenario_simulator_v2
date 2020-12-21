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

#include <simulation_api/behavior/pedestrian/acquire_position_action.hpp>

#include <boost/algorithm/clamp.hpp>

#include <string>
#include <vector>
#include <memory>

namespace entity_behavior
{
namespace pedestrian
{
AcquirePositionAction::AcquirePositionAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: entity_behavior::PedestrianActionNode(name, config) {}

void AcquirePositionAction::getBlackBoardValues()
{
  openscenario_msgs::msg::LaneletPose target_lanelet_pose;
  PedestrianActionNode::getBlackBoardValues();
  if (!getInput<openscenario_msgs::msg::LaneletPose>("target_lanelet_pose", target_lanelet_pose)) {
    target_lanelet_pose_ = boost::none;
  } else {
    target_lanelet_pose_ = target_lanelet_pose;
  }
}

BT::NodeStatus AcquirePositionAction::tick()
{
  getBlackBoardValues();
  if (request != "acquire_position") {
    target_lanelet_pose_ = boost::none;
    return BT::NodeStatus::FAILURE;
  }

  if (!target_lanelet_pose_) {
    route_ = hdmap_utils->getRoute(entity_status.lanelet_pose.lanelet_id,
        target_lanelet_pose_->lanelet_id);
  }

  if (!target_speed) {
    std::vector<std::int64_t> following_lanelets;
    bool is_finded = false;
    for (auto itr = route_.begin(); itr != route_.end(); itr++) {
      if (is_finded) {
        if (following_lanelets.size() <= 3) {
          following_lanelets.push_back(*itr);
        }
      } else {
        if (entity_status.lanelet_pose.lanelet_id == *itr) {
          following_lanelets.push_back(*itr);
          is_finded = true;
        }
      }
    }
    if (following_lanelets.size() != 0) {
      target_speed = hdmap_utils->getSpeedLimit(following_lanelets);
    }
  }

  geometry_msgs::msg::Accel accel_new;
  accel_new = entity_status.action_status.accel;
  double target_accel = (target_speed.get() - entity_status.action_status.twist.linear.x) /
    step_time;
  if (entity_status.action_status.twist.linear.x > target_speed.get()) {
    target_accel = boost::algorithm::clamp(target_accel, -5, 0);
  } else {
    target_accel = boost::algorithm::clamp(target_accel, 0, 3);
  }
  std::shared_ptr<simulation_api::entity::PedestrianParameters> pedestrian_param_ptr;
  if (!getInput<std::shared_ptr<simulation_api::entity::PedestrianParameters>>(
      "pedestrian_parameters", pedestrian_param_ptr))
  {
    throw BehaviorTreeRuntimeError("failed to get input pedestrian_parameters in FollowLaneAction");
  }

  accel_new.linear.x = target_accel;
  geometry_msgs::msg::Twist twist_new;
  twist_new.linear.x = boost::algorithm::clamp(
    entity_status.action_status.twist.linear.x + accel_new.linear.x * step_time,
    0, 10.0);
  twist_new.linear.y = 0.0;
  twist_new.linear.z = 0.0;
  twist_new.angular.x = 0.0;
  twist_new.angular.y = 0.0;
  twist_new.angular.z = 0.0;
  double new_s = entity_status.lanelet_pose.s +
    (twist_new.linear.x + entity_status.action_status.twist.linear.x) / 2.0 *
    step_time;

  if (target_lanelet_pose_->lanelet_id == entity_status.lanelet_pose.lanelet_id) {
    if (target_lanelet_pose_->s < entity_status.lanelet_pose.s) {
      geometry_msgs::msg::Vector3 rpy = entity_status.lanelet_pose.rpy;
      openscenario_msgs::msg::EntityStatus entity_status_updated;
      entity_status_updated.time = current_time + step_time;
      entity_status_updated.lanelet_pose.lanelet_id = entity_status.lanelet_pose.lanelet_id;
      entity_status_updated.lanelet_pose.s = new_s;
      entity_status_updated.lanelet_pose.offset = entity_status.lanelet_pose.offset;
      entity_status_updated.lanelet_pose.rpy = rpy;
      entity_status_updated.action_status.twist = twist_new;
      entity_status_updated.action_status.accel = accel_new;
      entity_status_updated.pose = hdmap_utils->toMapPose(entity_status.lanelet_pose).pose;
      setOutput("updated_status", entity_status_updated);
      target_lanelet_pose_ = boost::none;
      return BT::NodeStatus::SUCCESS;
    }
  }

  if (new_s > hdmap_utils->getLaneletLength(entity_status.lanelet_pose.lanelet_id)) {
    new_s = new_s - hdmap_utils->getLaneletLength(entity_status.lanelet_pose.lanelet_id);
    boost::optional<std::int64_t> next_lanelet_id;
    bool is_finded = false;
    for (size_t i = 0; i != route_.size(); i++) {
      if (route_[i] == entity_status.lanelet_pose.lanelet_id) {
        is_finded = true;
        continue;
      }
      if (is_finded && !next_lanelet_id) {
        next_lanelet_id = route_[i];
      }
    }
    if (is_finded && next_lanelet_id) {
      openscenario_msgs::msg::EntityStatus entity_status_updated;
      entity_status_updated.time = current_time + step_time;
      entity_status_updated.lanelet_pose.lanelet_id = next_lanelet_id.get();
      entity_status_updated.lanelet_pose.s = new_s;
      entity_status_updated.lanelet_pose.offset = entity_status.lanelet_pose.offset;
      entity_status_updated.lanelet_pose.rpy = entity_status.lanelet_pose.rpy;
      entity_status_updated.pose =
        hdmap_utils->toMapPose(entity_status_updated.lanelet_pose).pose;
      entity_status_updated.action_status.twist = twist_new;
      entity_status_updated.action_status.accel = accel_new;
      setOutput("updated_status", entity_status_updated);
      return BT::NodeStatus::RUNNING;
    } else {
      throw BehaviorTreeRuntimeError("failed to find next lanelet id");
    }
  } else {
    geometry_msgs::msg::Vector3 rpy = entity_status.lanelet_pose.rpy;
    openscenario_msgs::msg::LaneletPose lanelet_pose;
    lanelet_pose.lanelet_id = entity_status.lanelet_pose.lanelet_id;
    lanelet_pose.s = new_s;
    lanelet_pose.offset = entity_status.lanelet_pose.offset;
    lanelet_pose.rpy = rpy;
    openscenario_msgs::msg::EntityStatus entity_status_updated;
    entity_status_updated.time = current_time + step_time;
    entity_status_updated.lanelet_pose = lanelet_pose;
    entity_status_updated.pose = hdmap_utils->toMapPose(lanelet_pose).pose;
    entity_status_updated.action_status.twist = twist_new;
    entity_status_updated.action_status.accel = accel_new;
    setOutput("updated_status", entity_status_updated);
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::RUNNING;
}
}      // namespace pedestrian
}  // namespace entity_behavior
