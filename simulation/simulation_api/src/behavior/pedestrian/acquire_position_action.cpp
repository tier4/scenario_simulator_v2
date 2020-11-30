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
  simulation_api::entity::EntityStatus target_status;
  PedestrianActionNode::getBlackBoardValues();
  if (!getInput<simulation_api::entity::EntityStatus>("target_status", target_status)) {
    target_status_ = boost::none;
  } else {
    target_status_ = target_status;
  }
}

BT::NodeStatus AcquirePositionAction::tick()
{
  getBlackBoardValues();
  if (request != "acquire_position") {
    target_status_ = boost::none;
    return BT::NodeStatus::FAILURE;
  }

  if (entity_status.coordinate == simulation_api::entity::CoordinateFrameTypes::WORLD) {
    target_status_ = boost::none;
    return BT::NodeStatus::FAILURE;
  }

  if (target_status_->coordinate == simulation_api::entity::CoordinateFrameTypes::WORLD) {
    target_status_ = boost::none;
    return BT::NodeStatus::FAILURE;
  }

  if (!target_status_) {
    route_ = hdmap_utils->getRoute(entity_status.lanelet_id, target_status_->lanelet_id);
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
        if (entity_status.lanelet_id == *itr) {
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
  accel_new = entity_status.accel;
  double target_accel = (target_speed.get() - entity_status.twist.linear.x) / step_time;
  if (entity_status.twist.linear.x > target_speed.get()) {
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
    entity_status.twist.linear.x + accel_new.linear.x * step_time,
    0, 10.0);
  twist_new.linear.y = 0.0;
  twist_new.linear.z = 0.0;
  twist_new.angular.x = 0.0;
  twist_new.angular.y = 0.0;
  twist_new.angular.z = 0.0;
  double new_s = entity_status.s + (twist_new.linear.x + entity_status.twist.linear.x) / 2.0 *
    step_time;

  if (target_status_->lanelet_id == entity_status.lanelet_id) {
    if (target_status_->s < entity_status.s) {
      geometry_msgs::msg::Vector3 rpy = entity_status.rpy;
      simulation_api::entity::EntityStatus entity_status_updated(current_time + step_time,
        entity_status.lanelet_id, new_s, entity_status.offset, rpy, twist_new, accel_new);
      setOutput("updated_status", entity_status_updated);
      target_status_ = boost::none;
      return BT::NodeStatus::SUCCESS;
    }
  }

  if (new_s > hdmap_utils->getLaneletLength(entity_status.lanelet_id)) {
    new_s = new_s - hdmap_utils->getLaneletLength(entity_status.lanelet_id);
    boost::optional<std::int64_t> next_lanelet_id;
    bool is_finded = false;
    for (size_t i = 0; i != route_.size(); i++) {
      if (route_[i] == entity_status.lanelet_id) {
        is_finded = true;
        continue;
      }
      if (is_finded && !next_lanelet_id) {
        next_lanelet_id = route_[i];
      }
    }
    if (is_finded && next_lanelet_id) {
      geometry_msgs::msg::Vector3 rpy = entity_status.rpy;
      simulation_api::entity::EntityStatus entity_status_updated(current_time + step_time,
        next_lanelet_id.get(), new_s, entity_status.offset, rpy, twist_new, accel_new);
      setOutput("updated_status", entity_status_updated);
      return BT::NodeStatus::RUNNING;
    } else {
      throw BehaviorTreeRuntimeError("failed to find next lanelet id");
    }
  } else {
    geometry_msgs::msg::Vector3 rpy = entity_status.rpy;
    simulation_api::entity::EntityStatus entity_status_updated(current_time + step_time,
      entity_status.lanelet_id, new_s, entity_status.offset, rpy, twist_new, accel_new);
    setOutput("updated_status", entity_status_updated);
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::RUNNING;
}
}      // namespace pedestrian
}  // namespace entity_behavior
