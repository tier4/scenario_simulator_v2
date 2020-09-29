// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <simulation_controller/behavior/vehicle/behavior_tree.hpp>
#include <simulation_controller/behavior/vehicle/acquire_position_action.hpp>

#include <boost/algorithm/clamp.hpp>

namespace entity_behavior
{
namespace vehicle
{
AcquirePositionAction::AcquirePositionAction(
  const std::string & name,
  const BT::NodeConfiguration & config)
: entity_behavior::ActionNode(name, config)
{

}

BT::NodeStatus AcquirePositionAction::tick()
{
  std::string request;
  if (!getInput("request", request)) {
    throw BehaviorTreeRuntimeError("failed to get input request in AcquirePositionAction");
  }
  if (request != "acquire_position") {
    following_trajectory_ = std::vector<geometry_msgs::msg::Point>(0);
    target_status_ = boost::none;
    route_ = boost::none;
    return BT::NodeStatus::FAILURE;
  }
  double step_time, current_time;
  if (!getInput<double>("step_time", step_time)) {
    throw BehaviorTreeRuntimeError("failed to get input step_time in FollowLaneAction");
  }
  if (!getInput<double>("current_time", current_time)) {
    throw BehaviorTreeRuntimeError("failed to get input current_time in FollowLaneAction");
  }
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr;
  if (!getInput<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils", hdmap_utils_ptr)) {
    throw BehaviorTreeRuntimeError("failed to get input hdmap_utils in FollowLaneAction");
  }

  simulation_controller::entity::EntityStatus entity_status;
  if (!getInput<simulation_controller::entity::EntityStatus>("entity_status", entity_status)) {
    throw BehaviorTreeRuntimeError("failed to get input entity_status in FollowLaneAction");
  }
  simulation_controller::entity::EntityStatus target_status;
  if (!getInput<simulation_controller::entity::EntityStatus>("target_status", target_status)) {
    target_status_ = boost::none;
    following_trajectory_ = std::vector<geometry_msgs::msg::Point>(0);
    route_ = boost::none;
    return BT::NodeStatus::FAILURE;
  }

  if (entity_status.coordinate == simulation_controller::entity::CoordinateFrameTypes::WORLD) {
    target_status_ = boost::none;
    following_trajectory_ = std::vector<geometry_msgs::msg::Point>(0);
    route_ = boost::none;
    return BT::NodeStatus::FAILURE;
  }

  if (target_status.coordinate == simulation_controller::entity::CoordinateFrameTypes::WORLD) {
    target_status_ = boost::none;
    following_trajectory_ = std::vector<geometry_msgs::msg::Point>(0);
    route_ = boost::none;
    return BT::NodeStatus::FAILURE;
  }

  if (!target_status_) {
    target_status_ = target_status;
    route_ = hdmap_utils_ptr->getRoute(entity_status.lanelet_id, target_status_->lanelet_id);
  }

  boost::optional<double> target_speed;
  if (!getInput<boost::optional<double>>("target_speed", target_speed)) {
    target_speed = boost::none;
  }

  if (!target_speed) {
    std::vector<int> following_lanelets;
    bool is_finded = false;
    for (auto itr = route_->begin(); itr != route_->end(); itr++) {
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
      target_speed = hdmap_utils_ptr->getSpeedLimit(following_lanelets);
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

  std::shared_ptr<simulation_controller::entity::VehicleParameters> vehicle_param_ptr;
  if (!getInput<std::shared_ptr<simulation_controller::entity::VehicleParameters>>(
      "vehicle_parameters", vehicle_param_ptr))
  {
    throw BehaviorTreeRuntimeError("failed to get input vehicle_parameters in FollowLaneAction");
  }

  accel_new.linear.x = target_accel;
  geometry_msgs::msg::Twist twist_new;
  twist_new.linear.x = boost::algorithm::clamp(
    entity_status.twist.linear.x + accel_new.linear.x * step_time,
    0, vehicle_param_ptr->performance.max_speed);
  twist_new.linear.y = 0.0;
  twist_new.linear.z = 0.0;
  twist_new.angular.x = 0.0;
  twist_new.angular.y = 0.0;
  twist_new.angular.z = 0.0;
  double new_s = entity_status.s + (twist_new.linear.x + entity_status.twist.linear.x) / 2.0 *
    step_time;

  if (target_status.lanelet_id == entity_status.lanelet_id) {
    if (target_status.s < entity_status.s) {
      geometry_msgs::msg::Vector3 rpy = entity_status.rpy;
      simulation_controller::entity::EntityStatus entity_status_updated(current_time + step_time,
        entity_status.lanelet_id, new_s, entity_status.offset, rpy, twist_new, accel_new);
      setOutput("updated_status", entity_status_updated);
      following_trajectory_ = std::vector<geometry_msgs::msg::Point>(0);
      target_status_ = boost::none;
      route_ = boost::none;
      return BT::NodeStatus::SUCCESS;
    }
  }

  if (new_s > hdmap_utils_ptr->getLaneletLength(entity_status.lanelet_id)) {
    new_s = new_s - hdmap_utils_ptr->getLaneletLength(entity_status.lanelet_id);
    boost::optional<int> next_lanelet_id;
    bool is_finded = false;
    for (size_t i = 0; i != route_.get().size(); i++) {
      if (route_.get()[i] == entity_status.lanelet_id) {
        is_finded = true;
        continue;
      }
      if (is_finded && !next_lanelet_id) {
        next_lanelet_id = route_.get()[i];
      }
    }
    if (is_finded && next_lanelet_id) {
      geometry_msgs::msg::Vector3 rpy = entity_status.rpy;
      simulation_controller::entity::EntityStatus entity_status_updated(current_time + step_time,
        next_lanelet_id.get(), new_s, entity_status.offset, rpy, twist_new, accel_new);
      setOutput("updated_status", entity_status_updated);
      setOutput("trajectory", following_trajectory_);
      return BT::NodeStatus::RUNNING;
    } else {
      throw BehaviorTreeRuntimeError("failed to find next lanelet id");
    }
  } else {
    geometry_msgs::msg::Vector3 rpy = entity_status.rpy;
    simulation_controller::entity::EntityStatus entity_status_updated(current_time + step_time,
      entity_status.lanelet_id, new_s, entity_status.offset, rpy, twist_new, accel_new);
    setOutput("updated_status", entity_status_updated);
    setOutput("trajectory", following_trajectory_);
    return BT::NodeStatus::RUNNING;
  }
  setOutput("trajectory", following_trajectory_);
  return BT::NodeStatus::RUNNING;
}
}      // namespace vehicle
}  // namespace entity_behavior
