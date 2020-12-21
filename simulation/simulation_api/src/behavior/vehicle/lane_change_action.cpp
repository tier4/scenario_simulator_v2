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

#include <simulation_api/behavior/vehicle/behavior_tree.hpp>
#include <simulation_api/behavior/vehicle/lane_change_action.hpp>
#include <simulation_api/entity/vehicle_parameter.hpp>

#include <string>
#include <memory>

namespace entity_behavior
{
namespace vehicle
{
LaneChangeAction::LaneChangeAction(const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config) {}

const openscenario_msgs::msg::CatmullRomSpline LaneChangeAction::calculateTrajectory() const
{

}

BT::NodeStatus LaneChangeAction::tick()
{
  std::string request;
  if (!getInput("request", request)) {
    throw BehaviorTreeRuntimeError("failed to get input request in LaneChangeAction");
  }
  if (request != "lane_change") {
    curve_ = boost::none;
    current_s_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  LaneChangeParameter params;
  if (!getInput<LaneChangeParameter>("lane_change_params", params)) {
    throw BehaviorTreeRuntimeError("failed to get input lane_change_params in LaneChangeAction");
  }

  std::shared_ptr<simulation_api::entity::VehicleParameters> vehicle_params_ptr;
  if (!getInput<std::shared_ptr<simulation_api::entity::VehicleParameters>>(
      "vehicle_parameters", vehicle_params_ptr))
  {
    throw BehaviorTreeRuntimeError("failed to get input vehicle_parameters in LaneChangeAction");
  }

  double step_time, current_time;
  if (!getInput<double>("step_time", step_time)) {
    throw BehaviorTreeRuntimeError("failed to get input step_time in LaneChangeAction");
  }
  if (!getInput<double>("current_time", current_time)) {
    throw BehaviorTreeRuntimeError("failed to get input current_time in LaneChangeAction");
  }

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr;
  if (!getInput<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils", hdmap_utils_ptr)) {
    throw BehaviorTreeRuntimeError("failed to get input hdmap_utils in LaneChangeAction");
  }

  openscenario_msgs::msg::EntityStatus entity_status;
  if (!getInput<openscenario_msgs::msg::EntityStatus>("entity_status", entity_status)) {
    throw BehaviorTreeRuntimeError("failed to get input entity_status in LaneChangeAction");
  }

  if (!curve_) {
    if (request == "lane_change") {
      if (!hdmap_utils_ptr->canChangeLane(entity_status.lanelet_pose.lanelet_id,
        params.to_lanelet_id))
      {
        return BT::NodeStatus::FAILURE;
      }
      auto from_pose = hdmap_utils_ptr->toMapPose(entity_status.lanelet_pose).pose;
      auto ret = hdmap_utils_ptr->getLaneChangeTrajectory(from_pose, params.to_lanelet_id);
      if (ret) {
        curve_ = ret->first;
        target_s_ = ret->second;
      } else {
        return BT::NodeStatus::FAILURE;
      }
    }
  }
  if (curve_) {
    double current_linear_vel = entity_status.action_status.twist.linear.x;
    current_s_ = current_s_ + current_linear_vel * step_time;
    if (current_s_ < curve_->getLength()) {
      geometry_msgs::msg::Pose pose = curve_->getPose(current_s_, true);
      openscenario_msgs::msg::EntityStatus entity_status_updated;
      entity_status_updated.pose = pose;
      auto lanelet_pose = hdmap_utils_ptr->toLaneletPose(pose);
      if (lanelet_pose) {
        entity_status_updated.lanelet_pose = lanelet_pose.get();
      } else {
        entity_status_updated.lanelet_pose_valid = false;
      }
      entity_status_updated.action_status = entity_status.action_status;
      setOutput("updated_status", entity_status_updated);
      return BT::NodeStatus::RUNNING;
    } else {
      double s = (current_s_ - curve_->getLength()) + target_s_;
      curve_ = boost::none;
      current_s_ = 0;
      openscenario_msgs::msg::EntityStatus entity_status_updated;
      openscenario_msgs::msg::LaneletPose lanelet_pose;
      lanelet_pose.lanelet_id = params.to_lanelet_id;
      lanelet_pose.s = s;
      lanelet_pose.offset = 0;
      entity_status_updated.pose = hdmap_utils_ptr->toMapPose(lanelet_pose).pose;
      entity_status_updated.lanelet_pose = lanelet_pose;
      entity_status_updated.action_status = entity_status.action_status;
      setOutput("updated_status", entity_status_updated);
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;
}
}      // namespace vehicle
}  // namespace entity_behavior
