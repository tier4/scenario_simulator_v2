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

#include <behavior_tree_plugin/pedestrian/pedestrian_action_node.hpp>
#include <memory>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/helper/helper.hpp>

namespace entity_behavior
{
PedestrianActionNode::PedestrianActionNode(
  const std::string & name, const BT::NodeConfiguration & config)
: ActionNode(name, config)
{
}

void PedestrianActionNode::getBlackBoardValues()
{
  ActionNode::getBlackBoardValues();
  if (!getInput<traffic_simulator_msgs::msg::BehaviorParameter>(
        "behavior_parameter", behavior_parameter)) {
    behavior_parameter = traffic_simulator_msgs::msg::BehaviorParameter();
  }
  if (!getInput<traffic_simulator_msgs::msg::PedestrianParameters>(
        "pedestrian_parameters", pedestrian_parameters)) {
    THROW_SIMULATION_ERROR("failed to get input pedestrian_parameters in PedestrianActionNode");
  }
}

auto PedestrianActionNode::calculateUpdatedEntityStatus(double target_speed) const
  -> traffic_simulator_msgs::msg::EntityStatus
{
  return ActionNode::calculateUpdatedEntityStatus(
    target_speed, behavior_parameter.dynamic_constraints);
}

auto PedestrianActionNode::calculateUpdatedEntityStatusInWorldFrame(double target_speed) const
  -> traffic_simulator_msgs::msg::EntityStatus
{
  auto updated_status = ActionNode::calculateUpdatedEntityStatusInWorldFrame(
    target_speed, behavior_parameter.dynamic_constraints);
  const auto lanelet_pose = estimateLaneletPose(updated_status.pose);
  if (lanelet_pose) {
    updated_status.lanelet_pose_valid = true;
    updated_status.lanelet_pose = lanelet_pose.get();
  } else {
    updated_status.lanelet_pose_valid = false;
    updated_status.lanelet_pose = LaneletPoseType();
  }
  return updated_status;
}

auto PedestrianActionNode::estimateLaneletPose(const geometry_msgs::msg::Pose & pose) const
  -> boost::optional<LaneletPoseType>
{
  boost::optional<LaneletPoseType> lanelet_pose;
  if (entity_status.lanelet_pose_valid) {
    lanelet_pose = hdmap_utils->toLaneletPose(pose, entity_status.lanelet_pose.lanelet_id, 1.0);
  } else {
    lanelet_pose = hdmap_utils->toLaneletPose(pose, entity_status.bounding_box, true);
  }
  if (!lanelet_pose) {
    lanelet_pose = hdmap_utils->toLaneletPose(pose, true, 2.0);
  }
  return lanelet_pose;
}
}  // namespace entity_behavior
