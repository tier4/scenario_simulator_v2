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
#include <optional>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/utils/pose.hpp>

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
  -> traffic_simulator::EntityStatus
{
  return ActionNode::calculateUpdatedEntityStatus(
    target_speed, behavior_parameter.dynamic_constraints);
}

auto PedestrianActionNode::calculateUpdatedEntityStatusInWorldFrame(double target_speed) const
  -> traffic_simulator::EntityStatus
{
  auto entity_status_updated = ActionNode::calculateUpdatedEntityStatusInWorldFrame(
    target_speed, behavior_parameter.dynamic_constraints);
  if (
    const auto canonicalized_lanelet_pose =
      traffic_simulator::pose::pedestrian::transformToCanonicalizedLaneletPose(
        entity_status_updated.pose, canonicalized_entity_status->getBoundingBox(),
        canonicalized_entity_status->getLaneletIds(), true,
        default_matching_distance_for_lanelet_pose_calculation, hdmap_utils)) {
    entity_status_updated.lanelet_pose_valid = true;
    entity_status_updated.lanelet_pose =
      static_cast<traffic_simulator::LaneletPose>(canonicalized_lanelet_pose.value());
  } else {
    entity_status_updated.lanelet_pose_valid = false;
    entity_status_updated.lanelet_pose = traffic_simulator::LaneletPose();
  }
  return entity_status_updated;
}
}  // namespace entity_behavior
