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
  -> traffic_simulator::CanonicalizedEntityStatus
{
  return ActionNode::calculateUpdatedEntityStatus(
    target_speed, behavior_parameter.dynamic_constraints);
}

auto PedestrianActionNode::calculateUpdatedEntityStatusInWorldFrame(double target_speed) const
  -> traffic_simulator::CanonicalizedEntityStatus
{
  auto updated_status = static_cast<traffic_simulator::EntityStatus>(
    ActionNode::calculateUpdatedEntityStatusInWorldFrame(
      target_speed, behavior_parameter.dynamic_constraints));
  const auto lanelet_pose = estimateLaneletPose(updated_status.pose);
  if (lanelet_pose) {
    updated_status.lanelet_pose_valid = true;
    updated_status.lanelet_pose = static_cast<traffic_simulator::LaneletPose>(lanelet_pose.value());
  } else {
    updated_status.lanelet_pose_valid = false;
    updated_status.lanelet_pose = traffic_simulator::LaneletPose();
  }
  return traffic_simulator::CanonicalizedEntityStatus(updated_status, hdmap_utils);
}

auto PedestrianActionNode::estimateLaneletPose(const geometry_msgs::msg::Pose & pose) const
  -> std::optional<traffic_simulator::CanonicalizedLaneletPose>
{
  std::optional<traffic_simulator::LaneletPose> lanelet_pose;
  if (entity_status->laneMatchingSucceed()) {
    /**
     * @note In this branch, try to matching pedestrian entity to specified lanelet_id.
    */
    lanelet_pose = hdmap_utils->toLaneletPose(
      pose, entity_status->getLaneletPose().lanelet_id,
      default_matching_distance_for_lanelet_pose_calculation);
  } else {
    /**
     * @note In this branch, try to matching pedestrian entity considering bounding box.
     * true means considering crosswalk.
    */
    lanelet_pose = hdmap_utils->toLaneletPose(
      pose, entity_status->getBoundingBox(), true,
      default_matching_distance_for_lanelet_pose_calculation);
  }
  if (!lanelet_pose) {
    /**
     * @note Hard coded parameter. 2.0 is a matching threshold for lanelet.
     * true means considering crosswalk.
     * In this branch, the algorithm only consider entity pose.
    */
    lanelet_pose = hdmap_utils->toLaneletPose(pose, true, 2.0);
  }
  if (lanelet_pose) {
    const auto canonicalized = hdmap_utils->canonicalizeLaneletPose(lanelet_pose.value());
    if (
      const auto canonicalized_lanelet_pose =
        std::get<std::optional<traffic_simulator::LaneletPose>>(canonicalized)) {
      /// @note If canonicalize succeed, set canonicalized pose and set other values.
      return traffic_simulator::CanonicalizedLaneletPose(lanelet_pose.value(), hdmap_utils);
    } else {
      /// @note If canonicalize failed, set end of road lanelet pose.
      if (const auto end_of_road_lanelet_id = std::get<std::optional<lanelet::Id>>(canonicalized)) {
        if (lanelet_pose.value().s < 0) {
          return traffic_simulator::CanonicalizedLaneletPose(
            traffic_simulator_msgs::build<traffic_simulator::LaneletPose>()
              .lanelet_id(end_of_road_lanelet_id.value())
              .s(0.0)
              .offset(lanelet_pose.value().offset)
              .rpy(lanelet_pose.value().rpy),
            hdmap_utils);
        } else {
          return traffic_simulator::CanonicalizedLaneletPose(
            traffic_simulator_msgs::build<traffic_simulator::LaneletPose>()
              .lanelet_id(end_of_road_lanelet_id.value())
              .s(hdmap_utils->getLaneletLength(end_of_road_lanelet_id.value()))
              .offset(lanelet_pose.value().offset)
              .rpy(lanelet_pose.value().rpy),
            hdmap_utils);
        }
      } else {
        THROW_SIMULATION_ERROR("Failed to find trailing lanelet_id.");
      }
    }
  } else {
    return std::nullopt;
  }
}
}  // namespace entity_behavior
