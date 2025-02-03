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

#include <behavior_tree_plugin/vehicle/follow_lane_sequence/move_backward_action.hpp>
#include <optional>
#include <traffic_simulator/utils/route.hpp>

namespace entity_behavior
{
namespace vehicle
{
namespace follow_lane_sequence
{
MoveBackwardAction::MoveBackwardAction(
  const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::VehicleActionNode(name, config)
{
}

const std::optional<traffic_simulator_msgs::msg::Obstacle> MoveBackwardAction::calculateObstacle(
  const traffic_simulator_msgs::msg::WaypointsArray &)
{
  return std::nullopt;
}

const traffic_simulator_msgs::msg::WaypointsArray MoveBackwardAction::calculateWaypoints()
{
  if (canonicalized_entity_status->getTwist().linear.x >= 0) {
    return traffic_simulator_msgs::msg::WaypointsArray();
  } else if (
    const auto canonicalized_lanelet_pose =
      canonicalized_entity_status->getCanonicalizedLaneletPose()) {
    return traffic_simulator_msgs::build<traffic_simulator_msgs::msg::WaypointsArray>().waypoints(
      traffic_simulator::route::moveBackPoints(canonicalized_lanelet_pose.value()));
  } else {
    THROW_SIMULATION_ERROR(
      "Cannot move backward along lanelet - entity ",
      std::quoted(canonicalized_entity_status->getName()), " has invalid lanelet pose.");
  }
}

void MoveBackwardAction::getBlackBoardValues() { VehicleActionNode::getBlackBoardValues(); }

BT::NodeStatus MoveBackwardAction::tick()
{
  getBlackBoardValues();
  if (
    request != traffic_simulator::behavior::Request::NONE &&
    request != traffic_simulator::behavior::Request::FOLLOW_LANE) {
    return BT::NodeStatus::FAILURE;
  }
  if (!canonicalized_entity_status->isInLanelet()) {
    return BT::NodeStatus::FAILURE;
  }
  const auto waypoints = calculateWaypoints();
  if (waypoints.waypoints.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!target_speed) {
    target_speed = traffic_simulator::route::speedLimit(
      traffic_simulator::route::previousLanelets(canonicalized_entity_status->getLaneletId()));
  }

  setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed.value()));
  setOutput("waypoints", waypoints);
  setOutput("obstacle", calculateObstacle(waypoints));
  return BT::NodeStatus::RUNNING;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
