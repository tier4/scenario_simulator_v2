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
  if (canonicalized_entity_status_->getTwist().linear.x >= 0) {
    return traffic_simulator_msgs::msg::WaypointsArray();
  } else if (
    const auto canonicalized_lanelet_pose =
      canonicalized_entity_status_->getCanonicalizedLaneletPose()) {
    return traffic_simulator_msgs::build<traffic_simulator_msgs::msg::WaypointsArray>().waypoints(
      traffic_simulator::route::moveBackPoints(canonicalized_lanelet_pose.value()));
  } else {
    THROW_SIMULATION_ERROR(
      "Cannot move backward along lanelet - entity ",
      std::quoted(canonicalized_entity_status_->getName()), " has invalid lanelet pose.");
  }
}

void MoveBackwardAction::getBlackBoardValues() { VehicleActionNode::getBlackBoardValues(); }

bool MoveBackwardAction::checkPreconditions()
{
  if (
    request_ != traffic_simulator::behavior::Request::NONE &&
    request_ != traffic_simulator::behavior::Request::FOLLOW_LANE) {
    return false;
  } else if (!canonicalized_entity_status_->isInLanelet()) {
    return false;
  } else {
    return true;
  }
}

BT::NodeStatus MoveBackwardAction::doAction()
{
  const auto waypoints = calculateWaypoints();
  if (waypoints.waypoints.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!target_speed_) {
    target_speed_ = traffic_simulator::route::speedLimit(
      traffic_simulator::route::previousLanelets(canonicalized_entity_status_->getLaneletId()));
  }

  setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed_.value()));
  setOutput("waypoints", waypoints);
  setOutput("obstacle", calculateObstacle(waypoints));
  return BT::NodeStatus::RUNNING;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
