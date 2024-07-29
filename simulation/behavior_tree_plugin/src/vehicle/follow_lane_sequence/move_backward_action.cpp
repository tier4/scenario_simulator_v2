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
  if (!canonicalized_entity_status->laneMatchingSucceed()) {
    THROW_SIMULATION_ERROR("failed to assign lane");
  }
  if (canonicalized_entity_status->getTwist().linear.x >= 0) {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }
  const auto lanelet_pose = canonicalized_entity_status->getLaneletPose();
  const auto ids = hdmap_utils->getPreviousLanelets(lanelet_pose.lanelet_id);
  // DIFFERENT SPLINE - recalculation needed
  math::geometry::CatmullRomSpline spline(hdmap_utils->getCenterPoints(ids));
  double s_in_spline = 0;
  for (const auto id : ids) {
    if (id == lanelet_pose.lanelet_id) {
      s_in_spline = s_in_spline + lanelet_pose.s;
      break;
    } else {
      s_in_spline = hdmap_utils->getLaneletLength(id) + s_in_spline;
    }
  }
  traffic_simulator_msgs::msg::WaypointsArray waypoints;
  waypoints.waypoints =
    spline.getTrajectory(s_in_spline, s_in_spline - 5, 1.0, lanelet_pose.offset);
  return waypoints;
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
  if (!canonicalized_entity_status->laneMatchingSucceed()) {
    return BT::NodeStatus::FAILURE;
  }
  const auto waypoints = calculateWaypoints();
  if (waypoints.waypoints.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!target_speed) {
    target_speed = hdmap_utils->getSpeedLimit(
      hdmap_utils->getPreviousLanelets(canonicalized_entity_status->getLaneletId()));
  }

  setCanonicalizedEntityStatus(calculateUpdatedEntityStatus(target_speed.value()));
  setOutput("waypoints", waypoints);
  setOutput("obstacle", calculateObstacle(waypoints));
  return BT::NodeStatus::RUNNING;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
