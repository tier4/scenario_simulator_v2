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

#include <traffic_simulator/behavior/vehicle/follow_lane_sequence/move_backward_action.hpp>

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

const boost::optional<openscenario_msgs::msg::Obstacle> MoveBackwardAction::calculateObstacle(
  const openscenario_msgs::msg::WaypointsArray &)
{
  return boost::none;
}

const openscenario_msgs::msg::WaypointsArray MoveBackwardAction::calculateWaypoints()
{
  if (!entity_status.lanelet_pose_valid) {
    THROW_SIMULATION_ERROR("failed to assign lane");
  }
  if (entity_status.action_status.twist.linear.x >= 0) {
    return openscenario_msgs::msg::WaypointsArray();
  }
  const auto ids = hdmap_utils->getPreviousLanelets(entity_status.lanelet_pose.lanelet_id);
  traffic_simulator::math::CatmullRomSpline spline(hdmap_utils->getCenterPoints(ids));
  double s_in_spline = 0;
  for (const auto id : ids) {
    if (id == entity_status.lanelet_pose.lanelet_id) {
      s_in_spline = s_in_spline + entity_status.lanelet_pose.s;
      break;
    } else {
      s_in_spline = hdmap_utils->getLaneletLength(id) + s_in_spline;
    }
  }
  openscenario_msgs::msg::WaypointsArray waypoints;
  waypoints.waypoints = spline.getTrajectory(s_in_spline, s_in_spline - 5, 1.0);
  return waypoints;
}

void MoveBackwardAction::getBlackBoardValues() { VehicleActionNode::getBlackBoardValues(); }

BT::NodeStatus MoveBackwardAction::tick()
{
  getBlackBoardValues();
  if (request != "none" && request != "follow_lane") {
    return BT::NodeStatus::FAILURE;
  }
  if (!entity_status.lanelet_pose_valid) {
    return BT::NodeStatus::FAILURE;
  }
  const auto waypoints = calculateWaypoints();
  if (waypoints.waypoints.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("updated_status", calculateEntityStatusUpdated(target_speed.get()));
  const auto obstacle = calculateObstacle(waypoints);
  setOutput("waypoints", waypoints);
  setOutput("obstacle", obstacle);
  return BT::NodeStatus::RUNNING;
}
}  // namespace follow_lane_sequence
}  // namespace vehicle
}  // namespace entity_behavior
