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

#include "context_gamma_planner/behavior/pedestrian/follow_polyline_trajectory_action.hpp"

namespace context_gamma_planner::pedestrian
{

FollowPolylineTrajectoryAction::FollowPolylineTrajectoryAction(
  const std::string & name, const BT::NodeConfiguration & config)
: context_gamma_planner::pedestrian::ActionNode(name, config)
{
}

void FollowPolylineTrajectoryAction::getBlackBoardValues()
{
  ActionNode::getBlackBoardValues();
  if (
    request == traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY &&
    !getInput("polyline_trajectory", polyline_trajectory_)) {
    THROW_SIMULATION_ERROR("failed to get input polyline_trajectory in ActionNode");
  }
}

BT::NodeStatus FollowPolylineTrajectoryAction::tick()
{
  getBlackBoardValues();

  if (
    request != traffic_simulator::behavior::Request::NONE &&
    request != traffic_simulator::behavior::Request::FOLLOW_POLYLINE_TRAJECTORY) {
    planner_.clear();
    return BT::NodeStatus::FAILURE;
  }
  if (polyline_trajectory_->shape.vertices.empty()) {
    THROW_SIMULATION_ERROR("polyline_trajectory is empty");
  }
  planner_.setCurrentStatus(entity_status);
  if (target_speed) {
    planner_.setMaxSpeed(target_speed.value());
  }
  if (planner_.getGoalPoses().empty()) {
    planner_.setWaypoints(polyline_trajectory_);
  }
  if (const auto next_goal = planner_.calculateNextGoalPoint()) {
    setOutput("next_goal", next_goal.value());
  } else {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("waypoints", planner_.getWaypoints());
  setOutput("planning_speed", std::optional<double>(planner_.getTargetSpeed(current_time)));

  if (planner_.isReachedGoal()) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}
}  // namespace context_gamma_planner::pedestrian
