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

#include <context_gamma_planner/behavior/vehicle/lane_change_action.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/helper/stop_watch.hpp>
#include <vector>

namespace context_gamma_planner
{
namespace vehicle
{
LaneChangeAction::LaneChangeAction(const std::string & name, const BT::NodeConfiguration & config)
: context_gamma_planner::vehicle::ActionNode(name, config), planner_(5.0)
{
}

void LaneChangeAction::getBlackBoardValues()
{
  ActionNode::getBlackBoardValues();
  if (request == traffic_simulator::behavior::Request::LANE_CHANGE) {
    if (!getInput("lane_change_parameters", lane_change_parameters_)) {
      THROW_SIMULATION_ERROR("failed to get input lane_change_parameters in ActionNode");
    }
  }
}

BT::NodeStatus LaneChangeAction::tick()
{
  getBlackBoardValues();

  if (
    request != traffic_simulator::behavior::Request::NONE &&
    request != traffic_simulator::behavior::Request::LANE_CHANGE) {
    planner_.clear();
    return BT::NodeStatus::FAILURE;
  }

  if (
    lane_change_parameters_.constraint.policy ==
    traffic_simulator::lane_change::Constraint::Policy::FORCE) {
    THROW_SIMULATION_ERROR("Force lane change is not supported in context gamma planner");
  }
  planner_.setCurrentStatus(entity_status);
  if (target_speed) {
    planner_.setMaxSpeed(target_speed.value());
  }
  if (planner_.getGoalPoses().empty()) {
    planner_.setWaypoints(hdmap_utils, route_lanelets, lane_change_parameters_);
  }
  if (const auto next_goal = planner_.calculateNextGoalPoint()) {
    setOutput("next_goal", next_goal.value());
  } else {
    return BT::NodeStatus::SUCCESS;
  }
  setOutput("waypoints", planner_.getWaypoints());
  setOutput("planning_speed", std::optional<double>(planner_.getTargetSpeed()));
  // update constraints
  activator->appendLaneChangeConstraint(entity_status, lane_change_parameters_);
  if (planner_.isReachedGoal()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}
}  // namespace vehicle
}  // namespace context_gamma_planner
