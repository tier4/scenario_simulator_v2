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

#include <context_gamma_planner/behavior/vehicle/follow_lane_action.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/helper/stop_watch.hpp>
#include <vector>

namespace context_gamma_planner
{
namespace vehicle
{
FollowLaneAction::FollowLaneAction(const std::string & name, const BT::NodeConfiguration & config)
: context_gamma_planner::vehicle::ActionNode(name, config), planner_(5.0)
{
}

void FollowLaneAction::getBlackBoardValues() { ActionNode::getBlackBoardValues(); }

BT::NodeStatus FollowLaneAction::tick()
{
  getBlackBoardValues();
  if (
    request != traffic_simulator::behavior::Request::NONE &&
    request != traffic_simulator::behavior::Request::FOLLOW_LANE) {
    planner_.clear();
    return BT::NodeStatus::FAILURE;
  }
  planner_.setCurrentStatus(entity_status);
  // update max speed
  std::optional<double> max_speed = std::nullopt;
  if (target_speed) {
    planner_.setMaxSpeed(target_speed.value());
    max_speed = planner_.getMaxSpeed();
  }
  // update waypoints
  planner_.setWaypoints(hdmap_utils, route_lanelets);
  /// @todo Goal threshold should be defined by adaptive.
  if (const auto next_goal = planner_.calculateNextGoalPoint()) {
    setOutput("next_goal", next_goal.value());
  } else {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("waypoints", planner_.getWaypoints());
  setOutput("planning_speed", max_speed);

  // update constraints
  activator->appendRoadEdgeConstraint(route_lanelets);
  activator->appendStopLineConstraint(route_lanelets, reference_trajectory, entity_status);
  activator->appendTrafficLightConstraint(route_lanelets);
  activator->appendRoadEndConstraint(route_lanelets);
  activator->appendPreviousRoadEdgeConstraint(route_lanelets);
  return BT::NodeStatus::RUNNING;
}
}  // namespace vehicle
}  // namespace context_gamma_planner
