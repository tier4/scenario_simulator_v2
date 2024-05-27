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

#include <quaternion_operation/quaternion_operation.h>

#include <algorithm>
#include <behavior_tree_plugin/pedestrian/follow_lane_action.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <traffic_simulator/utils/route.hpp>
#include <vector>

namespace entity_behavior
{
namespace pedestrian
{
FollowLaneAction::FollowLaneAction(const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::PedestrianActionNode(name, config)
{
}

void FollowLaneAction::getBlackBoardValues() { PedestrianActionNode::getBlackBoardValues(); }

BT::NodeStatus FollowLaneAction::tick()
{
  getBlackBoardValues();
  if (
    request != traffic_simulator::behavior::Request::NONE &&
    request != traffic_simulator::behavior::Request::FOLLOW_LANE) {
    return BT::NodeStatus::FAILURE;
  }
  if (!entity_status->laneMatchingSucceed()) {
    stopEntity();
    setOutput(
      "non_canonicalized_updated_status",
      std::make_shared<traffic_simulator::EntityStatus>(
        static_cast<traffic_simulator::EntityStatus>(*entity_status)));
    return BT::NodeStatus::RUNNING;
  }
  if (!target_speed) {
    const auto following_lanelets =
      traffic_simulator::route::followingLanelets(entity_status->getLaneletId());
    target_speed = traffic_simulator::route::speedLimit(following_lanelets);
  }
  setOutput(
    "non_canonicalized_updated_status", std::make_shared<traffic_simulator::EntityStatus>(
                                          calculateUpdatedEntityStatus(target_speed.value())));
  return BT::NodeStatus::RUNNING;
}
}  // namespace pedestrian
}  // namespace entity_behavior
