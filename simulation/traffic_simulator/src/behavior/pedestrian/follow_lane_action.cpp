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

#include <quaternion_operation/quaternion_operation.h>

#include <boost/algorithm/clamp.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <traffic_simulator/behavior/pedestrian/follow_lane_action.hpp>
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
  if (request != "none" && request != "follow_lane") {
    return BT::NodeStatus::FAILURE;
  }
  if (!entity_status.lanelet_pose_valid) {
    setOutput("updated_status", stopAtEndOfRoad());
    return BT::NodeStatus::RUNNING;
  }
  auto following_lanelets =
    hdmap_utils->getFollowingLanelets(entity_status.lanelet_pose.lanelet_id);
  if (!target_speed) {
    target_speed = hdmap_utils->getSpeedLimit(following_lanelets);
  }
  setOutput("updated_status", calculateEntityStatusUpdated(target_speed.get()));
  return BT::NodeStatus::RUNNING;
}
}  // namespace pedestrian
}  // namespace entity_behavior
