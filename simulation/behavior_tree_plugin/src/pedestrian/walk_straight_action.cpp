/**
 * @file walk_straight_action.cpp
 * @author Masaya Kataoka (masaya.kataoka@tier4.jp)
 * @brief class implementation of the walk straight action
 * @version 0.1
 * @date 2021-04-02
 *
 * @copyright Copyright(c) TIER IV.Inc {2015}
 *
 */

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

#include <behavior_tree_plugin/pedestrian/walk_straight_action.hpp>
#include <string>

namespace entity_behavior
{
namespace pedestrian
{
WalkStraightAction::WalkStraightAction(
  const std::string & name, const BT::NodeConfiguration & config)
: entity_behavior::PedestrianActionNode(name, config)
{
}

void WalkStraightAction::getBlackBoardValues() { PedestrianActionNode::getBlackBoardValues(); }

bool WalkStraightAction::checkPreconditions()
{
  if (request_ != traffic_simulator::behavior::Request::WALK_STRAIGHT) {
    return false;
  } else {
    return true;
  }
}

BT::NodeStatus WalkStraightAction::doAction()
{
  if (!target_speed_) {
    target_speed_ = 1.111;
  }
  setCanonicalizedEntityStatus(calculateUpdatedEntityStatusInWorldFrame(target_speed_.value()));
  return BT::NodeStatus::RUNNING;
}
}  // namespace pedestrian
}  // namespace entity_behavior
