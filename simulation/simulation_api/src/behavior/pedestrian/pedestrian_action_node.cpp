// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#include <simulation_api/behavior/pedestrian/pedestrian_action_node.hpp>

#include <memory>
#include <string>

namespace entity_behavior
{
PedestrianActionNode::PedestrianActionNode(
  const std::string & name,
  const BT::NodeConfiguration & config)
: ActionNode(name, config) {}

void PedestrianActionNode::getBlackBoardValues()
{
  ActionNode::getBlackBoardValues();
  if (!getInput<std::shared_ptr<simulation_api::entity::PedestrianParameters>>(
      "pedestrian_parameters", pedestrian_parameters))
  {
    throw BehaviorTreeRuntimeError(
            "failed to get input pedestrian_parameters in PedestrianActionNode");
  }
}
}  // namespace entity_behavior
