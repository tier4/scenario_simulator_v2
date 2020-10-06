// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <simulation_controller/behavior/action_node.hpp>

#include <string>
#include <memory>
#include <unordered_map>

namespace entity_behavior
{

ActionNode::ActionNode(const std::string & name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{}

BT::NodeStatus ActionNode::executeTick()
{
  return BT::ActionNodeBase::executeTick();
}

void ActionNode::getBlackBoardValues()
{
  if (!getInput("request", request)) {
    throw BehaviorTreeRuntimeError("failed to get input request in ActionNode");
  }
  if (!getInput<double>("step_time", step_time)) {
    throw BehaviorTreeRuntimeError("failed to get input step_time in ActionNode");
  }
  if (!getInput<double>("current_time", current_time)) {
    throw BehaviorTreeRuntimeError("failed to get input current_time in ActionNode");
  }
  if (!getInput<std::shared_ptr<hdmap_utils::HdMapUtils>>("hdmap_utils", hdmap_utils)) {
    throw BehaviorTreeRuntimeError("failed to get input hdmap_utils in ActionNode");
  }

  if (!getInput<simulation_controller::entity::EntityStatus>("entity_status", entity_status)) {
    throw BehaviorTreeRuntimeError("failed to get input entity_status in ActionNode");
  }

  if (!getInput<boost::optional<double>>("target_speed", target_speed)) {
    target_speed = boost::none;
  }

  if (!getInput<std::unordered_map<std::string,
    simulation_controller::entity::EntityStatus>>("other_entity_status", other_entity_status))
  {
    throw BehaviorTreeRuntimeError("failed to get input other_entity_status in ActionNode");
  }
  if (!getInput<std::unordered_map<std::string,
    simulation_controller::entity::EntityType>>("entity_type_list", entity_type_list))
  {
    throw BehaviorTreeRuntimeError("failed to get input entity_type_list in ActionNode");
  }
}
}  // namespace entity_behavior
