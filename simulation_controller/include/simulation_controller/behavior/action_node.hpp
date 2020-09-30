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

#ifndef SIMULATION_CONTROLLER__BEHAVIOR__ACTION_NODE_HPP_
#define SIMULATION_CONTROLLER__BEHAVIOR__ACTION_NODE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <string>

namespace entity_behavior
{
class BehaviorTreeRuntimeError : public std::runtime_error
{
public:
  explicit BehaviorTreeRuntimeError(const char * message)
  : runtime_error(message) {}
};

class ActionNode : public BT::ActionNodeBase
{
public:
  ActionNode(const std::string & name, const BT::NodeConfiguration & config);
  ~ActionNode() override = default;

  /// throws if the derived class return RUNNING.
  virtual BT::NodeStatus executeTick() override;

  /// You don't need to override this
  virtual void halt() override final
  {
    setStatus(BT::NodeStatus::IDLE);
  }
};
}  // namespace entity_behavior

#endif  // SIMULATION_CONTROLLER__BEHAVIOR__ACTION_NODE_HPP_
