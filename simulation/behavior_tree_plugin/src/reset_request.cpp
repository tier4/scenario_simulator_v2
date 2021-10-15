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

#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <behavior_tree_plugin/reset_request.hpp>

namespace behavior_tree_plugin
{
ResetRequest::ResetRequest(const std::shared_ptr<BT::Tree> & tree_ptr)
: TransitionEvent(tree_ptr_), tree_ptr_(tree_ptr)
{
}

void ResetRequest::callback(
  BT::Duration timestamp, const BT::TreeNode & node, BT::NodeStatus prev_status,
  BT::NodeStatus status)
{
}
}  // namespace behavior_tree_plugin
