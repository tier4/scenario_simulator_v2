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

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behavior_tree_plugin/pedestrian/behavior_tree.hpp>
#include <iostream>
#include <string>
#include <utility>

namespace entity_behavior
{
namespace pedestrian
{
BehaviorTree::BehaviorTree()
{
  std::string path = ament_index_cpp::get_package_share_directory("traffic_simulator") +
                     "/resource/pedestrian_entity_behavior.xml";
  factory_.registerNodeType<entity_behavior::pedestrian::FollowLaneAction>("FollowLane");
  factory_.registerNodeType<entity_behavior::pedestrian::WalkStraightAction>("WalkStraightAction");
  tree_ = factory_.createTreeFromFile(path);
  current_action_ = "root";
  // logger_cout_ptr_ = std::make_shared<BT::StdCoutLogger>(tree_);
  setupLogger();
  setRequest("none");
}

void BehaviorTree::setupLogger()
{
  first_timestamp_ = std::chrono::high_resolution_clock::now();
  auto subscribeCallback = [this](
                             BT::TimePoint timestamp, const BT::TreeNode & node,
                             BT::NodeStatus prev, BT::NodeStatus status) {
    if (status != BT::NodeStatus::IDLE) {
      if (type_ == BT::TimestampType::ABSOLUTE) {
        this->callback(timestamp.time_since_epoch(), node, prev, status);
      } else {
        this->callback(timestamp - first_timestamp_, node, prev, status);
      }
    }
  };
  auto visitor = [this, subscribeCallback](BT::TreeNode * node) {
    subscribers_.push_back(node->subscribeToStatusChange(std::move(subscribeCallback)));
  };
  BT::applyRecursiveVisitor(tree_.rootNode(), visitor);
}

BT::NodeStatus BehaviorTree::tickOnce(double current_time, double step_time)
{
  setValueToBlackBoard("current_time", current_time);
  setValueToBlackBoard("step_time", step_time);
  return tree_.rootNode()->executeTick();
}

void BehaviorTree::tick(double current_time, double step_time)
{
  tickOnce(current_time, step_time);
  while (getCurrentAction() == "root") {
    tickOnce(current_time, step_time);
  }
}

void BehaviorTree::callback(
  BT::Duration timestamp, const BT::TreeNode & node, BT::NodeStatus prev_status,
  BT::NodeStatus status)
{
  constexpr const char * whitespaces = "                         ";
  constexpr const size_t ws_count = 25;
  double since_epoch = std::chrono::duration<double>(timestamp).count();
  printf(
    "[%.3f]: %s%s %s -> %s", since_epoch, node.name().c_str(),
    &whitespaces[std::min(ws_count, node.name().size())], toStr(prev_status, true).c_str(),
    toStr(status, true).c_str());
  std::cout << std::endl;
  if (status != BT::NodeStatus::SUCCESS) {
    current_action_ = node.name();
  }
  if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
    if (request_ == current_action_) {
      setRequest("none");
    }
  }
}
}  // namespace pedestrian
}  // namespace entity_behavior
