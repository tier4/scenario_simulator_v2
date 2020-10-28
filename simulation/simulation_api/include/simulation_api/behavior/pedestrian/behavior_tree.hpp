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

#ifndef SIMULATION_API__BEHAVIOR__PEDESTRIAN__BEHAVIOR_TREE_HPP_
#define SIMULATION_API__BEHAVIOR__PEDESTRIAN__BEHAVIOR_TREE_HPP_

#include <simulation_api/hdmap_utils/hdmap_utils.hpp>
#include <simulation_api/entity/entity_status.hpp>
#include <simulation_api/behavior/pedestrian/follow_lane_action.hpp>
#include <simulation_api/behavior/pedestrian/acquire_position_action.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

#include <memory>
#include <functional>
#include <map>
#include <vector>
#include <string>

namespace entity_behavior
{
namespace pedestrian
{
class BehaviorTree
{
public:
  BehaviorTree();
  BT::NodeStatus tick(double current_time, double step_time);
  const std::string getCurrentAction() const
  {
    return current_action_;
  }
  template<typename T>
  void setValueToBlackBoard(std::string key, T value)
  {
    tree_.rootBlackboard()->set(key, value);
  }
  simulation_api::entity::EntityStatus getUpdatedStatus()
  {
    simulation_api::entity::EntityStatus status;
    tree_.rootBlackboard()->get("updated_status", status);
    return status;
  }
  void setRequest(std::string request);

private:
  std::string request_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::shared_ptr<BT::StdCoutLogger> logger_cout_ptr_;
  void callback(
    BT::Duration timestamp, const BT::TreeNode & node, BT::NodeStatus prev_status,
    BT::NodeStatus status);
  void setupLogger();
  BT::TimestampType type_;
  BT::TimePoint first_timestamp_;
  std::vector<BT::TreeNode::StatusChangeSubscriber> subscribers_;
  std::string current_action_;
};
}      // namespace pedestrian
}  // namespace entity_behavior

#endif  // SIMULATION_API__BEHAVIOR__PEDESTRIAN__BEHAVIOR_TREE_HPP_
