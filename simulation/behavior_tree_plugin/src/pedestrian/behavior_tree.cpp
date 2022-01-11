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
#include <memory>
#include <string>
#include <utility>

namespace entity_behavior
{
void PedestrianBehaviorTree::configure(const rclcpp::Logger & logger)
{
  std::string path = ament_index_cpp::get_package_share_directory("behavior_tree_plugin") +
                     "/config/pedestrian_entity_behavior.xml";
  factory_.registerNodeType<entity_behavior::pedestrian::FollowLaneAction>("FollowLane");
  factory_.registerNodeType<entity_behavior::pedestrian::WalkStraightAction>("WalkStraightAction");
  tree_ = factory_.createTreeFromFile(path);
  logging_event_ptr_ =
    std::make_unique<behavior_tree_plugin::LoggingEvent>(tree_.rootNode(), logger);
  reset_request_event_ptr_ = std::make_unique<behavior_tree_plugin::ResetRequestEvent>(
    tree_.rootNode(), [&]() { return getRequest(); },
    [&](std::string request) { return setRequest(request); });
  setRequest("none");
}

const std::string & PedestrianBehaviorTree::getCurrentAction() const
{
  return logging_event_ptr_->getCurrentAction();
}

void PedestrianBehaviorTree::update(double current_time, double step_time)
{
  tickOnce(current_time, step_time);
  while (getCurrentAction() == "root") {
    tickOnce(current_time, step_time);
  }
}

BT::NodeStatus PedestrianBehaviorTree::tickOnce(double current_time, double step_time)
{
  setCurrentTime(current_time);
  setStepTime(step_time);
  return tree_.rootNode()->executeTick();
}
}  // namespace entity_behavior

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(entity_behavior::PedestrianBehaviorTree, entity_behavior::BehaviorPluginBase)
