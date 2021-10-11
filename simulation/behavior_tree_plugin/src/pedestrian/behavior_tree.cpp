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
  configure();
}
}  // namespace pedestrian
}  // namespace entity_behavior
