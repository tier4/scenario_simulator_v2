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

#ifndef TRAFFIC_SIMULATOR__BEHAVIOR__BEHAVIOR_PLUGIN_BASE_HPP_
#define TRAFFIC_SIMULATOR__BEHAVIOR__BEHAVIOR_PLUGIN_BASE_HPP_

#include <openscenario_msgs/msg/entity_status.hpp>
#include <string>

namespace entity_behavior
{
class BehaviorPluginBase
{
private:
  /* data */
public:
  BehaviorPluginBase(/* args */);
  ~BehaviorPluginBase();
};

BehaviorPluginBase::BehaviorPluginBase(/* args */) {}

BehaviorPluginBase::~BehaviorPluginBase() {}
template <typename T>
void setValueToBlackBoard(std::string key, T value)
{
  tree_.rootBlackboard()->set(key, value);
}
openscenario_msgs::msg::EntityStatus getUpdatedStatus()
{
  openscenario_msgs::msg::EntityStatus status;
  tree_.rootBlackboard()->get("updated_status", status);
  return status;
}
}  // namespace entity_behavior

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__BEHAVIOR_PLUGIN_BASE_HPP_
