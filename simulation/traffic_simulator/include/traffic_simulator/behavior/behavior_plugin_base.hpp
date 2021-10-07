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
#include <traffic_simulator/behavior/black_board.hpp>

namespace entity_behavior
{
class BehaviorPluginBase
{
private:
  BlackBoard black_board_;
  std::string current_action_;

public:
  virtual void update(double current_time, double step_time) = 0;
  template <typename T>
  void setValueToBlackBoard(const std::string & key, const T & value)
  {
    black_board_.set(key, value);
  }
  template <typename T>
  void getValueFromBlackBoard(std::string key, const T & value)
  {
    black_board_.get(key, value);
  }
  const std::string getCurrentAction() const { return current_action_; }
};
}  // namespace entity_behavior

#endif  // TRAFFIC_SIMULATOR__BEHAVIOR__BEHAVIOR_PLUGIN_BASE_HPP_
