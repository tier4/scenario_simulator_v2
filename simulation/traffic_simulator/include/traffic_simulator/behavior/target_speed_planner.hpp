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

#ifndef BEHAVIOR_TREE_PLUGIN__TARGET_SPEED_PLANNER_HPP_
#define BEHAVIOR_TREE_PLUGIN__TARGET_SPEED_PLANNER_HPP_

#include <boost/optional.hpp>
#include <traffic_simulator/data_type/data_types.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <unordered_map>

namespace traffic_simulator
{
namespace behavior
{
class TargetSpeedPlanner
{
public:
  void setTargetSpeed(double target_speed, bool continuous);
  void setTargetSpeed(const RelativeTargetSpeed & target_speed, bool continuous);
  void update(
    double current_speed,
    const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus> &
      other_status);
  boost::optional<double> getTargetSpeed() const;

private:
  boost::optional<double> target_speed_;
  boost::optional<RelativeTargetSpeed> relative_target_speed_;
  bool continuous_;
  std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus> other_status_;
};
}  // namespace behavior
}  // namespace traffic_simulator

#endif  // BEHAVIOR_TREE_PLUGIN__TARGET_SPEED_PLANNER_HPP_
