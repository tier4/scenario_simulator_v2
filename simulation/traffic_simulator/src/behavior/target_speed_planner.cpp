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

#include <iostream>
#include <traffic_simulator/behavior/target_speed_planner.hpp>

namespace traffic_simulator
{
namespace behavior
{
void TargetSpeedPlanner::setTargetSpeed(double target_speed, bool continuous)
{
  continuous_ = continuous;
  target_speed_ = target_speed;
  relative_target_speed_ = boost::none;
}

void TargetSpeedPlanner::setTargetSpeed(const RelativeTargetSpeed & target_speed, bool continuous)
{
  continuous_ = continuous;
  relative_target_speed_ = target_speed;
  target_speed_ = boost::none;
}

void TargetSpeedPlanner::update(
  double current_speed,
  const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityStatus> & other_status)
{
  other_status_ = other_status;
  if (!continuous_ && target_speed_) {
    if (current_speed >= target_speed_.get()) {
      target_speed_ = boost::none;
    }
  }
  if (!continuous_ && relative_target_speed_) {
    if (current_speed >= relative_target_speed_->getAbsoluteValue(other_status_)) {
      relative_target_speed_ = boost::none;
    }
  }
}

boost::optional<double> TargetSpeedPlanner::getTargetSpeed() const
{
  if (target_speed_ && relative_target_speed_) {
    THROW_SIMULATION_ERROR(
      "target_speed and relative_target_speed should not have value at the same time.");
  }
  if (target_speed_) {
    return target_speed_.get();
  }
  if (relative_target_speed_) {
    return relative_target_speed_->getAbsoluteValue(other_status_);
  }
  return boost::none;
}
}  // namespace behavior
}  // namespace traffic_simulator
