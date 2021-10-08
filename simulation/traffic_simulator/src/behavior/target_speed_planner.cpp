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
}

void TargetSpeedPlanner::update(double current_speed)
{
  if (!continuous_ && target_speed_) {
    if (current_speed >= target_speed_.get()) {
      target_speed_ = boost::none;
    }
  }
}

boost::optional<double> TargetSpeedPlanner::getTargetSpeed() const { return target_speed_; }
}  // namespace behavior
}  // namespace traffic_simulator
