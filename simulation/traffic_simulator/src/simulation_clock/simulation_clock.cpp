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

#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/simulation_clock/simulation_clock.hpp>

namespace traffic_simulator
{
SimulationClock::SimulationClock()
: rclcpp::Clock(RCL_SYSTEM_TIME), step_time_duration_(0), initialized_(false)
{
}

void SimulationClock::initialize(double initial_simulation_time, double step_time)
{
  initialized_ = true;
  initial_simulation_time_ = initial_simulation_time;
  current_simulation_time_ = initial_simulation_time_;
  step_time_ = step_time;
  step_time_duration_ = rclcpp::Duration::from_seconds(step_time_);
  system_time_on_initialize_ = now();
}

void SimulationClock::update()
{
  if (!initialized_) {
    THROW_SIMULATION_ERROR("SimulationClock does not initialized yet.");
  }
  current_simulation_time_ = current_simulation_time_ + step_time_;
}

rosgraph_msgs::msg::Clock SimulationClock::getCurrentRosTimeAsMsg() const
{
  rosgraph_msgs::msg::Clock clock;
  clock.clock = getCurrentRosTime();
  return clock;
}

rclcpp::Time SimulationClock::getCurrentRosTime() const
{
  if (!initialized_) {
    THROW_SIMULATION_ERROR("SimulationClock does not initialized yet.");
  }
  return system_time_on_initialize_ +
         rclcpp::Duration::from_seconds(current_simulation_time_ - initial_simulation_time_);
}
}  // namespace traffic_simulator