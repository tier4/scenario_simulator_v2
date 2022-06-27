// Copyright 2015 TIER IV, Inc. All rights reserved.
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
SimulationClock::SimulationClock(rcl_clock_type_t clock_type, bool use_raw_clock)
: rclcpp::Clock(clock_type),
  use_raw_clock(use_raw_clock),
  step_time_duration_(rclcpp::Duration::from_seconds(0)),
  initialized_(false),
  is_npc_logic_started_(false)
{
}

void SimulationClock::initialize(double initial_simulation_time, double step_time)
{
  initialized_ = true;
  initial_simulation_time_ = initial_simulation_time;
  current_simulation_time_ = initial_simulation_time_;
  step_time_ = step_time;
  step_time_duration_ = rclcpp::Duration::from_seconds(step_time_);
  time_on_initialize_ = now();
}

void SimulationClock::update()
{
  if (!initialized_) {
    THROW_SIMULATION_ERROR("SimulationClock has not been initialized yet.");
  }
  current_simulation_time_ = current_simulation_time_ + step_time_;
}

const rosgraph_msgs::msg::Clock SimulationClock::getCurrentRosTimeAsMsg()
{
  rosgraph_msgs::msg::Clock clock;
  clock.clock = getCurrentRosTime();
  return clock;
}

const rclcpp::Time SimulationClock::getCurrentRosTime()
{
  if (!initialized_) {
    THROW_SIMULATION_ERROR("SimulationClock has not been initialized yet.");
  }
  if (use_raw_clock) {
    return now();
  } else {
    return time_on_initialize_ +
           rclcpp::Duration::from_seconds(current_simulation_time_ - initial_simulation_time_);
  }
}

void SimulationClock::onNpcLogicStart()
{
  if (is_npc_logic_started_) {
    THROW_SIMULATION_ERROR(
      "npc logic is already started. Please check simulation clock instance was destroyed.");
  }
  is_npc_logic_started_ = true;
  scenario_time_offset_ = getCurrentSimulationTime();
}

boost::optional<double> SimulationClock::getCurrentScenarioTime() const
{
  if (!is_npc_logic_started_) {
    return boost::none;
  }
  return getCurrentSimulationTime() - scenario_time_offset_;
}
}  // namespace traffic_simulator
