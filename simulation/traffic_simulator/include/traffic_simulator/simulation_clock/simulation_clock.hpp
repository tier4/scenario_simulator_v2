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

#ifndef TRAFFIC_SIMULATOR__SIMULATION_CLOCK__SIMULATION_CLOCK_HPP_
#define TRAFFIC_SIMULATOR__SIMULATION_CLOCK__SIMULATION_CLOCK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace traffic_simulator
{
class SimulationClock : rclcpp::Clock
{
public:
  explicit SimulationClock(double realtime_factor, double frame_rate);

  auto getCurrentRosTime() -> rclcpp::Time;

  auto getCurrentRosTimeAsMsg() -> rosgraph_msgs::msg::Clock;

  auto getCurrentScenarioTime() const { return (frame_ - frame_offset_) / frame_rate * realtime_factor; }

  auto getCurrentSimulationTime() const { return frame_ / frame_rate * realtime_factor; }

  auto getStepTime() const { return realtime_factor / frame_rate; }

  auto start() -> void;

  auto started() const { return not std::isnan(frame_offset_); }

  auto update() -> void;

  const bool use_raw_clock;

  double realtime_factor;

  double frame_rate;

  const rclcpp::Time time_on_initialize;

private:
  double frame_ = 0;

  double frame_offset_ = std::numeric_limits<double>::quiet_NaN();
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__SIMULATION_CLOCK__SIMULATION_CLOCK_HPP_
