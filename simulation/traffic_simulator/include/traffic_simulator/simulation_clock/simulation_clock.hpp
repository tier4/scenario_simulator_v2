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

  auto getCurrentScenarioTime() const
  {
    auto nanoseconds = nanoseconds_count - nanoseconds_offset;
    auto seconds = seconds_count - seconds_offset;

    if (nanoseconds < 0) {
      seconds -= 1;
      nanoseconds += 1000000000;
    }

    return nanoseconds / 1000000000.0 + seconds;
  }

  auto getCurrentSimulationTime() const { return nanoseconds_count / 1000000000.0 + seconds_count; }

  auto getStepTime() const { return realtime_factor / frame_rate; }

  auto start() -> void;

  auto started() const
  {
    return not std::isnan(nanoseconds_offset) || not std::isnan(seconds_offset);
  }

  auto update() -> void;

  auto setRealTimeFactor(double realtime_factor_) { realtime_factor = realtime_factor_; };

  const bool use_raw_clock;

  double realtime_factor;

  double frame_rate;

  const rclcpp::Time time_on_initialize;

private:
  double seconds_count = 0;

  double nanoseconds_count = 0;

  double seconds_offset = std::numeric_limits<double>::quiet_NaN();

  double nanoseconds_offset = std::numeric_limits<double>::quiet_NaN();
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__SIMULATION_CLOCK__SIMULATION_CLOCK_HPP_
