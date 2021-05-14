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

#ifndef TRAFFIC_SIMULATOR__SIMULATION_CLOCK__SIMULATION_CLOCK_HPP_
#define TRAFFIC_SIMULATOR__SIMULATION_CLOCK__SIMULATION_CLOCK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace traffic_simulator
{
class SimulationClockError : public std::runtime_error
{
public:
  explicit SimulationClockError(const char * message) : runtime_error(message) {}
  explicit SimulationClockError(std::string message) : runtime_error(message.c_str()) {}
  explicit SimulationClockError(std::string message, const char * file, int line)
  : runtime_error(message + "\nFile:" + file + "\nLine:" + std::to_string(line))
  {
  }
};

#define THROW_SIMULATION_CLOCK_ERROR(description) \
  throw SimulationClockError(description, __FILE__, __LINE__);

class SimulationClock : rclcpp::Clock
{
public:
  SimulationClock();
  void initialize(double initial_simulation_time, double step_time);
  void update();
  double getCurrentSimulationTime() const { return current_simulation_time_; }
  double getStepTime() const { return step_time_; }
  rclcpp::Time getCurrentRosTime() const;
  rosgraph_msgs::msg::Clock getCurrentRosTimeAsMsg() const;

private:
  rclcpp::Duration step_time_duration_;
  rclcpp::Time system_time_on_initialize_;
  double current_simulation_time_;
  double initial_simulation_time_;
  double step_time_;
  bool initialized_;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__SIMULATION_CLOCK__SIMULATION_CLOCK_HPP_
