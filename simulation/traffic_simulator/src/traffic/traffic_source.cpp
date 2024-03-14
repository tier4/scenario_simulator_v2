/**
 * @file traffic_source.cpp
 * @author Mateusz Palczuk (mateusz.palczuk@robotec.ai)
 * @brief implementation of the TrafficSource class
 * @version 0.1
 * @date 2024-03-14
 *
 * @copyright Copyright(c) TIER IV.Inc {2015}
 *
 */

// Copyright 2015 TIER IV.inc. All rights reserved.
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

#include <traffic_simulator/traffic/traffic_source.hpp>

namespace traffic_simulator
{
namespace traffic
{
TrafficSource::TrafficSource(
  const double radius, const double rate, const double speed, const geometry_msgs::msg::Pose & pose,
  const std::function<void(const geometry_msgs::msg::Pose &, const double)> & spawn_function)
: radius(radius), rate(rate), speed(speed), pose(pose), spawn_function(spawn_function)
{
}
void TrafficSource::execute(
  [[maybe_unused]] const double current_time, [[maybe_unused]] const double step_time)
{
  if (current_time - last_spawn_time < 1.0 / rate) {
    return;
  }
  spawn_function(pose, speed);
  last_spawn_time = current_time;
}
}  // namespace traffic
}  // namespace traffic_simulator
