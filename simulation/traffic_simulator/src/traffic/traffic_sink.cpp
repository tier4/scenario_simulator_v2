/**
 * @file traffic_sink.cpp
 * @author Masaya Kataoka (masaya.kataoka@tier4.jp)
 * @brief implementation of the TrafficSink class
 * @version 0.1
 * @date 2021-04-01
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

#include <functional>
#include <geometry/distance.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <traffic_simulator/traffic/traffic_sink.hpp>
#include <vector>

namespace traffic_simulator
{
namespace traffic
{
TrafficSink::TrafficSink(
  double radius, const geometry_msgs::msg::Point & position,
  const std::function<std::vector<std::string>(void)> & get_entity_names_function,
  const std::function<geometry_msgs::msg::Pose(const std::string &)> & get_entity_pose_function,
  const std::function<void(std::string)> & despawn_function)
: TrafficModuleBase(),
  radius(radius),
  position(position),
  get_entity_names_function(get_entity_names_function),
  get_entity_pose_function(get_entity_pose_function),
  despawn_function(despawn_function)
{
}

void TrafficSink::execute(
  [[maybe_unused]] const double current_time, [[maybe_unused]] const double step_time)
{
  const auto names = get_entity_names_function();
  for (const auto & name : names) {
    const auto pose = get_entity_pose_function(name);
    if (math::geometry::getDistance(position, pose) <= radius) {
      despawn_function(name);
    }
  }
}
}  // namespace traffic
}  // namespace traffic_simulator
