/**
 * @file traffic_controller.cpp
 * @author Masaya Kataoka (masaya.kataoka@tier4.jp)
 * @brief class implementation for the traffic controller
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

#include <memory>
#include <string>
#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator/traffic/traffic_controller.hpp>
#include <traffic_simulator/traffic/traffic_sink.hpp>
#include <traffic_simulator/utils/lanelet.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <utility>
#include <vector>

namespace traffic_simulator
{
namespace traffic
{
TrafficController::TrafficController(
  const std::function<std::vector<std::string>(void)> & get_entity_names_function,
  const std::function<geometry_msgs::msg::Pose(const std::string &)> & get_entity_pose_function,
  const std::function<void(std::string)> & despawn_function, bool auto_sink)
: get_entity_names_function(get_entity_names_function),
  get_entity_pose_function(get_entity_pose_function),
  despawn_function(despawn_function),
  auto_sink(auto_sink)
{
  if (auto_sink) {
    autoSink();
  }
}

void TrafficController::autoSink()
{
  for (const auto & lanelet_id : lanelet2::getLaneletIds()) {
    if (lanelet2::getNextLaneletIds(lanelet_id).empty()) {
      LaneletPose lanelet_pose;
      lanelet_pose.lanelet_id = lanelet_id;
      lanelet_pose.s = pose::laneletLength(lanelet_id);
      const auto pose = pose::toMapPose(lanelet_pose);
      addModule<traffic_simulator::traffic::TrafficSink>(
        1, pose.position, get_entity_names_function, get_entity_pose_function, despawn_function);
    }
  }
}

void TrafficController::execute()
{
  for (const auto & module : modules_) {
    module->execute();
  }
}
}  // namespace traffic
}  // namespace traffic_simulator
