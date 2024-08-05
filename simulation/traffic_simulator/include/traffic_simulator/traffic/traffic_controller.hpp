/**
 * @file traffic_controller.hpp
 * @author Masaya Kataoka (masaya.kataoka@tier4.jp)
 * @brief class definition for the traffic controller
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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_CONTROLLER_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic/traffic_module_base.hpp>
#include <traffic_simulator/traffic/traffic_source.hpp>
#include <utility>
#include <vector>

namespace traffic_simulator
{
namespace traffic
{
class TrafficController
{
public:
  explicit TrafficController(
    std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils,
    const std::function<std::vector<std::string>(void)> & get_entity_names_function,
    const std::function<geometry_msgs::msg::Pose(const std::string &)> & get_entity_pose_function,
    const std::function<void(std::string)> & despawn_function, bool auto_sink = false);

  template <typename T, typename... Ts>
  void addModule(Ts &&... xs)
  {
    auto module_ptr = std::make_shared<T>(std::forward<Ts>(xs)...);
    modules_.emplace_back(module_ptr);
  }
  void execute(const double current_time, const double step_time);
  auto makeDebugMarker() const -> const visualization_msgs::msg::MarkerArray;

private:
  void autoSink();
  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_;
  std::vector<std::shared_ptr<traffic_simulator::traffic::TrafficModuleBase>> modules_;
  const std::function<std::vector<std::string>(void)> get_entity_names_function;
  const std::function<geometry_msgs::msg::Pose(const std::string &)> get_entity_pose_function;
  const std::function<void(const std::string &)> despawn_function;

public:
  const bool auto_sink;
};
}  // namespace traffic
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_CONTROLLER_HPP_
