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
#include <set>
#include <string>
#include <traffic_simulator/entity/entity_manager.hpp>
#include <traffic_simulator/traffic/traffic_module_base.hpp>
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
    const std::function<void(const std::string &)> & despawn_,
    const std::shared_ptr<entity::EntityManager> entity_manager_ptr,
    const std::set<std::uint8_t> auto_sink_entity_types /*= {}*/);

  template <typename T, typename... Ts>
  void addModule(Ts &&... xs)
  {
    auto module_ptr = std::make_shared<T>(std::forward<Ts>(xs)...);
    modules_.emplace_back(module_ptr);
  }
  auto execute(const double current_time, const double step_time) -> void;
  auto makeDebugMarker() const -> visualization_msgs::msg::MarkerArray;

private:
  auto appendAutoSinks(const std::set<std::uint8_t> & auto_sink_entity_types) -> void;
  const std::function<void(const std::string &)> despawn_;
  const std::shared_ptr<entity::EntityManager> entity_manager_ptr_;
  std::vector<std::shared_ptr<TrafficModuleBase> > modules_;
};
}  // namespace traffic
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC__TRAFFIC_CONTROLLER_HPP_
