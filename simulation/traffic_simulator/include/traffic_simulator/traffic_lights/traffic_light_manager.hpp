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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_BASE_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_BASE_HPP_

#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>  // std::out_of_range
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace traffic_simulator
{
class TrafficLightManager
{
protected:
  using LaneletID = std::int64_t;
  using TrafficLightMap = std::unordered_map<LaneletID, TrafficLight>;

  TrafficLightMap traffic_lights_;
  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_;

public:
  explicit TrafficLightManager(const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap);

  auto getTrafficLight(const LaneletID traffic_light_id) -> TrafficLight &;

  auto getTrafficLights() const -> const TrafficLightMap &;

  auto getTrafficLights() -> TrafficLightMap &;

  auto getTrafficLights(const LaneletID lanelet_id)
    -> std::vector<std::reference_wrapper<TrafficLight>>;

  auto hasAnyLightChanged() -> bool;
};
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_BASE_HPP_
