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

#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
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
#include <visualization_msgs/msg/marker_array.hpp>

namespace traffic_simulator
{
class TrafficLightManager
{
protected:
  using LaneletID = std::int64_t;

  std::unordered_map<LaneletID, TrafficLight> traffic_lights_;

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_;
public:
  explicit TrafficLightManager(const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap)
      : hdmap_(hdmap)
  {
  }

  auto getTrafficLight(const LaneletID traffic_light_id) -> auto &
  {
    if (auto iter = traffic_lights_.find(traffic_light_id); iter != std::end(traffic_lights_)) {
      return iter->second;
    } else {
      traffic_lights_.emplace(
        std::piecewise_construct, std::forward_as_tuple(traffic_light_id),
        std::forward_as_tuple(traffic_light_id, *hdmap_));
      return traffic_lights_.at(traffic_light_id);
    }
  }

  auto getTrafficLights() const -> const auto & { return traffic_lights_; }

  auto getTrafficLights() -> auto & { return traffic_lights_; }

  auto getTrafficLights(const LaneletID lanelet_id)
    -> std::vector<std::reference_wrapper<TrafficLight>>
  {
    std::vector<std::reference_wrapper<TrafficLight>> traffic_lights;

    if (hdmap_->isTrafficRelation(lanelet_id)) {
      for (auto && traffic_light : hdmap_->getTrafficRelation(lanelet_id)->trafficLights()) {
        traffic_lights.emplace_back(getTrafficLight(traffic_light.id()));
      }
    } else if (hdmap_->isTrafficLight(lanelet_id)) {
      traffic_lights.emplace_back(getTrafficLight(lanelet_id));
    } else {
      throw common::scenario_simulator_exception::Error(
        "Given lanelet ID ", lanelet_id,
        " is neither a traffic light ID not a traffic relation ID.");
    }

    return traffic_lights;
  }

  auto hasAnyLightChanged() -> bool;
};
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_BASE_HPP_
