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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_HPP_

#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <unordered_map>
#include <vector>
#include <utility>
#include <string>

namespace traffic_simulator
{
class TrafficLightManager
{
public:
  explicit TrafficLightManager(
    std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher,
    const std::shared_ptr<rclcpp::Clock> & clock_ptr,
    const std::string & map_frame = "map");
  void update(double step_time);
  template<typename ... Ts>
  void setColorPhase(std::int64_t lanelet_id, Ts && ... xs)
  {
    if (traffic_lights_.count(lanelet_id) == 0) {
      throw SimulationRuntimeError("lanelet id does not match");
    }
    traffic_lights_.at(lanelet_id)->setColorPhase(std::forward<Ts>(xs)...);
  }
  template<typename ... Ts>
  void setArrowPhase(std::int64_t lanelet_id, Ts && ... xs)
  {
    if (traffic_lights_.count(lanelet_id) == 0) {
      throw SimulationRuntimeError("lanelet id does not match");
    }
    traffic_lights_.at(lanelet_id)->setArrowPhase(std::forward<Ts>(xs)...);
  }
  template<typename ... Ts>
  void setColor(std::int64_t lanelet_id, Ts && ... xs)
  {
    if (traffic_lights_.count(lanelet_id) == 0) {
      throw SimulationRuntimeError("lanelet id does not match");
    }
    traffic_lights_.at(lanelet_id)->setColor(std::forward<Ts>(xs)...);
  }
  template<typename ... Ts>
  void setArrow(std::int64_t lanelet_id, Ts && ... xs)
  {
    if (traffic_lights_.count(lanelet_id) == 0) {
      throw SimulationRuntimeError("lanelet id does not match");
    }
    traffic_lights_.at(lanelet_id)->setArrow(std::forward<Ts>(xs)...);
  }
  TrafficLightColor getColor(std::int64_t lanelet_id) const;
  TrafficLightArrow getArrow(std::int64_t lanelet_id) const;
  std::vector<std::int64_t> getIds() const;

private:
  void deleteAllMarkers() const;
  void drawMarkers() const;
  std::unordered_map<std::int64_t, std::shared_ptr<TrafficLight>> traffic_lights_;
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  const std::shared_ptr<rclcpp::Clock> clock_ptr_;
  const std::string map_frame_;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_HPP_
