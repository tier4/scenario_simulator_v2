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
#include <visualization_msgs/msg/marker_array.hpp>

namespace traffic_simulator
{
class TrafficLightManagerBase
{
protected:
  using LaneletID = std::int64_t;

  std::unordered_map<LaneletID, TrafficLight> traffic_lights_;

  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  const rclcpp::Clock::SharedPtr clock_ptr_;

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_;

  const std::string map_frame_;

  rclcpp::TimerBase::SharedPtr publish_timer_ = nullptr;

  // node interfaces for creating timer
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;

  const rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_interface_;

  double publish_rate_ = 0.0;

  template <typename NodePointer>
  explicit TrafficLightManagerBase(
    const NodePointer & node, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap,
    const std::string & map_frame = "map")
  : marker_pub_(rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      node, "traffic_light/marker", rclcpp::QoS(1).transient_local())),
    clock_ptr_(node->get_clock()),
    hdmap_(hdmap),
    map_frame_(map_frame),
    node_base_interface_(node->get_node_base_interface()),
    node_timers_interface_(node->get_node_timers_interface())
  {
  }

  auto deleteAllMarkers() const -> void;

  auto drawMarkers() const -> void;

  virtual auto publishTrafficLightStateArray() const -> void = 0;

public:
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

  // Note: lanelet_id can be either a way ID or a relation ID.
  auto getTrafficLights(const LaneletID lanelet_id)
    -> std::vector<std::reference_wrapper<TrafficLight>>
  {
    std::vector<std::reference_wrapper<TrafficLight>> traffic_lights;

    if (hdmap_->isTrafficLightRelation(lanelet_id)) {
      for (auto && traffic_light_element :
           hdmap_->getTrafficLightRelation(lanelet_id)->trafficLights()) {
        traffic_lights.emplace_back(getTrafficLight(traffic_light_element.id()));
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

  auto createPublishTimer(double update_rate) -> void;

  auto resetPublishRate(double update_rate) -> void;

  auto update(const double) -> void;
};
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_BASE_HPP_
