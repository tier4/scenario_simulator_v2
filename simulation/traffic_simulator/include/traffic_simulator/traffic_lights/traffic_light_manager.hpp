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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_HPP_

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
class TrafficLightManagerBase
{
protected:
  using LaneletID = std::int64_t;

  std::unordered_map<LaneletID, TrafficLight> traffic_lights_;

  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  const rclcpp::Clock::SharedPtr clock_ptr_;

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_;

  const std::string map_frame_;

  template <typename NodePointer>
  explicit TrafficLightManagerBase(
    const NodePointer & node, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap,
    const std::string & map_frame = "map")
  : marker_pub_(rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      node, "traffic_light/marker", rclcpp::QoS(1).transient_local())),
    clock_ptr_(node->get_clock()),
    hdmap_(hdmap),
    map_frame_(map_frame)
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

  auto getTrafficRelationReferees(const LaneletID lanelet_id)
    -> std::vector<std::reference_wrapper<TrafficLight>>
  {
    std::vector<std::reference_wrapper<TrafficLight>> refers;

    if (hdmap_->isTrafficRelation(lanelet_id)) {
      for (auto && traffic_light : hdmap_->getTrafficRelation(lanelet_id)->trafficLights()) {
        refers.emplace_back(getTrafficLight(traffic_light.id()));
      }
    } else if (hdmap_->isTrafficLight(lanelet_id)) {
      refers.emplace_back(getTrafficLight(lanelet_id));
    } else {
      throw common::scenario_simulator_exception::Error(
        "Given lanelet ID ", lanelet_id,
        " is neither a traffic light ID not a traffic relation ID.");
    }

    return refers;
  }

  auto hasAnyLightChanged() -> bool;

  auto update(const double) -> void;
};

template <typename Message>
class TrafficLightManager : public TrafficLightManagerBase
{
  const typename rclcpp::Publisher<Message>::SharedPtr traffic_light_state_array_publisher_;

public:
  template <typename Node>
  explicit TrafficLightManager(
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap, const Node & node,
    const std::string & map_frame = "map")
  : TrafficLightManagerBase(node, hdmap, map_frame),
    traffic_light_state_array_publisher_(
      rclcpp::create_publisher<Message>(node, name(), rclcpp::QoS(10).transient_local()))
  {
  }

private:
  static auto name() -> const char *;

  auto publishTrafficLightStateArray() const -> void override;
};

template <>
auto TrafficLightManager<
  autoware_auto_perception_msgs::msg::TrafficSignalArray>::publishTrafficLightStateArray() const
  -> void;

template <>
auto TrafficLightManager<autoware_auto_perception_msgs::msg::TrafficSignalArray>::name() -> const
  char *;
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_HPP_
