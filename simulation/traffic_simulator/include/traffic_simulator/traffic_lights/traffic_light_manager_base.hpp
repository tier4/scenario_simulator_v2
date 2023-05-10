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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MODULE_BASE_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MODULE_BASE_HPP_

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

enum class TrafficLightType {
  conventional,
  v2i,
};

class TrafficLightManagerBase
{
protected:
  using LaneletID = std::int64_t;

  std::unordered_map<LaneletID, TrafficLight> traffic_lights_;

  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  const rclcpp::Clock::SharedPtr clock_ptr_;

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_;

  const std::string map_frame_;

  rclcpp::TimerBase::SharedPtr timer_ = nullptr;

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;

  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_interface_;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  double update_rate_ = 0.0;

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

  auto updatePublishRate(double update_rate) -> void
  {
    if (update_rate_ != update_rate) {
      update_rate_ = update_rate;
      if (timer_) {
        timer_->reset();
      }

      timer_ = rclcpp::create_timer(
        node_base_interface_, node_timers_interface_, clock_ptr_,
        rclcpp::Duration::from_seconds(1.0 / update_rate_),
        [this]() -> void { update(1.0 / update_rate_); });
    }
  }

  auto update(const double) -> void;
};
}  // namespace traffic_simulator
#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MODULE_BASE_HPP_
