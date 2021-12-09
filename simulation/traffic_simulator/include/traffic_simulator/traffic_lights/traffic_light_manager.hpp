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

#ifndef SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#endif

#include <autoware_perception_msgs/msg/traffic_light_state_array.hpp>
#include <iomanip>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>  // std::out_of_range
#include <string>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>
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

  const std::string map_frame_;

  template <typename NodePointer>
  explicit TrafficLightManagerBase(
    const NodePointer & node, const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap,
    const std::string & map_frame = "map")
  : marker_pub_(rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      node, "traffic_light/marker", rclcpp::QoS(1).transient_local())),
    clock_ptr_(node->get_clock()),
    map_frame_(map_frame)
  {
    for (const auto id : hdmap->getTrafficLightIds()) {
      std::unordered_map<TrafficLightColor, geometry_msgs::msg::Point> color_positions;
      if (
        const auto red_position = hdmap->getTrafficLightBulbPosition(id, TrafficLightColor::RED)) {
        color_positions.emplace(TrafficLightColor::RED, red_position.get());
      }
      if (
        const auto yellow_position =
          hdmap->getTrafficLightBulbPosition(id, TrafficLightColor::YELLOW)) {
        color_positions.emplace(TrafficLightColor::YELLOW, yellow_position.get());
      }
      if (
        const auto green_position =
          hdmap->getTrafficLightBulbPosition(id, TrafficLightColor::GREEN)) {
        color_positions.emplace(TrafficLightColor::GREEN, green_position.get());
      }
      traffic_lights_.emplace(
        std::piecewise_construct, std::make_tuple(id), std::make_tuple(id, color_positions));
    }
  }

  auto deleteAllMarkers() const -> void;

  auto drawMarkers() const -> void;

  virtual auto publishTrafficLightStateArray() const -> void = 0;

public:
  auto getIds() const -> std::vector<LaneletID>;

  auto getInstance(const LaneletID) const -> TrafficLight;

  auto hasAnyLightChanged() -> bool;

  auto update(const double) -> void;

#define FORWARD_TO_GIVEN_TRAFFIC_LIGHT(IDENTIFIER)                                         \
  template <typename... Ts>                                                                \
  auto IDENTIFIER(const LaneletID lanelet_id, Ts &&... xs)->decltype(auto)                 \
  {                                                                                        \
    try {                                                                                  \
      return traffic_lights_.at(lanelet_id).IDENTIFIER(std::forward<decltype(xs)>(xs)...); \
    } catch (const std::out_of_range &) {                                                  \
      std::stringstream what;                                                              \
      what << "Given lanelet ID " << std::quoted(std::to_string(lanelet_id))               \
           << " is not a valid traffic-light ID.";                                         \
      THROW_SEMANTIC_ERROR(what.str());                                                    \
    }                                                                                      \
  }                                                                                        \
  static_assert(true, "")

  FORWARD_TO_GIVEN_TRAFFIC_LIGHT(getArrow);
  FORWARD_TO_GIVEN_TRAFFIC_LIGHT(getColor);
  FORWARD_TO_GIVEN_TRAFFIC_LIGHT(setArrow);
  FORWARD_TO_GIVEN_TRAFFIC_LIGHT(setArrowPhase);
  FORWARD_TO_GIVEN_TRAFFIC_LIGHT(setColor);
  FORWARD_TO_GIVEN_TRAFFIC_LIGHT(setColorPhase);

#undef FORWARD_TO_GIVEN_TRAFFIC_LIGHT
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
    traffic_light_state_array_publisher_(rclcpp::create_publisher<Message>(
      node, "/perception/traffic_light_recognition/traffic_light_states",
      rclcpp::QoS(10).transient_local()))
  {
  }

private:
  auto publishTrafficLightStateArray() const -> void override;
};

template <>
auto TrafficLightManager<
  autoware_perception_msgs::msg::TrafficLightStateArray>::publishTrafficLightStateArray() const
  -> void;

#ifndef SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO
template <>
auto TrafficLightManager<
  autoware_auto_perception_msgs::msg::TrafficSignalArray>::publishTrafficLightStateArray() const
  -> void;
#endif
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_HPP_
