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

  template <typename F>
  auto forEachTrafficLights(const LaneletID lanelet_id, F && f) -> void
  {
    if (isTrafficLightId(lanelet_id)) {
      f(traffic_lights_.at(lanelet_id));
    } else if (isTrafficRelationId(lanelet_id)) {
      for (auto && traffic_light : hdmap_->getTrafficLight(lanelet_id)->trafficLights()) {
        f(traffic_lights_.at(traffic_light.id()));
      }
    } else {
      std::stringstream what;
      what << "Given lanelet ID " << std::quoted(std::to_string(lanelet_id))
           << " is neither a traffic light ID not a traffc light relation ID.";
      THROW_SEMANTIC_ERROR(what.str());
    }
  }

  virtual auto publishTrafficLightStateArray() const -> void = 0;

public:
  auto getIds() const -> std::vector<LaneletID>;

  auto getInstance(const LaneletID) const -> TrafficLight;

  auto hasAnyLightChanged() -> bool;

  auto update(const double) -> void;

  auto isTrafficLightId(const LaneletID) -> bool;

  auto isTrafficRelationId(const LaneletID) -> bool;

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

#undef FORWARD_TO_GIVEN_TRAFFIC_LIGHT

#define FORWARD_TO_GIVEN_TRAFFIC_LIGHT(IDENTIFIER)                         \
  template <typename T>                                                    \
  auto IDENTIFIER(const LaneletID lanelet_id, const T & x)->decltype(auto) \
  {                                                                        \
    forEachTrafficLights(lanelet_id, [&](auto && traffic_light) {          \
      return traffic_light.IDENTIFIER(std::forward<decltype(x)>(x));       \
    });                                                                    \
  }                                                                        \
  static_assert(true, "")

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
  autoware_perception_msgs::msg::TrafficLightStateArray>::publishTrafficLightStateArray() const
  -> void;

template <>
auto TrafficLightManager<autoware_perception_msgs::msg::TrafficLightStateArray>::name() -> const
  char *;

#ifndef SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO
template <>
auto TrafficLightManager<
  autoware_auto_perception_msgs::msg::TrafficSignalArray>::publishTrafficLightStateArray() const
  -> void;

template <>
auto TrafficLightManager<autoware_auto_perception_msgs::msg::TrafficSignalArray>::name() -> const
  char *;
#endif
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_HPP_
