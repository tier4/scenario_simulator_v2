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

#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
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
class TrafficLightManager
{
  const rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>::SharedPtr
    traffic_light_state_array_publisher_;

public:
  template <typename Node>
  explicit TrafficLightManager(
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr, const Node & node,
    const std::string & map_frame = "map")
  : traffic_light_state_array_publisher_(
      rclcpp::create_publisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>(
        node, "/perception/traffic_light_recognition/traffic_light_states",
        rclcpp::QoS(10).transient_local())),
    traffic_lights_(),
    marker_pub_(rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      node, "traffic_light/marker", rclcpp::QoS(1).transient_local())),
    clock_ptr_(node->get_clock()),
    map_frame_(map_frame)
  {
    for (const auto id : hdmap_utils_ptr->getTrafficLightIds()) {
      std::unordered_map<TrafficLightColor, geometry_msgs::msg::Point> color_positions;

      const auto red_position =
        hdmap_utils_ptr->getTrafficLightBulbPosition(id, TrafficLightColor::RED);
      if (red_position) {
        color_positions.emplace(TrafficLightColor::RED, red_position.get());
      }

      const auto yellow_position =
        hdmap_utils_ptr->getTrafficLightBulbPosition(id, TrafficLightColor::YELLOW);
      if (yellow_position) {
        color_positions.emplace(TrafficLightColor::YELLOW, yellow_position.get());
      }

      const auto green_position =
        hdmap_utils_ptr->getTrafficLightBulbPosition(id, TrafficLightColor::GREEN);
      if (green_position) {
        color_positions.emplace(TrafficLightColor::GREEN, green_position.get());
      }

      traffic_lights_.emplace(
        std::piecewise_construct, std::make_tuple(id), std::make_tuple(id, color_positions));
    }
  }

  bool hasAnyLightChanged();
  TrafficLight getInstance(const std::int64_t lanelet_id);
  void update(const double step_time);

#define FORWARD_TO_GIVEN_TRAFFIC_LIGHT(IDENTIFIER)                                         \
  template <typename... Ts>                                                                \
  decltype(auto) IDENTIFIER(const std::int64_t lanelet_id, Ts &&... xs)                    \
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

  std::vector<std::int64_t> getIds() const;

private:
  void deleteAllMarkers() const;

  void drawMarkers() const;

  void publishTrafficLightStateArray() const
  {
    autoware_auto_perception_msgs::msg::TrafficSignalArray traffic_light_state_array;
    {
      traffic_light_state_array.header.frame_id = "camera_link";  // XXX DIRTY HACK!!!
      traffic_light_state_array.header.stamp = (*clock_ptr_).now();
      for (const auto & each : traffic_lights_) {
        if (each.second.getColor() != TrafficLightColor::NONE) {
          traffic_light_state_array.signals.push_back(
            static_cast<autoware_auto_perception_msgs::msg::TrafficSignal>(std::get<1>(each)));
        }
      }
    }
    traffic_light_state_array_publisher_->publish(traffic_light_state_array);
  }

  std::unordered_map<std::int64_t, TrafficLight> traffic_lights_;

  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  const std::shared_ptr<rclcpp::Clock> clock_ptr_;

  const std::string map_frame_;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_HPP_
