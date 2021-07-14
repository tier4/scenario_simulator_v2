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
class TrafficLightManager
{
  const rclcpp::Publisher<autoware_perception_msgs::msg::TrafficLightStateArray>::SharedPtr
    traffic_light_state_array_publisher_;

public:
  explicit TrafficLightManager(
    const std::shared_ptr<hdmap_utils::HdMapUtils> & hdmap_utils_ptr,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
    const rclcpp::Publisher<autoware_perception_msgs::msg::TrafficLightStateArray>::SharedPtr &,
    const std::shared_ptr<rclcpp::Clock> & clock_ptr, const std::string & map_frame = "map");

  void update(const double current_time);

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

  decltype(auto) publishTrafficLightStateArray() const
  {
    autoware_perception_msgs::msg::TrafficLightStateArray traffic_light_state_array;
    {
      traffic_light_state_array.header.frame_id = "camera_link";  // XXX DIRTY HACK!!!
      traffic_light_state_array.header.stamp = (*clock_ptr_).now();

      for (const auto & each : traffic_lights_) {
        traffic_light_state_array.states.push_back(
          static_cast<autoware_perception_msgs::msg::TrafficLightState>(std::get<1>(each)));
      }
    }

    return (*traffic_light_state_array_publisher_).publish(traffic_light_state_array);
  }

  std::unordered_map<std::int64_t, TrafficLight> traffic_lights_;

  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  const std::shared_ptr<rclcpp::Clock> clock_ptr_;

  const std::string map_frame_;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_MANAGER_HPP_
