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

#ifndef TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_HPP_
#define TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_HPP_

#include <autoware_auto_perception_msgs/msg/traffic_signal.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <traffic_simulator/color_utils/color_utils.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_state.hpp>
#include <unordered_map>
#include <utility>
#include <vector>

namespace traffic_simulator
{
class TrafficLight
{
public:
  const std::int64_t id;

  explicit TrafficLight(
    const std::int64_t id, const std::shared_ptr<hdmap_utils::HdMapUtils> & map_manager = nullptr)
  : id(id), color_(TrafficLightColor::GREEN), arrow_(TrafficLightArrow::NONE)
  {
    auto locate = [&](auto && color) {
      assert(map_manager);
      if (const auto position = map_manager->getTrafficLightBulbPosition(id, color)) {
        color_positions_.emplace(color, position.get());
      }
    };

    if (map_manager) {
      locate(TrafficLightColor::GREEN);
      locate(TrafficLightColor::RED);
      locate(TrafficLightColor::YELLOW);
    }
  }

  void setArrow(const TrafficLightArrow arrow)
  {
    arrow_ = arrow;
    arrow_changed_ = true;
  }

  void setColor(const TrafficLightColor color)
  {
    color_ = color;
    color_changed_ = true;
  }

  void update(const double) { arrow_changed_ = color_changed_ = false; }

  auto getArrow() const { return arrow_; }
  auto getColor() const { return color_; }

  const geometry_msgs::msg::Point & getPosition(const TrafficLightColor & color) const;
  const geometry_msgs::msg::Point & getPosition(const TrafficLightArrow & arrow) const;

  auto colorChanged() const { return color_changed_; }
  auto arrowChanged() const { return arrow_changed_; }

  explicit operator autoware_auto_perception_msgs::msg::TrafficSignal() const
  {
    autoware_auto_perception_msgs::msg::TrafficSignal traffic_light_state;
    {
      traffic_light_state.map_primitive_id = id;

      try {
        traffic_light_state.lights.push_back(
          convert<autoware_auto_perception_msgs::msg::TrafficLight>(getArrow()));
      } catch (const std::out_of_range &) {
        // NOTE: The traffic light is in Autoware-incompatible state; ignore it.
      }

      try {
        traffic_light_state.lights.push_back(
          convert<autoware_auto_perception_msgs::msg::TrafficLight>(getColor()));
      } catch (const std::out_of_range &) {
        // NOTE: The traffic light is in Autoware-incompatible state; ignore it.
      }
    }

    return traffic_light_state;
  }

private:
  std::unordered_map<TrafficLightColor, geometry_msgs::msg::Point> color_positions_;
  std::unordered_map<TrafficLightArrow, geometry_msgs::msg::Point> arrow_positions_;

  TrafficLightColor color_;
  TrafficLightArrow arrow_;

  bool color_changed_ = false;
  bool arrow_changed_ = false;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__TRAFFIC_LIGHTS__TRAFFIC_LIGHT_HPP_
