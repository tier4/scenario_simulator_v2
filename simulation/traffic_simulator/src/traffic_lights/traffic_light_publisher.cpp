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

#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_publisher.hpp>

namespace traffic_simulator
{
template <>
auto TrafficLightPublisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>::publish(
  const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request) -> void
{
  autoware_auto_perception_msgs::msg::TrafficSignalArray msg;
  using TrafficLightType = autoware_auto_perception_msgs::msg::TrafficSignal;
  msg.header.frame_id = "camera_link";  // DIRTY HACK!!!
  msg.header.stamp = current_ros_time;
  for (const auto & traffic_light : request.states()) {
    TrafficLightType traffic_light_msg;
    traffic_light_msg.map_primitive_id = traffic_light.id();
    for (auto bulb_status : traffic_light.traffic_light_status()) {
      using TrafficLightBulbType = TrafficLightType::_lights_type::value_type;
      TrafficLightBulbType light_bulb_msg;
      simulation_interface::toTrafficLightBulbMsg<TrafficLightBulbType>(
        bulb_status, light_bulb_msg);
      traffic_light_msg.lights.push_back(light_bulb_msg);
    }
    msg.signals.push_back(traffic_light_msg);
  }
  traffic_light_state_array_publisher_->publish(msg);
}

template <>
auto TrafficLightPublisher<autoware_perception_msgs::msg::TrafficSignalArray>::publish(
  const rclcpp::Time & current_ros_time,
  const simulation_api_schema::UpdateTrafficLightsRequest & request) -> void
{
  assert(hdmap_utils_ != nullptr);
  // cache store for conversion from way_id to relation_id
  static std::unordered_map<std::int64_t, std::vector<std::int64_t>> id_conversion_cache_map;

  autoware_perception_msgs::msg::TrafficSignalArray msg;
  msg.stamp = current_ros_time;
  for (const auto & traffic_light : request.states()) {
    auto relation_ids = [&]() -> std::vector<std::int64_t> {
      if (auto cache = id_conversion_cache_map.find(traffic_light.id());
          cache != id_conversion_cache_map.end()) {
        // use cached relation_id if the conversion is already cached
        return cache->second;
      } else {
        auto relation_ids = hdmap_utils_->getTrafficLightRelationIDsFromWayID(traffic_light.id());
        // cache the conversion
        id_conversion_cache_map[traffic_light.id()] = relation_ids;
        return relation_ids;
      }
    }();

    for (auto relation_id : relation_ids) {
      // skip if the traffic light has no bulbs
      if (not traffic_light.traffic_light_status().empty()) {
        using TrafficLightType = autoware_perception_msgs::msg::TrafficSignal;
        TrafficLightType traffic_light_msg;
        traffic_light_msg.traffic_signal_id = relation_id;

        for (auto bulb_status : traffic_light.traffic_light_status()) {
          using TrafficLightBulbType =
            autoware_perception_msgs::msg::TrafficSignal::_elements_type::value_type;
          TrafficLightBulbType light_bulb_msg;
          simulation_interface::toTrafficLightBulbMsg<TrafficLightBulbType>(
            bulb_status, light_bulb_msg);
          traffic_light_msg.elements.push_back(light_bulb_msg);
        }
        msg.signals.push_back(traffic_light_msg);
      }
    }
  }
  traffic_light_state_array_publisher_->publish(msg);
}
}  // namespace traffic_simulator
