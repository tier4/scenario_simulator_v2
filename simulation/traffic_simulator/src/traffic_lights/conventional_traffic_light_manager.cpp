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

#include <traffic_simulator/traffic_lights/conventional_traffic_light_manager.hpp>

namespace traffic_simulator
{
template <>
auto ConventionalTrafficLightManager<
  autoware_auto_perception_msgs::msg::TrafficSignalArray>::publishTrafficLightStateArray() const
  -> void
{
  autoware_auto_perception_msgs::msg::TrafficSignalArray traffic_light_state_array;
  {
    traffic_light_state_array.header.frame_id = "camera_link";  // DIRTY HACK!!!
    traffic_light_state_array.header.stamp = clock_ptr_->now();
    for (const auto & [id, traffic_light] : getTrafficLights()) {
      traffic_light_state_array.signals.push_back(
        static_cast<autoware_auto_perception_msgs::msg::TrafficSignal>(traffic_light));
    }
  }
//  traffic_light_state_array_publisher_->publish(traffic_light_state_array);
}

template <>
auto ConventionalTrafficLightManager<autoware_auto_perception_msgs::msg::TrafficSignalArray>::name()
  -> const char *
{
  return "/perception/traffic_light_recognition/traffic_signals";
}
}  // namespace traffic_simulator
