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

#include <traffic_simulator/traffic_lights/traffic_lights.hpp>

namespace traffic_simulator
{
auto V2ITrafficLights::setTrafficLightsStatePrediction(
  const lanelet::Id lanelet_id, const std::string & state, double time_ahead_seconds) -> void
{
  if (hdmap_utils_->isTrafficLightRegulatoryElement(lanelet_id)) {
    const auto & regulatory_element = hdmap_utils_->getTrafficLightRegulatoryElement(lanelet_id);
    for (auto && traffic_light : regulatory_element->trafficLights()) {
      // call with traffic light way id
      setTrafficLightsStatePrediction(traffic_light.id(), state, time_ahead_seconds);
    }
  } else if (not hdmap_utils_->isTrafficLight(lanelet_id)) {
    throw common::scenario_simulator_exception::Error(
      "Given lanelet ID ", lanelet_id, " is neither a traffic light ID not a traffic relation ID.");
  }
  const auto predicted_time =
    clock_ptr_->now() + rclcpp::Duration(std::chrono::duration<double>(time_ahead_seconds));
  auto & predictions_for_current_traffic_light = predictions_[lanelet_id];
  auto bulb_proto = static_cast<simulation_api_schema::TrafficLight>(TrafficLight::Bulb(state));
  if (auto prediction = std::find_if(
        predictions_for_current_traffic_light.begin(), predictions_for_current_traffic_light.end(),
        [&predicted_time](const auto & pair) {
          constexpr double TIME_EPSILON = 0.001;  // 1ms
          return std::abs((pair.first - predicted_time).seconds()) < TIME_EPSILON;
        });
      prediction != predictions_for_current_traffic_light.end()) {
    // merge into existing prediction
    prediction->second.push_back(bulb_proto);
  } else {
    predictions_for_current_traffic_light.emplace_back(
      predicted_time, std::vector<simulation_api_schema::TrafficLight>{bulb_proto});
  }
}

auto V2ITrafficLights::clearTrafficLightsStatePrediction() -> void { predictions_.clear(); }

auto TrafficLights::isAnyTrafficLightChanged() -> bool
{
  return conventional_traffic_lights_->isAnyTrafficLightChanged() or
         v2i_traffic_lights_->isAnyTrafficLightChanged();
}

auto TrafficLights::startTrafficLightsUpdate(
  const double conventional_traffic_light_update_rate, const double v2i_traffic_lights_update_rate)
  -> void
{
  conventional_traffic_lights_->startUpdate(conventional_traffic_light_update_rate);
  v2i_traffic_lights_->startUpdate(v2i_traffic_lights_update_rate);
}

auto TrafficLights::getConventionalTrafficLights() const
  -> std::shared_ptr<ConventionalTrafficLights>
{
  return conventional_traffic_lights_;
}

auto TrafficLights::getV2ITrafficLights() const -> std::shared_ptr<V2ITrafficLights>
{
  return v2i_traffic_lights_;
}
}  // namespace traffic_simulator
