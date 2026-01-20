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
auto TrafficLights::isAnyTrafficLightChanged() -> bool
{
  return getConventionalTrafficLights()->isAnyTrafficLightChanged() or
         getV2ITrafficLights()->isAnyTrafficLightChanged() or
         conventional_channel_.hasDetectedChanges() or v2i_channel_.hasDetectedChanges();
}

auto TrafficLights::startTrafficLightsUpdate(
  const double conventional_traffic_light_update_rate, const double v2i_traffic_lights_update_rate)
  -> void
{
  getConventionalTrafficLights()->startUpdate(conventional_traffic_light_update_rate);
  getV2ITrafficLights()->startUpdate(v2i_traffic_lights_update_rate);
}

auto TrafficLights::getConventionalTrafficLights() const
  -> std::shared_ptr<ConventionalTrafficLights>
{
  return conventional_channel_.getGroundTruth();
}

auto TrafficLights::getV2ITrafficLights() const -> std::shared_ptr<V2ITrafficLights>
{
  return v2i_channel_.getGroundTruth();
}

auto TrafficLights::getConventionalDetectedTrafficLights() const
  -> std::shared_ptr<DetectedTrafficLights>
{
  return conventional_channel_.getDetected();
}

auto TrafficLights::getV2IDetectedTrafficLights() const -> std::shared_ptr<DetectedTrafficLights>
{
  return v2i_channel_.getDetected();
}

auto TrafficLights::generateConventionalUpdateRequest() const
  -> simulation_api_schema::UpdateTrafficLightsRequest
{
  return conventional_channel_.generateUpdateRequest();
}

auto TrafficLights::isV2ITrafficLightEnabled(const lanelet::Id lanelet_id) const -> bool
{
  return v2i_enabled_traffic_lights_.count(lanelet_id) > 0;
}

auto V2ITrafficLights::addTrafficLightsStatePrediction(
  const lanelet::Id lanelet_id, const std::string & state, double time_ahead_seconds) -> void
{
  if (hdmap_utils_->isTrafficLightRegulatoryElement(lanelet_id)) {
    // relation id -> way id
    const auto & regulatory_element = hdmap_utils_->getTrafficLightRegulatoryElement(lanelet_id);
    for (const auto & traffic_light : regulatory_element->trafficLights()) {
      addTrafficLightsStatePrediction(traffic_light.id(), state, time_ahead_seconds);
    }
  } else if (not hdmap_utils_->isTrafficLight(lanelet_id)) {
    throw common::scenario_simulator_exception::Error(
      "Given lanelet ID (", lanelet_id, ") is not a traffic light.");
  } else {
    const auto way_id = lanelet_id;
    const auto predicted_time =
      clock_ptr_->now() + rclcpp::Duration(std::chrono::duration<double>(time_ahead_seconds));

    auto & predictions_for_current_traffic_light = predictions_[way_id];

    // state string -> proto
    auto bulb_proto = static_cast<simulation_api_schema::TrafficLight>(TrafficLight::Bulb(state));

    auto existing_prediction = std::find_if(
      predictions_for_current_traffic_light.begin(), predictions_for_current_traffic_light.end(),
      [&predicted_time](const auto & prediction) { return prediction.first == predicted_time; });

    if (existing_prediction != predictions_for_current_traffic_light.end()) {
      // merge if exist
      existing_prediction->second.push_back(bulb_proto);
    } else {
      predictions_for_current_traffic_light.emplace_back(
        predicted_time, std::vector<simulation_api_schema::TrafficLight>{bulb_proto});
    }
  }
}

auto V2ITrafficLights::clearTrafficLightsStatePredictions() -> void { predictions_.clear(); }
}  // namespace traffic_simulator
