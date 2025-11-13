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

#include <traffic_simulator/traffic_lights/traffic_lights_base.hpp>

namespace traffic_simulator
{
auto TrafficLightsBase::startUpdate(const double update_rate) -> void
{
  rate_updater_.startTimer(update_rate);
}

auto TrafficLightsBase::resetUpdate(const double update_rate) -> void
{
  rate_updater_.resetTimer(update_rate);
}

// checks, setters, getters
auto TrafficLightsBase::isAnyTrafficLightChanged() const -> bool { return true; }

auto TrafficLightsBase::isRequiredStopTrafficLightState(const lanelet::Id traffic_light_id) -> bool
{
  using Color = traffic_simulator::TrafficLight::Color;
  using Status = traffic_simulator::TrafficLight::Status;
  using Shape = traffic_simulator::TrafficLight::Shape;
  const auto & traffic_light = getTrafficLight(traffic_light_id);
  return (
    traffic_light.contains(Color::red, Status::solid_on, Shape::circle) or
    traffic_light.contains(Color::yellow, Status::solid_on, Shape::circle));
}

auto TrafficLightsBase::compareTrafficLightsState(
  const lanelet::Id lanelet_id, const std::string & state) -> bool
{
  if (const auto & considered_traffic_lights = getTrafficLights(lanelet_id);
      state.empty() || state == "none") {
    return std::all_of(
      std::begin(considered_traffic_lights), std::end(considered_traffic_lights),
      [](const auto & considered_traffic_light) { return considered_traffic_light.get().empty(); });
  } else {
    return std::all_of(
      std::begin(considered_traffic_lights), std::end(considered_traffic_lights),
      [&state](const auto & considered_traffic_light) {
        return considered_traffic_light.get().contains(state);
      });
  }
}

auto TrafficLightsBase::setTrafficLightsColor(
  const lanelet::Id lanelet_id, const traffic_simulator::TrafficLight::Color & color) -> void
{
  for (const auto & traffic_light : getTrafficLights(lanelet_id)) {
    traffic_light.get().emplace(color);
  }
}

auto TrafficLightsBase::setTrafficLightsState(
  const lanelet::Id lanelet_id, const std::string & state) -> void
{
  clearTrafficLightsState(lanelet_id);
  addTrafficLightsState(lanelet_id, state);
}

auto TrafficLightsBase::clearTrafficLightsState(const lanelet::Id lanelet_id) -> void
{
  for (const auto & traffic_light : getTrafficLights(lanelet_id)) {
    traffic_light.get().clear();
  }
}

auto TrafficLightsBase::addTrafficLightsState(
  const lanelet::Id lanelet_id, const std::string & state) -> void
{
  for (const auto & traffic_light : getTrafficLights(lanelet_id)) {
    traffic_light.get().set(state);
  }
}

auto TrafficLightsBase::setTrafficLightsConfidence(
  const lanelet::Id lanelet_id, const double confidence) -> void
{
  for (auto & traffic_light : getTrafficLights(lanelet_id)) {
    traffic_light.get().confidence = confidence;
  }
}

auto TrafficLightsBase::getTrafficLightsComposedState(const lanelet::Id lanelet_id) -> std::string
{
  std::stringstream ss;
  std::string separator = "";
  for (const auto & traffic_light : getTrafficLights(lanelet_id)) {
    ss << separator << traffic_light;
    separator = "; ";
  }
  return ss.str();
}

auto TrafficLightsBase::generateUpdateTrafficLightsRequest() const
  -> simulation_api_schema::UpdateTrafficLightsRequest
{
  simulation_api_schema::UpdateTrafficLightsRequest update_traffic_lights_request;
  for (auto && [lanelet_id, traffic_light] : traffic_lights_map_) {
    auto traffic_signal = static_cast<simulation_api_schema::TrafficSignal>(traffic_light);
    for (const auto & relation_id : traffic_light.regulatory_elements_ids) {
      traffic_signal.add_relation_ids(relation_id);
    }
    *update_traffic_lights_request.add_states() = traffic_signal;
  }
  return update_traffic_lights_request;
}

// private
auto TrafficLightsBase::isTrafficLightAdded(const lanelet::Id traffic_light_id) const -> bool
{
  return traffic_lights_map_.find(traffic_light_id) != traffic_lights_map_.end();
}

auto TrafficLightsBase::addTrafficLight(const lanelet::Id traffic_light_id) -> void
{
  // emplace will not modify the map if the key already exists
  traffic_lights_map_.emplace(
    std::piecewise_construct, std::forward_as_tuple(traffic_light_id),
    std::forward_as_tuple(traffic_light_id, *hdmap_utils_));
}

auto TrafficLightsBase::getTrafficLight(const lanelet::Id traffic_light_id) -> TrafficLight &
{
  if (not isTrafficLightAdded(traffic_light_id)) {
    addTrafficLight(traffic_light_id);
  }
  return traffic_lights_map_.at(traffic_light_id);
}

auto TrafficLightsBase::getTrafficLights(const lanelet::Id lanelet_id)
  -> std::vector<std::reference_wrapper<TrafficLight>>
{
  // if passed id is regulatory element containing traffic_lights, add all of them
  // if passed id is single traffic_light - add it, return all added traffic_lights
  std::vector<std::reference_wrapper<TrafficLight>> traffic_lights;
  if (hdmap_utils_->isTrafficLightRegulatoryElement(lanelet_id)) {
    const auto & regulatory_element = hdmap_utils_->getTrafficLightRegulatoryElement(lanelet_id);
    for (auto && traffic_light : regulatory_element->trafficLights()) {
      traffic_lights.emplace_back(getTrafficLight(traffic_light.id()));
    }
  } else if (hdmap_utils_->isTrafficLight(lanelet_id)) {
    traffic_lights.emplace_back(getTrafficLight(lanelet_id));
  } else {
    throw common::scenario_simulator_exception::Error(
      "Given lanelet ID ", lanelet_id, " is neither a traffic light ID not a traffic relation ID.");
  }
  return traffic_lights;
}
}  // namespace traffic_simulator
