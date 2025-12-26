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
}  // namespace traffic_simulator
