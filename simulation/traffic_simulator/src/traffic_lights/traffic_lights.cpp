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
