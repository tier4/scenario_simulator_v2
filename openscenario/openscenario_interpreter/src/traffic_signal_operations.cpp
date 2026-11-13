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

#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/traffic_signal_operations.hpp>
#include <traffic_simulator/api/api.hpp>

namespace openscenario_interpreter
{
auto TrafficSignalOperations::api() -> traffic_simulator::API & { return *SimulatorCore::core; }

auto TrafficSignalOperations::setTrafficSignalV2IFeature(const std::int64_t id, const bool enabled)
  -> void
{
  api().setTrafficSignalV2IFeature(id, enabled);
}

auto TrafficSignalOperations::setConventionalTrafficLightsState(
  const std::int64_t id, const std::string & state) -> void
{
  api().getConventionalTrafficLights()->setTrafficLightsState(id, state);
}

auto TrafficSignalOperations::clearConventionalTrafficLightsState(const std::int64_t id) -> void
{
  api().getConventionalTrafficLights()->clearTrafficLightsState(id);
}

auto TrafficSignalOperations::addConventionalTrafficLightsState(
  const std::int64_t id, const std::string & state) -> void
{
  api().getConventionalTrafficLights()->addTrafficLightsState(id, state);
}

auto TrafficSignalOperations::clearConventionalDetectedTrafficLightsState(const std::int64_t id)
  -> void
{
  api().getConventionalDetectedTrafficLights()->clearState(id);
}

auto TrafficSignalOperations::addConventionalDetectedTrafficLightsState(
  const std::int64_t id, const std::string & state) -> void
{
  api().getConventionalDetectedTrafficLights()->addState(id, state);
}

auto TrafficSignalOperations::setConventionalTrafficLightConfidence(
  const std::int64_t id, const double value) -> void
{
  api().getConventionalTrafficLights()->setTrafficLightsConfidence(id, value);
}

auto TrafficSignalOperations::getConventionalTrafficLightsComposedState(const std::int64_t id) const
  -> std::string
{
  return api().getConventionalTrafficLights()->getTrafficLightsComposedState(id);
}

auto TrafficSignalOperations::compareConventionalTrafficLightsState(
  const std::int64_t id, const std::string & state) const -> bool
{
  return api().getConventionalTrafficLights()->compareTrafficLightsState(id, state);
}

auto TrafficSignalOperations::resetConventionalTrafficLightPublishRate(const double rate) -> void
{
  api().getConventionalTrafficLights()->resetUpdate(rate);
}

auto TrafficSignalOperations::setV2ITrafficLightsState(
  const std::int64_t id, const std::string & state) -> void
{
  api().getV2ITrafficLights()->setTrafficLightsState(id, state);
}

auto TrafficSignalOperations::clearV2ITrafficLightsState(const std::int64_t id) -> void
{
  api().getV2ITrafficLights()->clearTrafficLightsState(id);
}

auto TrafficSignalOperations::addV2ITrafficLightsState(
  const std::int64_t id, const std::string & state) -> void
{
  api().getV2ITrafficLights()->addTrafficLightsState(id, state);
}

auto TrafficSignalOperations::clearV2IDetectedTrafficLightsState(const std::int64_t id) -> void
{
  api().getV2IDetectedTrafficLights()->clearState(id);
}

auto TrafficSignalOperations::addV2IDetectedTrafficLightsState(
  const std::int64_t id, const std::string & state) -> void
{
  api().getV2IDetectedTrafficLights()->addState(id, state);
}

auto TrafficSignalOperations::resetV2ITrafficLightPublishRate(const double rate) -> void
{
  api().getV2ITrafficLights()->resetUpdate(rate);
}

auto TrafficSignalOperations::addV2ITrafficLightsStatePrediction(
  const std::int64_t id, const std::string & state, const double time) -> void
{
  api().getV2ITrafficLights()->addTrafficLightsStatePrediction(id, state, time);
}

auto TrafficSignalOperations::clearV2ITrafficLightsStatePredictions() -> void
{
  api().getV2ITrafficLights()->clearTrafficLightsStatePredictions();
}

auto TrafficSignalOperations::isV2ITrafficLightEnabled(const std::int64_t id) const -> bool
{
  return api().isV2ITrafficLightEnabled(id);
}
}  // namespace openscenario_interpreter
