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

#ifndef OPENSCENARIO_INTERPRETER__TRAFFIC_SIGNAL_OPERATIONS_HPP_
#define OPENSCENARIO_INTERPRETER__TRAFFIC_SIGNAL_OPERATIONS_HPP_

#include <cstdint>
#include <string>

namespace traffic_simulator
{
class API;
}  // namespace traffic_simulator

namespace openscenario_interpreter
{
class TrafficSignalOperations
{
  static auto api() -> traffic_simulator::API &;

public:
  auto setTrafficSignalV2IFeature(std::int64_t, bool) -> void;
  auto setConventionalTrafficLightsState(std::int64_t, const std::string &) -> void;
  auto clearConventionalTrafficLightsState(std::int64_t) -> void;
  auto addConventionalTrafficLightsState(std::int64_t, const std::string &) -> void;
  auto clearConventionalDetectedTrafficLightsState(std::int64_t) -> void;
  auto addConventionalDetectedTrafficLightsState(std::int64_t, const std::string &) -> void;
  auto setConventionalTrafficLightConfidence(std::int64_t, double) -> void;
  auto getConventionalTrafficLightsComposedState(std::int64_t) const -> std::string;
  auto compareConventionalTrafficLightsState(std::int64_t, const std::string &) const -> bool;
  auto resetConventionalTrafficLightPublishRate(double) -> void;
  auto setV2ITrafficLightsState(std::int64_t, const std::string &) -> void;
  auto clearV2ITrafficLightsState(std::int64_t) -> void;
  auto addV2ITrafficLightsState(std::int64_t, const std::string &) -> void;
  auto clearV2IDetectedTrafficLightsState(std::int64_t) -> void;
  auto addV2IDetectedTrafficLightsState(std::int64_t, const std::string &) -> void;
  auto resetV2ITrafficLightPublishRate(double) -> void;
  auto addV2ITrafficLightsStatePrediction(std::int64_t, const std::string &, double) -> void;
  auto clearV2ITrafficLightsStatePredictions() -> void;
  auto isV2ITrafficLightEnabled(std::int64_t) const -> bool;
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__TRAFFIC_SIGNAL_OPERATIONS_HPP_
