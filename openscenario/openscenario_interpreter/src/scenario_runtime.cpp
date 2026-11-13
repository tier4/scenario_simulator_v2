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

#include <memory>
#include <openscenario_interpreter/entity_operations.hpp>
#include <openscenario_interpreter/scenario_runtime.hpp>
#include <openscenario_interpreter/traffic_signal_operations.hpp>

namespace openscenario_interpreter
{
ScenarioRuntime::ScenarioRuntime()
: traffic_signal_operations(std::make_unique<TrafficSignalOperations>()),
  entity_operations(std::make_unique<EntityOperations>())
{
}

ScenarioRuntime::~ScenarioRuntime() = default;

auto ScenarioRuntime::trafficSignals() noexcept -> TrafficSignalOperations &
{
  return *traffic_signal_operations;
}

auto ScenarioRuntime::trafficSignals() const noexcept -> const TrafficSignalOperations &
{
  return *traffic_signal_operations;
}

auto ScenarioRuntime::entities() noexcept -> EntityOperations & { return *entity_operations; }

auto ScenarioRuntime::entities() const noexcept -> const EntityOperations &
{
  return *entity_operations;
}
}  // namespace openscenario_interpreter
