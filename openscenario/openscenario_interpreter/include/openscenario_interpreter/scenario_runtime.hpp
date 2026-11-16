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

#ifndef OPENSCENARIO_INTERPRETER__SCENARIO_RUNTIME_HPP_
#define OPENSCENARIO_INTERPRETER__SCENARIO_RUNTIME_HPP_

#include <memory>
#include <openscenario_interpreter/traffic_signal_operations.hpp>

namespace openscenario_interpreter
{
class EntityOperations;

class ScenarioRuntime
{
  const std::unique_ptr<TrafficSignalOperations> traffic_signal_operations;

  const std::unique_ptr<EntityOperations> entity_operations;

public:
  ScenarioRuntime();

  ~ScenarioRuntime();

  auto trafficSignals() noexcept -> TrafficSignalOperations &;

  auto trafficSignals() const noexcept -> const TrafficSignalOperations &;

  auto entities() noexcept -> EntityOperations &;

  auto entities() const noexcept -> const EntityOperations &;
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SCENARIO_RUNTIME_HPP_
