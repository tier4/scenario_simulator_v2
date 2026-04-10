// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef OSI_INTERFACE__GROUND_TRUTH_BUILDER_HPP_
#define OSI_INTERFACE__GROUND_TRUTH_BUILDER_HPP_

#include <osi3/osi_groundtruth.pb.h>

#include <osi_interface/entity_id_registry.hpp>
#include <osi_interface/osi_entity_conversions.hpp>
#include <osi_interface/osi_traffic_light_conversions.hpp>
#include <string>
#include <vector>

namespace osi_interface
{
class GroundTruthBuilder
{
public:
  explicit GroundTruthBuilder(EntityIdRegistry & registry);

  // Set simulation timestamp
  auto setTimestamp(double simulation_time) -> GroundTruthBuilder &;

  // Set the host (ego) vehicle by entity name
  auto setHostVehicle(const std::string & ego_name) -> GroundTruthBuilder &;

  // Set map reference (e.g. lanelet2 map path)
  auto setMapReference(const std::string & map_reference) -> GroundTruthBuilder &;

  // Add entities (vehicles, pedestrians, ego)
  auto setMovingEntities(const std::vector<EntityData> & entities) -> GroundTruthBuilder &;

  // Add stationary entities (misc objects)
  auto setStationaryEntities(const std::vector<EntityData> & entities) -> GroundTruthBuilder &;

  // Add traffic light signal groups
  auto setTrafficSignals(const std::vector<TrafficSignalGroup> & signals) -> GroundTruthBuilder &;

  // Build the GroundTruth message
  auto build() -> osi3::GroundTruth;

private:
  EntityIdRegistry & registry_;
  double simulation_time_{0.0};
  std::string ego_name_;
  std::string map_reference_;
  std::vector<EntityData> moving_entities_;
  std::vector<EntityData> stationary_entities_;
  std::vector<TrafficSignalGroup> traffic_signals_;
};

}  // namespace osi_interface

#endif  // OSI_INTERFACE__GROUND_TRUTH_BUILDER_HPP_
