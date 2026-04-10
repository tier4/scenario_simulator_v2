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

#ifndef OSI_INTERFACE__OSI_GROUND_TRUTH_HANDLER_HPP_
#define OSI_INTERFACE__OSI_GROUND_TRUTH_HANDLER_HPP_

#include <osi3/osi_groundtruth.pb.h>
#include <osi3/osi_trafficupdate.pb.h>

#include <osi_interface/entity_id_registry.hpp>
#include <osi_interface/ground_truth_builder.hpp>
#include <osi_interface/osi_entity_conversions.hpp>
#include <osi_interface/osi_traffic_light_conversions.hpp>
#include <osi_interface/traffic_update_builder.hpp>
#include <set>
#include <string>
#include <vector>

namespace osi_interface
{
// Result of parsing a GroundTruth message.
// Contains the extracted entity data and lifecycle events (spawn/despawn).
struct GroundTruthFrame
{
  double simulation_time{0.0};
  std::string ego_name;
  std::string map_reference;

  // All entities extracted from this frame
  std::vector<EntityData> moving_entities;
  std::vector<EntityData> stationary_entities;
  std::vector<TrafficSignalGroup> traffic_signals;

  // Lifecycle events detected by diffing with previous frame
  std::vector<EntityData> spawned_moving;
  std::vector<EntityData> spawned_stationary;
  std::vector<std::string> despawned_names;
};

// Parses GroundTruth messages into EntityData and detects spawn/despawn events.
// Builds TrafficUpdate messages from ego entity data.
// Does NOT perform ego dynamics or sensor simulation — those are the consumer's
// responsibility (e.g., simple_sensor_simulator).
class OsiGroundTruthHandler
{
public:
  OsiGroundTruthHandler();

  // Parse a GroundTruth message into a GroundTruthFrame.
  // Detects newly appeared and disappeared entities by comparing with previous frame.
  auto parseGroundTruth(const osi3::GroundTruth & gt) -> GroundTruthFrame;

  // Build a TrafficUpdate from an updated ego entity.
  auto buildTrafficUpdate(double simulation_time, const EntityData & ego_entity)
    -> osi3::TrafficUpdate;

  auto getRegistry() -> EntityIdRegistry & { return registry_; }
  auto getRegistry() const -> const EntityIdRegistry & { return registry_; }

private:
  EntityIdRegistry registry_;
  std::set<uint64_t> prev_moving_ids_;
  std::set<uint64_t> prev_stationary_ids_;
};

}  // namespace osi_interface

#endif  // OSI_INTERFACE__OSI_GROUND_TRUTH_HANDLER_HPP_
