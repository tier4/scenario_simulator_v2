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

#include <algorithm>
#include <osi_interface/osi_ground_truth_handler.hpp>

namespace osi_interface
{
OsiGroundTruthHandler::OsiGroundTruthHandler() = default;

auto OsiGroundTruthHandler::parseGroundTruth(const osi3::GroundTruth & gt) -> GroundTruthFrame
{
  GroundTruthFrame frame;

  // Pre-populate registry from source_reference so that fromOsiMovingObject can resolve names.
  // The client embeds the entity name in source_reference[0].identifier[0] via toOsiMovingObject,
  // but the server-side registry starts empty and must be seeded from the incoming message.
  auto registerFromSourceRef = [this](uint64_t id, const auto & source_refs) {
    if (registry_.reverseLookup(id).has_value()) return;
    for (const auto & ref : source_refs) {
      for (const auto & identifier : ref.identifier()) {
        if (!identifier.empty()) {
          registry_.insertMapping(identifier, id);
          return;
        }
      }
    }
  };
  for (int i = 0; i < gt.moving_object_size(); ++i) {
    const auto & obj = gt.moving_object(i);
    if (obj.has_id()) registerFromSourceRef(obj.id().value(), obj.source_reference());
  }
  for (int i = 0; i < gt.stationary_object_size(); ++i) {
    const auto & obj = gt.stationary_object(i);
    if (obj.has_id()) registerFromSourceRef(obj.id().value(), obj.source_reference());
  }

  // Timestamp
  if (gt.has_timestamp()) {
    frame.simulation_time = fromOsiTimestamp(gt.timestamp());
  }

  // Map reference
  if (gt.has_map_reference()) {
    frame.map_reference = gt.map_reference();
  }

  // Ego name (from host_vehicle_id)
  uint64_t host_id = 0;
  if (gt.has_host_vehicle_id()) {
    host_id = gt.host_vehicle_id().value();
    auto name = registry_.reverseLookup(host_id);
    if (name.has_value()) {
      frame.ego_name = *name;
    }
  }

  // Parse moving objects
  std::set<uint64_t> current_moving_ids;
  for (int i = 0; i < gt.moving_object_size(); ++i) {
    const auto & obj = gt.moving_object(i);
    auto entity = fromOsiMovingObject(obj, registry_);

    // Identify ego by host_vehicle_id
    if (obj.has_id() && obj.id().value() == host_id) {
      entity.type = EntityType::EGO;
      frame.ego_name = entity.name;
    }

    current_moving_ids.insert(obj.id().value());
    frame.moving_entities.push_back(entity);
  }

  // Parse stationary objects
  std::set<uint64_t> current_stationary_ids;
  for (int i = 0; i < gt.stationary_object_size(); ++i) {
    const auto & obj = gt.stationary_object(i);
    auto entity = fromOsiStationaryObject(obj, registry_);
    current_stationary_ids.insert(obj.id().value());
    frame.stationary_entities.push_back(entity);
  }

  // Parse traffic lights (group by source_reference lanelet ID)
  // For now, each osi3::TrafficLight is treated independently
  // TODO: Group by lanelet ID for proper TrafficSignalGroup reconstruction
  for (int i = 0; i < gt.traffic_light_size(); ++i) {
    const auto & tl = gt.traffic_light(i);
    auto group = fromOsiTrafficLights({tl}, registry_);
    if (!group.bulbs.empty()) {
      frame.traffic_signals.push_back(group);
    }
  }

  // Detect spawned moving entities (new IDs not in previous frame)
  for (const auto & entity : frame.moving_entities) {
    auto id = registry_.lookup(entity.name);
    if (id.has_value() && prev_moving_ids_.find(id->value()) == prev_moving_ids_.end()) {
      frame.spawned_moving.push_back(entity);
    }
  }

  // Detect spawned stationary entities
  for (const auto & entity : frame.stationary_entities) {
    auto id = registry_.lookup(entity.name);
    if (id.has_value() && prev_stationary_ids_.find(id->value()) == prev_stationary_ids_.end()) {
      frame.spawned_stationary.push_back(entity);
    }
  }

  // Detect despawned entities (IDs in previous frame but not in current)
  for (const auto & prev_id : prev_moving_ids_) {
    if (current_moving_ids.find(prev_id) == current_moving_ids.end()) {
      auto name = registry_.reverseLookup(prev_id);
      if (name.has_value()) {
        frame.despawned_names.push_back(*name);
      }
    }
  }
  for (const auto & prev_id : prev_stationary_ids_) {
    if (current_stationary_ids.find(prev_id) == current_stationary_ids.end()) {
      auto name = registry_.reverseLookup(prev_id);
      if (name.has_value()) {
        frame.despawned_names.push_back(*name);
      }
    }
  }

  // Update previous frame state
  prev_moving_ids_ = current_moving_ids;
  prev_stationary_ids_ = current_stationary_ids;

  return frame;
}

auto OsiGroundTruthHandler::buildTrafficUpdate(
  double simulation_time, const EntityData & ego_entity) -> osi3::TrafficUpdate
{
  return TrafficUpdateBuilder(registry_)
    .setTimestamp(simulation_time)
    .setEgoUpdate(ego_entity)
    .build();
}

}  // namespace osi_interface
