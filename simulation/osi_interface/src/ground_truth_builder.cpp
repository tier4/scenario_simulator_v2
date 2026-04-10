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

#include <osi_interface/ground_truth_builder.hpp>

namespace osi_interface
{
GroundTruthBuilder::GroundTruthBuilder(EntityIdRegistry & registry) : registry_(registry) {}

auto GroundTruthBuilder::setTimestamp(double simulation_time) -> GroundTruthBuilder &
{
  simulation_time_ = simulation_time;
  return *this;
}

auto GroundTruthBuilder::setHostVehicle(const std::string & ego_name) -> GroundTruthBuilder &
{
  ego_name_ = ego_name;
  return *this;
}

auto GroundTruthBuilder::setMapReference(const std::string & map_reference) -> GroundTruthBuilder &
{
  map_reference_ = map_reference;
  return *this;
}

auto GroundTruthBuilder::setMovingEntities(const std::vector<EntityData> & entities)
  -> GroundTruthBuilder &
{
  moving_entities_ = entities;
  return *this;
}

auto GroundTruthBuilder::setStationaryEntities(const std::vector<EntityData> & entities)
  -> GroundTruthBuilder &
{
  stationary_entities_ = entities;
  return *this;
}

auto GroundTruthBuilder::setTrafficSignals(const std::vector<TrafficSignalGroup> & signals)
  -> GroundTruthBuilder &
{
  traffic_signals_ = signals;
  return *this;
}

auto GroundTruthBuilder::build() -> osi3::GroundTruth
{
  osi3::GroundTruth gt;

  // Version
  auto * version = gt.mutable_version();
  version->set_version_major(3);
  version->set_version_minor(8);
  version->set_version_patch(0);

  // Timestamp
  *gt.mutable_timestamp() = toOsiTimestamp(simulation_time_);

  // Host vehicle ID
  if (!ego_name_.empty()) {
    *gt.mutable_host_vehicle_id() = registry_.assign(ego_name_);
  }

  // Map reference
  if (!map_reference_.empty()) {
    gt.set_map_reference(map_reference_);
  }

  // Moving objects (vehicles, pedestrians, ego)
  for (const auto & entity : moving_entities_) {
    *gt.add_moving_object() = toOsiMovingObject(entity, registry_);
  }

  // Stationary objects (misc objects)
  for (const auto & entity : stationary_entities_) {
    *gt.add_stationary_object() = toOsiStationaryObject(entity, registry_);
  }

  // Traffic lights
  for (const auto & signal : traffic_signals_) {
    auto osi_lights = toOsiTrafficLights(signal, registry_);
    for (auto & light : osi_lights) {
      *gt.add_traffic_light() = std::move(light);
    }
  }

  return gt;
}

}  // namespace osi_interface
