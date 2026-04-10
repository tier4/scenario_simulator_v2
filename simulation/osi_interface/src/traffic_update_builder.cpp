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

#include <osi_interface/traffic_update_builder.hpp>

namespace osi_interface
{
TrafficUpdateBuilder::TrafficUpdateBuilder(EntityIdRegistry & registry) : registry_(registry) {}

auto TrafficUpdateBuilder::setTimestamp(double simulation_time) -> TrafficUpdateBuilder &
{
  simulation_time_ = simulation_time;
  return *this;
}

auto TrafficUpdateBuilder::setEgoUpdate(const EntityData & ego_entity) -> TrafficUpdateBuilder &
{
  ego_entity_ = ego_entity;
  has_ego_ = true;
  return *this;
}

auto TrafficUpdateBuilder::setHostVehicleData(const osi3::HostVehicleData & data)
  -> TrafficUpdateBuilder &
{
  host_vehicle_data_ = data;
  has_host_vehicle_data_ = true;
  return *this;
}

auto TrafficUpdateBuilder::build() -> osi3::TrafficUpdate
{
  osi3::TrafficUpdate tu;

  // Version
  auto * version = tu.mutable_version();
  version->set_version_major(3);
  version->set_version_minor(8);
  version->set_version_patch(0);

  // Timestamp (coincides with the GroundTruth timestamp - no latency for TPM updates)
  *tu.mutable_timestamp() = toOsiTimestamp(simulation_time_);

  // Ego vehicle update (dynamic fields only per OSI spec)
  if (has_ego_) {
    *tu.add_update() = toOsiMovingObject(ego_entity_, registry_);
  }

  // Host vehicle internal state
  if (has_host_vehicle_data_) {
    *tu.add_internal_state() = host_vehicle_data_;
  }

  return tu;
}

}  // namespace osi_interface
