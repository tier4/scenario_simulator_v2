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

#ifndef OSI_INTERFACE__TRAFFIC_UPDATE_BUILDER_HPP_
#define OSI_INTERFACE__TRAFFIC_UPDATE_BUILDER_HPP_

#include <osi3/osi_hostvehicledata.pb.h>
#include <osi3/osi_trafficupdate.pb.h>

#include <osi_interface/entity_id_registry.hpp>
#include <osi_interface/osi_entity_conversions.hpp>

namespace osi_interface
{
// Builds a pure osi3::TrafficUpdate message.
// In the TPM (TrafficParticipantModel) pattern, this represents the output
// of the ego vehicle's dynamics model after processing a GroundTruth input.
class TrafficUpdateBuilder
{
public:
  explicit TrafficUpdateBuilder(EntityIdRegistry & registry);

  // Set the timestamp (should match the GroundTruth timestamp)
  auto setTimestamp(double simulation_time) -> TrafficUpdateBuilder &;

  // Set the updated ego vehicle state (after dynamics computation)
  auto setEgoUpdate(const EntityData & ego_entity) -> TrafficUpdateBuilder &;

  // Set the host vehicle internal state (optional, for internal sensor data)
  auto setHostVehicleData(const osi3::HostVehicleData & data) -> TrafficUpdateBuilder &;

  // Build the TrafficUpdate message
  auto build() -> osi3::TrafficUpdate;

private:
  EntityIdRegistry & registry_;
  double simulation_time_{0.0};
  EntityData ego_entity_;
  bool has_ego_{false};
  osi3::HostVehicleData host_vehicle_data_;
  bool has_host_vehicle_data_{false};
};

}  // namespace osi_interface

#endif  // OSI_INTERFACE__TRAFFIC_UPDATE_BUILDER_HPP_
