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

#ifndef SIMPLE_SENSOR_SIMULATOR__OSI_BRIDGE_HPP_
#define SIMPLE_SENSOR_SIMULATOR__OSI_BRIDGE_HPP_

#include <simulation_interface/simulation_api_schema.pb.h>

#include <osi_interface/osi_entity_conversions.hpp>
#include <osi_interface/osi_traffic_light_conversions.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>

namespace simple_sensor_simulator
{
namespace osi_bridge
{
// EntityData → simulation_api_schema::EntityStatus (for entity_status_ map)
auto toProtoEntityStatus(const osi_interface::EntityData & entity)
  -> simulation_api_schema::EntityStatus;

// EntityData → traffic_simulator_msgs::msg::EntityStatus (for EgoEntitySimulation)
auto toRosMsgEntityStatus(const osi_interface::EntityData & entity)
  -> traffic_simulator_msgs::msg::EntityStatus;

// traffic_simulator_msgs::msg::EntityStatus → EntityData (for TrafficUpdate from ego)
auto fromRosMsgEntityStatus(const traffic_simulator_msgs::msg::EntityStatus & msg)
  -> osi_interface::EntityData;

// Create default VehicleParameters from EntityData (for EgoEntitySimulation initialization)
auto makeDefaultVehicleParameters(const osi_interface::EntityData & entity)
  -> traffic_simulator_msgs::msg::VehicleParameters;

// TrafficSignalGroup[] → simulation_api_schema::UpdateTrafficLightsRequest
auto toProtoTrafficLightsRequest(const std::vector<osi_interface::TrafficSignalGroup> & signals)
  -> simulation_api_schema::UpdateTrafficLightsRequest;

}  // namespace osi_bridge
}  // namespace simple_sensor_simulator

#endif  // SIMPLE_SENSOR_SIMULATOR__OSI_BRIDGE_HPP_
