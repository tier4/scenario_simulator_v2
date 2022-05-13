// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#ifndef SIMULATION_INTERFACE__CONSTANTS_HPP_
#define SIMULATION_INTERFACE__CONSTANTS_HPP_

#include <string>

namespace simulation_interface
{
enum class TransportProtocol { TCP /*, UDP*/ };

std::string enumToString(const TransportProtocol & protocol);

enum class HostName { LOCALHOST, ANY };

std::string enumToString(const HostName & hostname);

const TransportProtocol protocol = TransportProtocol::TCP;

namespace ports
{
const unsigned int initialize = 5555;
const unsigned int update_frame = 5556;
const unsigned int update_sensor_frame = 5557;
const unsigned int spawn_vehicle_entity = 5558;
const unsigned int spawn_pedestrian_entity = 5559;
const unsigned int spawn_misc_object_entity = 5560;
const unsigned int despawn_entity = 5561;
const unsigned int update_entity_status = 5562;
const unsigned int attach_lidar_sensor = 5563;
const unsigned int attach_detection_sensor = 5564;
const unsigned int attach_occupancy_grid_sensor = 5565;
const unsigned int update_traffic_lights = 5566;
}  // namespace ports

std::string getEndPoint(
  const TransportProtocol & protocol, const HostName & hostname, const unsigned int & port);

std::string getEndPoint(
  const TransportProtocol & protocol, const std::string & hostname, const unsigned int & port);
}  // namespace simulation_interface

#endif  // SIMULATION_INTERFACE__CONSTANTS_HPP_
