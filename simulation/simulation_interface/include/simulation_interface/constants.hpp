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
enum class TransportProtocol
{
  TCP,
  UDP
};

std::string enumToString(const TransportProtocol & protocol);

enum class HostName
{
  LOCLHOST,
  ANY
};

std::string enumToString(const HostName & hostname);

namespace ports
{
const unsigned int initialize = 5555;
const unsigned int update_frame = 5556;
const unsigned int update_sensor_frame = 5557;
const unsigned int spawn_vehicle_entity = 5558;
const unsigned int spawn_pedestrian_entity = 5559;
const unsigned int despawn_entity = 5560;
const unsigned int update_entity_status = 5561;
const unsigned int attach_lidar_sensor = 5562;
const unsigned int attach_detection_sensor = 5563;
}  // namespace ports

namespace key
{
const char success[] = "success";
const char description[] = "description";
const char realtime_factor[] = "realtime_factor";
const char step_time[] = "step_time";
const char current_time[] = "current_time";
const char method_name[] = "methodName";
}  // namespace key

namespace method
{
const char initialize[] = "initialize";
const char update_frame[] = "update_frame";
const char update_sensor_frame[] = "update_sensor_frame";
const char spawn_vehicle_entity[] = "spawn_vehicle_entity";
const char spawn_pedestrian_entity[] = "spawn_pedestrian_entity";
const char despawn_entity[] = "despawn_entity";
const char update_entity_status[] = "update_entity_status";
const char attach_lidar_sensor[] = "attach_lidar_sensor";
const char attach_detection_sensor[] = "attach_detection_sensor";
}  // namespace method
}  // namespace simulation_interface

#endif  // SIMULATION_INTERFACE__CONSTANTS_HPP_
