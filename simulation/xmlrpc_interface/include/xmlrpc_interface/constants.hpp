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

#ifndef XMLRPC_INTERFACE__CONSTANTS_HPP_
#define XMLRPC_INTERFACE__CONSTANTS_HPP_

namespace xmlrpc_interface
{
namespace key
{
const char success[] = "success";
const char description[] = "description";
const char realtime_factor[] = "realtime_factor";
const char step_time[] = "step_time";
const char current_time[] = "current_time";
const char method_name[] = "methodName";
const char parameters[] = "params";
const char response[] = "return";
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
}  // namespace method
}  // namespace xmlrpc_interface

#endif  // XMLRPC_INTERFACE__CONSTANTS_HPP_
