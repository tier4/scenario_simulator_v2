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

#include <simulation_interface/conversions.hpp>
#include <simulation_interface/zmq_multi_client.hpp>
#include <string>
namespace zeromq
{
void MultiClient::call(
  const simulation_api_schema::InitializeRequest & req,
  simulation_api_schema::InitializeResponse & res)
{
  zmqpp::message message = toZMQ(req);
  socket_initialize_.send(message);
  zmqpp::message buffer;
  socket_initialize_.receive(buffer);
  res = toProto<simulation_api_schema::InitializeResponse>(buffer);
}
void MultiClient::call(
  const simulation_api_schema::UpdateFrameRequest & req,
  simulation_api_schema::UpdateFrameResponse & res)
{
  zmqpp::message message = toZMQ(req);
  socket_update_frame_.send(message);
  zmqpp::message buffer;
  socket_update_frame_.receive(buffer);
  res = toProto<simulation_api_schema::UpdateFrameResponse>(buffer);
}
void MultiClient::call(
  const simulation_api_schema::UpdateSensorFrameRequest & req,
  simulation_api_schema::UpdateSensorFrameResponse & res)
{
  zmqpp::message message = toZMQ(req);
  socket_update_sensor_frame_.send(message);
  zmqpp::message buffer;
  socket_update_sensor_frame_.receive(buffer);
  res = toProto<simulation_api_schema::UpdateSensorFrameResponse>(buffer);
}
void MultiClient::call(
  const simulation_api_schema::SpawnVehicleEntityRequest & req,
  simulation_api_schema::SpawnVehicleEntityResponse & res)
{
  zmqpp::message message = toZMQ(req);
  socket_spawn_vehicle_entity_.send(message);
  zmqpp::message buffer;
  socket_spawn_vehicle_entity_.receive(buffer);
  res = toProto<simulation_api_schema::SpawnVehicleEntityResponse>(buffer);
}
void MultiClient::call(
  const simulation_api_schema::SpawnPedestrianEntityRequest & req,
  simulation_api_schema::SpawnPedestrianEntityResponse & res)
{
  zmqpp::message message = toZMQ(req);
  socket_spawn_pedestrian_entity_.send(message);
  zmqpp::message buffer;
  socket_spawn_pedestrian_entity_.receive(buffer);
  res = toProto<simulation_api_schema::SpawnPedestrianEntityResponse>(buffer);
}
void MultiClient::call(
  const simulation_api_schema::SpawnMiscObjectEntityRequest & req,
  simulation_api_schema::SpawnMiscObjectEntityResponse & res)
{
  zmqpp::message message = toZMQ(req);
  socket_spawn_misc_object_entity_.send(message);
  zmqpp::message buffer;
  socket_spawn_misc_object_entity_.receive(buffer);
  res = toProto<simulation_api_schema::SpawnMiscObjectEntityResponse>(buffer);
}
void MultiClient::call(
  const simulation_api_schema::DespawnEntityRequest & req,
  simulation_api_schema::DespawnEntityResponse & res)
{
  zmqpp::message message = toZMQ(req);
  socket_despawn_entity_.send(message);
  zmqpp::message buffer;
  socket_despawn_entity_.receive(buffer);
  res = toProto<simulation_api_schema::DespawnEntityResponse>(buffer);
}
void MultiClient::call(
  const simulation_api_schema::UpdateEntityStatusRequest & req,
  simulation_api_schema::UpdateEntityStatusResponse & res)
{
  zmqpp::message message = toZMQ(req);
  socket_update_entity_status_.send(message);
  zmqpp::message buffer;
  socket_update_entity_status_.receive(buffer);
  res = toProto<simulation_api_schema::UpdateEntityStatusResponse>(buffer);
}
void MultiClient::call(
  const simulation_api_schema::AttachLidarSensorRequest & req,
  simulation_api_schema::AttachLidarSensorResponse & res)
{
  zmqpp::message message = toZMQ(req);
  socket_attach_lidar_sensor_.send(message);
  zmqpp::message buffer;
  socket_attach_lidar_sensor_.receive(buffer);
  res = toProto<simulation_api_schema::AttachLidarSensorResponse>(buffer);
}
void MultiClient::call(
  const simulation_api_schema::AttachDetectionSensorRequest & req,
  simulation_api_schema::AttachDetectionSensorResponse & res)
{
  zmqpp::message message = toZMQ(req);
  socket_attach_detection_sensor_.send(message);
  zmqpp::message buffer;
  socket_attach_detection_sensor_.receive(buffer);
  res = toProto<simulation_api_schema::AttachDetectionSensorResponse>(buffer);
}
void MultiClient::call(
  const simulation_api_schema::UpdateTrafficLightsRequest & req,
  simulation_api_schema::UpdateTrafficLightsResponse & res)
{
  zmqpp::message message = toZMQ(req);
  socket_update_traffic_lights_.send(message);
  zmqpp::message buffer;
  socket_update_traffic_lights_.receive(buffer);
  res = toProto<simulation_api_schema::UpdateTrafficLightsResponse>(buffer);
}
}  // namespace zeromq