// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <rclcpp/utilities.hpp>
#include <simulation_interface/conversions.hpp>
#include <simulation_interface/zmq_multi_client.hpp>
#include <string>
namespace zeromq
{
MultiClient::MultiClient(
  const simulation_interface::TransportProtocol & protocol, const std::string & hostname,
  const unsigned int socket_port)
: protocol(protocol),
  hostname(hostname),
  context_(zmqpp::context()),
  type_(zmqpp::socket_type::request),
  socket_(context_, type_)
{
  socket_.connect(simulation_interface::getEndPoint(protocol, hostname, socket_port));
}

void MultiClient::closeConnection()
{
  if (is_running) {
    is_running = false;
    socket_.close();
  }
}

MultiClient::~MultiClient() { closeConnection(); }

void MultiClient::call(
  const simulation_api_schema::SimulationRequest & req,
  simulation_api_schema::SimulationResponse & res)
{
  zmqpp::message message = toZMQ(req);
  socket_.send(message);
  zmqpp::message buffer;
  socket_.receive(buffer);
  res = toProto<simulation_api_schema::SimulationResponse>(buffer);
}

void MultiClient::call(
  const simulation_api_schema::InitializeRequest & req,
  simulation_api_schema::InitializeResponse & res)
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    simulation_api_schema::SimulationResponse sim_response;
    *sim_request.mutable_initialize() = req;
    call(sim_request, sim_response);
    res = sim_response.initialize();
  }
}
void MultiClient::call(
  const simulation_api_schema::UpdateFrameRequest & req,
  simulation_api_schema::UpdateFrameResponse & res)
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    simulation_api_schema::SimulationResponse sim_response;
    *sim_request.mutable_update_frame() = req;
    call(sim_request, sim_response);
    res = sim_response.update_frame();
  }
}
void MultiClient::call(
  const simulation_api_schema::SpawnVehicleEntityRequest & req,
  simulation_api_schema::SpawnVehicleEntityResponse & res)
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    simulation_api_schema::SimulationResponse sim_response;
    *sim_request.mutable_spawn_vehicle_entity() = req;
    call(sim_request, sim_response);
    res = sim_response.spawn_vehicle_entity();
  }
}
void MultiClient::call(
  const simulation_api_schema::SpawnPedestrianEntityRequest & req,
  simulation_api_schema::SpawnPedestrianEntityResponse & res)
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    simulation_api_schema::SimulationResponse sim_response;
    *sim_request.mutable_spawn_pedestrian_entity() = req;
    call(sim_request, sim_response);
    res = sim_response.spawn_pedestrian_entity();
  }
}
void MultiClient::call(
  const simulation_api_schema::SpawnMiscObjectEntityRequest & req,
  simulation_api_schema::SpawnMiscObjectEntityResponse & res)
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    simulation_api_schema::SimulationResponse sim_response;
    *sim_request.mutable_spawn_misc_object_entity() = req;
    call(sim_request, sim_response);
    res = sim_response.spawn_misc_object_entity();
  }
}
void MultiClient::call(
  const simulation_api_schema::DespawnEntityRequest & req,
  simulation_api_schema::DespawnEntityResponse & res)
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    simulation_api_schema::SimulationResponse sim_response;
    *sim_request.mutable_despawn_entity() = req;
    call(sim_request, sim_response);
    res = sim_response.despawn_entity();
  }
}
void MultiClient::call(
  const simulation_api_schema::UpdateEntityStatusRequest & req,
  simulation_api_schema::UpdateEntityStatusResponse & res)
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    simulation_api_schema::SimulationResponse sim_response;
    *sim_request.mutable_update_entity_status() = req;
    call(sim_request, sim_response);
    res = sim_response.update_entity_status();
  }
}
void MultiClient::call(
  const simulation_api_schema::AttachLidarSensorRequest & req,
  simulation_api_schema::AttachLidarSensorResponse & res)
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    simulation_api_schema::SimulationResponse sim_response;
    *sim_request.mutable_attach_lidar_sensor() = req;
    call(sim_request, sim_response);
    res = sim_response.attach_lidar_sensor();
  }
}
void MultiClient::call(
  const simulation_api_schema::AttachDetectionSensorRequest & req,
  simulation_api_schema::AttachDetectionSensorResponse & res)
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    simulation_api_schema::SimulationResponse sim_response;
    *sim_request.mutable_attach_detection_sensor() = req;
    call(sim_request, sim_response);
    res = sim_response.attach_detection_sensor();
  }
}

void MultiClient::call(
  const simulation_api_schema::AttachOccupancyGridSensorRequest & req,
  simulation_api_schema::AttachOccupancyGridSensorResponse & res)
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    simulation_api_schema::SimulationResponse sim_response;
    *sim_request.mutable_attach_occupancy_grid_sensor() = req;
    call(sim_request, sim_response);
    res = sim_response.attach_occupancy_grid_sensor();
  }
}

void MultiClient::call(
  const simulation_api_schema::UpdateTrafficLightsRequest & req,
  simulation_api_schema::UpdateTrafficLightsResponse & res)
{
  if (is_running) {
    simulation_api_schema::SimulationRequest sim_request;
    simulation_api_schema::SimulationResponse sim_response;
    *sim_request.mutable_update_traffic_lights() = req;
    call(sim_request, sim_response);
    res = sim_response.update_traffic_lights();
  }
}

}  // namespace zeromq
