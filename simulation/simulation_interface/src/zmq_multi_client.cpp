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

void MultiClient::closeConnection() { socket_.close(); }

MultiClient::~MultiClient() { closeConnection(); }

auto MultiClient::call(const simulation_api_schema::SimulationRequest & req)
  -> simulation_api_schema::SimulationResponse
{
  zmqpp::message message = toZMQ(req);
  socket_.send(message);
  zmqpp::message buffer;
  socket_.receive(buffer);
  return toProto<simulation_api_schema::SimulationResponse>(buffer);
}

auto MultiClient::call(const simulation_api_schema::InitializeRequest & request)
  -> simulation_api_schema::InitializeResponse
{
  simulation_api_schema::SimulationRequest sim_request;
  *sim_request.mutable_initialize() = request;
  return call(sim_request).initialize();
}

auto MultiClient::call(const simulation_api_schema::UpdateFrameRequest & request)
  -> simulation_api_schema::UpdateFrameResponse
{
  simulation_api_schema::SimulationRequest sim_request;
  *sim_request.mutable_update_frame() = request;
  return call(sim_request).update_frame();
}

auto MultiClient::call(const simulation_api_schema::SpawnVehicleEntityRequest & request)
  -> simulation_api_schema::SpawnVehicleEntityResponse
{
  simulation_api_schema::SimulationRequest sim_request;
  *sim_request.mutable_spawn_vehicle_entity() = request;
  return call(sim_request).spawn_vehicle_entity();
}

auto MultiClient::call(const simulation_api_schema::SpawnPedestrianEntityRequest & request)
  -> simulation_api_schema::SpawnPedestrianEntityResponse
{
  simulation_api_schema::SimulationRequest sim_request;
  *sim_request.mutable_spawn_pedestrian_entity() = request;
  return call(sim_request).spawn_pedestrian_entity();
}

auto MultiClient::call(const simulation_api_schema::SpawnMiscObjectEntityRequest & request)
  -> simulation_api_schema::SpawnMiscObjectEntityResponse
{
  simulation_api_schema::SimulationRequest sim_request;
  *sim_request.mutable_spawn_misc_object_entity() = request;
  return call(sim_request).spawn_misc_object_entity();
}

auto MultiClient::call(const simulation_api_schema::DespawnEntityRequest & request)
  -> simulation_api_schema::DespawnEntityResponse
{
  simulation_api_schema::SimulationRequest sim_request;
  *sim_request.mutable_despawn_entity() = request;
  return call(sim_request).despawn_entity();
}

auto MultiClient::call(const simulation_api_schema::UpdateEntityStatusRequest & request)
  -> simulation_api_schema::UpdateEntityStatusResponse
{
  simulation_api_schema::SimulationRequest sim_request;
  *sim_request.mutable_update_entity_status() = request;
  return call(sim_request).update_entity_status();
}

auto MultiClient::call(const simulation_api_schema::AttachLidarSensorRequest & request)
  -> simulation_api_schema::AttachLidarSensorResponse
{
  simulation_api_schema::SimulationRequest sim_request;
  *sim_request.mutable_attach_lidar_sensor() = request;
  return call(sim_request).attach_lidar_sensor();
}

auto MultiClient::call(const simulation_api_schema::AttachDetectionSensorRequest & request)
  -> simulation_api_schema::AttachDetectionSensorResponse
{
  simulation_api_schema::SimulationRequest sim_request;
  *sim_request.mutable_attach_detection_sensor() = request;
  return call(sim_request).attach_detection_sensor();
}

auto MultiClient::call(const simulation_api_schema::AttachOccupancyGridSensorRequest & request)
  -> simulation_api_schema::AttachOccupancyGridSensorResponse
{
  simulation_api_schema::SimulationRequest sim_request;
  *sim_request.mutable_attach_occupancy_grid_sensor() = request;
  return call(sim_request).attach_occupancy_grid_sensor();
}

auto MultiClient::call(const simulation_api_schema::UpdateTrafficLightsRequest & request)
  -> simulation_api_schema::UpdateTrafficLightsResponse
{
  simulation_api_schema::SimulationRequest sim_request;
  *sim_request.mutable_update_traffic_lights() = request;
  return call(sim_request).update_traffic_lights();
}

auto MultiClient::call(const simulation_api_schema::FollowPolylineTrajectoryRequest & request)
  -> simulation_api_schema::FollowPolylineTrajectoryResponse
{
  auto simulation_request = simulation_api_schema::SimulationRequest();
  *simulation_request.mutable_follow_polyline_trajectory() = request;
  return call(simulation_request).follow_polyline_trajectory();
}
}  // namespace zeromq
