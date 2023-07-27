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

#include <simulation_interface/conversions.hpp>
#include <simulation_interface/zmq_multi_server.hpp>
#include <status_monitor/status_monitor.hpp>

namespace zeromq
{
MultiServer::MultiServer(
  const simulation_interface::TransportProtocol & protocol,
  const simulation_interface::HostName & hostname, const unsigned int socket_port,
  std::function<void(
    const simulation_api_schema::InitializeRequest &, simulation_api_schema::InitializeResponse &)>
    initialize_func,
  std::function<void(
    const simulation_api_schema::UpdateFrameRequest &,
    simulation_api_schema::UpdateFrameResponse &)>
    update_frame_func,
  std::function<void(
    const simulation_api_schema::SpawnVehicleEntityRequest &,
    simulation_api_schema::SpawnVehicleEntityResponse &)>
    spawn_vehicle_entity_func,
  std::function<void(
    const simulation_api_schema::SpawnPedestrianEntityRequest &,
    simulation_api_schema::SpawnPedestrianEntityResponse &)>
    spawn_pedestrian_entity_func,
  std::function<void(
    const simulation_api_schema::SpawnMiscObjectEntityRequest &,
    simulation_api_schema::SpawnMiscObjectEntityResponse &)>
    spawn_misc_object_entity_func,
  std::function<void(
    const simulation_api_schema::DespawnEntityRequest &,
    simulation_api_schema::DespawnEntityResponse &)>
    despawn_entity_func,
  std::function<void(
    const simulation_api_schema::UpdateEntityStatusRequest &,
    simulation_api_schema::UpdateEntityStatusResponse &)>
    update_entity_status_func,
  std::function<void(
    const simulation_api_schema::AttachLidarSensorRequest &,
    simulation_api_schema::AttachLidarSensorResponse &)>
    attach_lidar_sensor_func,
  std::function<void(
    const simulation_api_schema::AttachDetectionSensorRequest &,
    simulation_api_schema::AttachDetectionSensorResponse &)>
    attach_detection_sensor_func,
  std::function<void(
    const simulation_api_schema::AttachOccupancyGridSensorRequest &,
    simulation_api_schema::AttachOccupancyGridSensorResponse &)>
    attach_occupancy_sensor_func,
  std::function<void(
    const simulation_api_schema::UpdateTrafficLightsRequest &,
    simulation_api_schema::UpdateTrafficLightsResponse &)>
    update_traffic_lights_func)
: context_(zmqpp::context()),
  type_(zmqpp::socket_type::reply),
  socket_(context_, type_),
  initialize_func_(initialize_func),
  update_frame_func_(update_frame_func),
  spawn_vehicle_entity_func_(spawn_vehicle_entity_func),
  spawn_pedestrian_entity_func_(spawn_pedestrian_entity_func),
  spawn_misc_object_entity_func_(spawn_misc_object_entity_func),
  despawn_entity_func_(despawn_entity_func),
  update_entity_status_func_(update_entity_status_func),
  attach_lidar_sensor_func_(attach_lidar_sensor_func),
  attach_detection_sensor_func_(attach_detection_sensor_func),
  attach_occupancy_grid_sensor_func_(attach_occupancy_sensor_func),
  update_traffic_lights_func_(update_traffic_lights_func)
{
  socket_.bind(simulation_interface::getEndPoint(protocol, hostname, socket_port));
  poller_.add(socket_);
  thread_ = std::thread(&MultiServer::start_poll, this);
}

MultiServer::~MultiServer() { thread_.join(); }

void MultiServer::poll()
{
  constexpr long timeout_ms = 1L;
  poller_.poll(timeout_ms);
  if (poller_.has_input(socket_)) {
    simulation_api_schema::SimulationResponse sim_response;
    zmqpp::message sim_request;
    socket_.receive(sim_request);
    auto proto = toProto<simulation_api_schema::SimulationRequest>(sim_request);
    switch (proto.request_case()) {
      case simulation_api_schema::SimulationRequest::RequestCase::kInitialize: {
        simulation_api_schema::InitializeResponse response;
        initialize_func_(proto.initialize(), response);
        *sim_response.mutable_initialize() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kUpdateFrame: {
        simulation_api_schema::UpdateFrameResponse response;
        update_frame_func_(proto.update_frame(), response);
        *sim_response.mutable_update_frame() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kSpawnVehicleEntity: {
        simulation_api_schema::SpawnVehicleEntityResponse response;
        spawn_vehicle_entity_func_(proto.spawn_vehicle_entity(), response);
        *sim_response.mutable_spawn_vehicle_entity() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kSpawnPedestrianEntity: {
        simulation_api_schema::SpawnPedestrianEntityResponse response;
        spawn_pedestrian_entity_func_(proto.spawn_pedestrian_entity(), response);
        *sim_response.mutable_spawn_pedestrian_entity() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kSpawnMiscObjectEntity: {
        simulation_api_schema::SpawnMiscObjectEntityResponse response;
        spawn_misc_object_entity_func_(proto.spawn_misc_object_entity(), response);
        *sim_response.mutable_spawn_misc_object_entity() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kDespawnEntity: {
        simulation_api_schema::DespawnEntityResponse response;
        despawn_entity_func_(proto.despawn_entity(), response);
        *sim_response.mutable_despawn_entity() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kUpdateEntityStatus: {
        simulation_api_schema::UpdateEntityStatusResponse response;
        update_entity_status_func_(proto.update_entity_status(), response);
        *sim_response.mutable_update_entity_status() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kAttachLidarSensor: {
        simulation_api_schema::AttachLidarSensorResponse response;
        attach_lidar_sensor_func_(proto.attach_lidar_sensor(), response);
        *sim_response.mutable_attach_lidar_sensor() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kAttachDetectionSensor: {
        simulation_api_schema::AttachDetectionSensorResponse response;
        attach_detection_sensor_func_(proto.attach_detection_sensor(), response);
        *sim_response.mutable_attach_detection_sensor() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kAttachOccupancyGridSensor: {
        simulation_api_schema::AttachOccupancyGridSensorResponse response;
        attach_occupancy_grid_sensor_func_(proto.attach_occupancy_grid_sensor(), response);
        *sim_response.mutable_attach_occupancy_grid_sensor() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kUpdateTrafficLights: {
        simulation_api_schema::UpdateTrafficLightsResponse response;
        update_traffic_lights_func_(proto.update_traffic_lights(), response);
        *sim_response.mutable_update_traffic_lights() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::REQUEST_NOT_SET: {
        THROW_SIMULATION_ERROR("No case defined for oneof in SimulationRequest message");
      }
    }
    auto msg = toZMQ(sim_response);
    socket_.send(msg);
  }
}

void MultiServer::start_poll()
{
  while (rclcpp::ok()) {
    common::status_monitor.touch(__func__);
    poll();
  }
}
}  // namespace zeromq
