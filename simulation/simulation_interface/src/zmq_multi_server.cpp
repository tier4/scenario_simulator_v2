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
        std::get<Initialize>(functions_)(proto.initialize(), response);
        *sim_response.mutable_initialize() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kUpdateFrame: {
        simulation_api_schema::UpdateFrameResponse response;
        std::get<UpdateFrame>(functions_)(proto.update_frame(), response);
        *sim_response.mutable_update_frame() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kSpawnVehicleEntity: {
        simulation_api_schema::SpawnVehicleEntityResponse response;
        std::get<SpawnVehicleEntity>(functions_)(proto.spawn_vehicle_entity(), response);
        *sim_response.mutable_spawn_vehicle_entity() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kSpawnPedestrianEntity: {
        simulation_api_schema::SpawnPedestrianEntityResponse response;
        std::get<SpawnPedestrianEntity>(functions_)(proto.spawn_pedestrian_entity(), response);
        *sim_response.mutable_spawn_pedestrian_entity() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kSpawnMiscObjectEntity: {
        simulation_api_schema::SpawnMiscObjectEntityResponse response;
        std::get<SpawnMiscObjectEntity>(functions_)(proto.spawn_misc_object_entity(), response);
        *sim_response.mutable_spawn_misc_object_entity() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kDespawnEntity: {
        simulation_api_schema::DespawnEntityResponse response;
        std::get<DespawnEntity>(functions_)(proto.despawn_entity(), response);
        *sim_response.mutable_despawn_entity() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kUpdateEntityStatus: {
        simulation_api_schema::UpdateEntityStatusResponse response;
        std::get<UpdateEntityStatus>(functions_)(proto.update_entity_status(), response);
        *sim_response.mutable_update_entity_status() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kAttachLidarSensor: {
        simulation_api_schema::AttachLidarSensorResponse response;
        std::get<AttachLidarSensor>(functions_)(proto.attach_lidar_sensor(), response);
        *sim_response.mutable_attach_lidar_sensor() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kAttachDetectionSensor: {
        simulation_api_schema::AttachDetectionSensorResponse response;
        std::get<AttachDetectionSensor>(functions_)(proto.attach_detection_sensor(), response);
        *sim_response.mutable_attach_detection_sensor() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kAttachOccupancyGridSensor: {
        simulation_api_schema::AttachOccupancyGridSensorResponse response;
        std::get<AttachOccupancyGridSensor>(functions_)(
          proto.attach_occupancy_grid_sensor(), response);
        *sim_response.mutable_attach_occupancy_grid_sensor() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kUpdateTrafficLights: {
        simulation_api_schema::UpdateTrafficLightsResponse response;
        std::get<UpdateTrafficLights>(functions_)(proto.update_traffic_lights(), response);
        *sim_response.mutable_update_traffic_lights() = response;
        break;
      }
      case simulation_api_schema::SimulationRequest::RequestCase::kFollowPolylineTrajectory:
        *sim_response.mutable_follow_polyline_trajectory() =
          std::get<FollowPolylineTrajectory>(functions_)(proto.follow_polyline_trajectory());
        break;
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
