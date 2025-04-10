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
      case simulation_api_schema::SimulationRequest::RequestCase::kInitialize:
        *sim_response.mutable_initialize() = std::get<Initialize>(functions_)(proto.initialize());
        break;
      case simulation_api_schema::SimulationRequest::RequestCase::kUpdateFrame:
        *sim_response.mutable_update_frame() =
          std::get<UpdateFrame>(functions_)(proto.update_frame());
        break;
      case simulation_api_schema::SimulationRequest::RequestCase::kSpawnVehicleEntity:
        *sim_response.mutable_spawn_vehicle_entity() =
          std::get<SpawnVehicleEntity>(functions_)(proto.spawn_vehicle_entity());
        break;
      case simulation_api_schema::SimulationRequest::RequestCase::kSpawnPedestrianEntity:
        *sim_response.mutable_spawn_pedestrian_entity() =
          std::get<SpawnPedestrianEntity>(functions_)(proto.spawn_pedestrian_entity());
        break;
      case simulation_api_schema::SimulationRequest::RequestCase::kSpawnMiscObjectEntity:
        *sim_response.mutable_spawn_misc_object_entity() =
          std::get<SpawnMiscObjectEntity>(functions_)(proto.spawn_misc_object_entity());
        break;
      case simulation_api_schema::SimulationRequest::RequestCase::kDespawnEntity:
        *sim_response.mutable_despawn_entity() =
          std::get<DespawnEntity>(functions_)(proto.despawn_entity());
        break;
      case simulation_api_schema::SimulationRequest::RequestCase::kUpdateEntityStatus:
        *sim_response.mutable_update_entity_status() =
          std::get<UpdateEntityStatus>(functions_)(proto.update_entity_status());
        break;
      case simulation_api_schema::SimulationRequest::RequestCase::kAttachImuSensor:
        *sim_response.mutable_attach_imu_sensor() =
          std::get<AttachImuSensor>(functions_)(proto.attach_imu_sensor());
        break;
      case simulation_api_schema::SimulationRequest::RequestCase::kAttachLidarSensor:
        *sim_response.mutable_attach_lidar_sensor() =
          std::get<AttachLidarSensor>(functions_)(proto.attach_lidar_sensor());
        break;
      case simulation_api_schema::SimulationRequest::RequestCase::kAttachDetectionSensor:
        *sim_response.mutable_attach_detection_sensor() =
          std::get<AttachDetectionSensor>(functions_)(proto.attach_detection_sensor());
        break;
      case simulation_api_schema::SimulationRequest::RequestCase::kAttachOccupancyGridSensor:
        *sim_response.mutable_attach_occupancy_grid_sensor() =
          std::get<AttachOccupancyGridSensor>(functions_)(proto.attach_occupancy_grid_sensor());
        break;
      case simulation_api_schema::SimulationRequest::RequestCase::kUpdateTrafficLights:
        *sim_response.mutable_update_traffic_lights() =
          std::get<UpdateTrafficLights>(functions_)(proto.update_traffic_lights());
        break;
      case simulation_api_schema::SimulationRequest::RequestCase::kAttachPseudoTrafficLightDetector:
        *sim_response.mutable_attach_pseudo_traffic_light_detector() =
          std::get<AttachPseudoTrafficLightDetector>(functions_)(
            proto.attach_pseudo_traffic_light_detector());
        break;
      case simulation_api_schema::SimulationRequest::RequestCase::kUpdateStepTime:
        *sim_response.mutable_update_step_time() =
          std::get<UpdateStepTime>(functions_)(proto.update_step_time());
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
