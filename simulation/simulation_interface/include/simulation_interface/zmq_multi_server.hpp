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

#ifndef SIMULATION_INTERFACE__ZMQ_MULTI_SERVER_HPP_
#define SIMULATION_INTERFACE__ZMQ_MULTI_SERVER_HPP_

#include <simulation_api_schema.pb.h>

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <simulation_interface/constants.hpp>
#include <string>
#include <thread>
#include <zmqpp/zmqpp.hpp>

namespace zeromq
{
class MultiServer
{
public:
  explicit MultiServer(
    const simulation_interface::TransportProtocol & protocol,
    const simulation_interface::HostName & hostname, const unsigned int socket_port,
    std::function<void(
      const simulation_api_schema::InitializeRequest &,
      simulation_api_schema::InitializeResponse &)>
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
      update_traffic_lights_func);
  ~MultiServer();

private:
  void poll();
  void start_poll();
  std::thread thread_;
  const zmqpp::context context_;
  const zmqpp::socket_type type_;
  zmqpp::poller poller_;
  zmqpp::socket socket_;
  std::function<void(
    const simulation_api_schema::InitializeRequest &, simulation_api_schema::InitializeResponse &)>
    initialize_func_;
  std::function<void(
    const simulation_api_schema::UpdateFrameRequest &,
    simulation_api_schema::UpdateFrameResponse &)>
    update_frame_func_;
  std::function<void(
    const simulation_api_schema::SpawnVehicleEntityRequest &,
    simulation_api_schema::SpawnVehicleEntityResponse &)>
    spawn_vehicle_entity_func_;
  std::function<void(
    const simulation_api_schema::SpawnPedestrianEntityRequest &,
    simulation_api_schema::SpawnPedestrianEntityResponse &)>
    spawn_pedestrian_entity_func_;
  std::function<void(
    const simulation_api_schema::SpawnMiscObjectEntityRequest &,
    simulation_api_schema::SpawnMiscObjectEntityResponse &)>
    spawn_misc_object_entity_func_;
  std::function<void(
    const simulation_api_schema::DespawnEntityRequest &,
    simulation_api_schema::DespawnEntityResponse &)>
    despawn_entity_func_;
  std::function<void(
    const simulation_api_schema::UpdateEntityStatusRequest &,
    simulation_api_schema::UpdateEntityStatusResponse &)>
    update_entity_status_func_;
  std::function<void(
    const simulation_api_schema::AttachLidarSensorRequest &,
    simulation_api_schema::AttachLidarSensorResponse &)>
    attach_lidar_sensor_func_;
  std::function<void(
    const simulation_api_schema::AttachDetectionSensorRequest &,
    simulation_api_schema::AttachDetectionSensorResponse &)>
    attach_detection_sensor_func_;
  std::function<void(
    const simulation_api_schema::AttachOccupancyGridSensorRequest &,
    simulation_api_schema::AttachOccupancyGridSensorResponse &)>
    attach_occupancy_grid_sensor_func_;
  std::function<void(
    const simulation_api_schema::UpdateTrafficLightsRequest &,
    simulation_api_schema::UpdateTrafficLightsResponse &)>
    update_traffic_lights_func_;
};
}  // namespace zeromq

#endif  // SIMULATION_INTERFACE__ZMQ_MULTI_SERVER_HPP_
