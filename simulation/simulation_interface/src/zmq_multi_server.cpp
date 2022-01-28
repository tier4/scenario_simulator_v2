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
#include <simulation_interface/zmq_multi_server.hpp>
#include <string>

namespace zeromq
{
MultiServer::MultiServer(
  const simulation_interface::TransportProtocol & protocol,
  const simulation_interface::HostName & hostname,
  std::function<void(
    const simulation_api_schema::InitializeRequest &, simulation_api_schema::InitializeResponse &)>
    initialize_func,
  std::function<void(
    const simulation_api_schema::UpdateFrameRequest &,
    simulation_api_schema::UpdateFrameResponse &)>
    update_frame_func,
  std::function<void(
    const simulation_api_schema::UpdateSensorFrameRequest &,
    simulation_api_schema::UpdateSensorFrameResponse &)>
    update_sensor_frame_func,
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
    const simulation_api_schema::UpdateTrafficLightsRequest &,
    simulation_api_schema::UpdateTrafficLightsResponse &)>
    update_traffic_lights_func)
: context_(zmqpp::context()),
  type_(zmqpp::socket_type::reply),
  initialize_sock_(context_, type_),
  initialize_func_(initialize_func),
  update_frame_sock_(context_, type_),
  update_frame_func_(update_frame_func),
  update_sensor_frame_sock_(context_, type_),
  update_sensor_frame_func_(update_sensor_frame_func),
  spawn_vehicle_entity_sock_(context_, type_),
  spawn_vehicle_entity_func_(spawn_vehicle_entity_func),
  spawn_pedestrian_entity_sock_(context_, type_),
  spawn_pedestrian_entity_func_(spawn_pedestrian_entity_func),
  spawn_misc_object_entity_sock_(context_, type_),
  spawn_misc_object_entity_func_(spawn_misc_object_entity_func),
  despawn_entity_sock_(context_, type_),
  despawn_entity_func_(despawn_entity_func),
  update_entity_status_sock_(context_, type_),
  update_entity_status_func_(update_entity_status_func),
  attach_lidar_sensor_sock_(context_, type_),
  attach_lidar_sensor_func_(attach_lidar_sensor_func),
  attach_detection_sensor_sock_(context_, type_),
  attach_detection_sensor_func_(attach_detection_sensor_func),
  update_traffic_lights_sock_(context_, type_),
  update_traffic_lights_func_(update_traffic_lights_func)
{
  initialize_sock_.bind(
    simulation_interface::getEndPoint(protocol, hostname, simulation_interface::ports::initialize));
  update_entity_status_sock_.bind(simulation_interface::getEndPoint(
    protocol, hostname, simulation_interface::ports::update_entity_status));
  update_frame_sock_.bind(simulation_interface::getEndPoint(
    protocol, hostname, simulation_interface::ports::update_frame));
  spawn_vehicle_entity_sock_.bind(simulation_interface::getEndPoint(
    protocol, hostname, simulation_interface::ports::spawn_vehicle_entity));
  spawn_pedestrian_entity_sock_.bind(simulation_interface::getEndPoint(
    protocol, hostname, simulation_interface::ports::spawn_pedestrian_entity));
  spawn_misc_object_entity_sock_.bind(simulation_interface::getEndPoint(
    protocol, hostname, simulation_interface::ports::spawn_misc_object_entity));
  despawn_entity_sock_.bind(simulation_interface::getEndPoint(
    protocol, hostname, simulation_interface::ports::despawn_entity));
  update_sensor_frame_sock_.bind(simulation_interface::getEndPoint(
    protocol, hostname, simulation_interface::ports::update_sensor_frame));
  attach_lidar_sensor_sock_.bind(simulation_interface::getEndPoint(
    protocol, hostname, simulation_interface::ports::attach_lidar_sensor));
  attach_detection_sensor_sock_.bind(simulation_interface::getEndPoint(
    protocol, hostname, simulation_interface::ports::attach_detection_sensor));
  update_traffic_lights_sock_.bind(simulation_interface::getEndPoint(
    protocol, hostname, simulation_interface::ports::update_traffic_lights));
  poller_.add(initialize_sock_);
  poller_.add(update_frame_sock_);
  poller_.add(update_sensor_frame_sock_);
  poller_.add(spawn_vehicle_entity_sock_);
  poller_.add(spawn_pedestrian_entity_sock_);
  poller_.add(spawn_misc_object_entity_sock_);
  poller_.add(despawn_entity_sock_);
  poller_.add(update_entity_status_sock_);
  poller_.add(attach_lidar_sensor_sock_);
  poller_.add(attach_detection_sensor_sock_);
  poller_.add(update_traffic_lights_sock_);
  thread_ = std::thread(&MultiServer::start_poll, this);
}

MultiServer::~MultiServer() { thread_.join(); }

void MultiServer::poll()
{
  constexpr long timeout_ms = 1L;
  poller_.poll(timeout_ms);
  if (poller_.has_input(initialize_sock_)) {
    zmqpp::message request;
    initialize_sock_.receive(request);
    simulation_api_schema::InitializeResponse response;
    initialize_func_(toProto<simulation_api_schema::InitializeRequest>(request), response);
    auto msg = toZMQ(response);
    initialize_sock_.send(msg);
  }
  if (poller_.has_input(update_frame_sock_)) {
    zmqpp::message request;
    update_frame_sock_.receive(request);
    simulation_api_schema::UpdateFrameResponse response;
    update_frame_func_(toProto<simulation_api_schema::UpdateFrameRequest>(request), response);
    auto msg = toZMQ(response);
    update_frame_sock_.send(msg);
  }
  if (poller_.has_input(update_sensor_frame_sock_)) {
    zmqpp::message request;
    update_sensor_frame_sock_.receive(request);
    simulation_api_schema::UpdateSensorFrameResponse response;
    update_sensor_frame_func_(
      toProto<simulation_api_schema::UpdateSensorFrameRequest>(request), response);
    auto msg = toZMQ(response);
    update_sensor_frame_sock_.send(msg);
  }
  if (poller_.has_input(spawn_vehicle_entity_sock_)) {
    zmqpp::message request;
    spawn_vehicle_entity_sock_.receive(request);
    simulation_api_schema::SpawnVehicleEntityResponse response;
    spawn_vehicle_entity_func_(
      toProto<simulation_api_schema::SpawnVehicleEntityRequest>(request), response);
    auto msg = toZMQ(response);
    spawn_vehicle_entity_sock_.send(msg);
  }
  if (poller_.has_input(spawn_pedestrian_entity_sock_)) {
    zmqpp::message request;
    spawn_pedestrian_entity_sock_.receive(request);
    simulation_api_schema::SpawnPedestrianEntityResponse response;
    spawn_pedestrian_entity_func_(
      toProto<simulation_api_schema::SpawnPedestrianEntityRequest>(request), response);
    auto msg = toZMQ(response);
    spawn_pedestrian_entity_sock_.send(msg);
  }
  if (poller_.has_input(spawn_misc_object_entity_sock_)) {
    zmqpp::message request;
    spawn_misc_object_entity_sock_.receive(request);
    simulation_api_schema::SpawnMiscObjectEntityResponse response;
    spawn_misc_object_entity_func_(
      toProto<simulation_api_schema::SpawnMiscObjectEntityRequest>(request), response);
    auto msg = toZMQ(response);
    spawn_misc_object_entity_sock_.send(msg);
  }
  if (poller_.has_input(despawn_entity_sock_)) {
    zmqpp::message request;
    despawn_entity_sock_.receive(request);
    simulation_api_schema::DespawnEntityResponse response;
    despawn_entity_func_(toProto<simulation_api_schema::DespawnEntityRequest>(request), response);
    auto msg = toZMQ(response);
    despawn_entity_sock_.send(msg);
  }
  if (poller_.has_input(update_entity_status_sock_)) {
    zmqpp::message request;
    update_entity_status_sock_.receive(request);
    simulation_api_schema::UpdateEntityStatusResponse response;
    update_entity_status_func_(
      toProto<simulation_api_schema::UpdateEntityStatusRequest>(request), response);
    auto msg = toZMQ(response);
    update_entity_status_sock_.send(msg);
  }
  if (poller_.has_input(attach_lidar_sensor_sock_)) {
    zmqpp::message request;
    attach_lidar_sensor_sock_.receive(request);
    simulation_api_schema::AttachLidarSensorResponse response;
    attach_lidar_sensor_func_(
      toProto<simulation_api_schema::AttachLidarSensorRequest>(request), response);
    auto msg = toZMQ(response);
    attach_lidar_sensor_sock_.send(msg);
  }
  if (poller_.has_input(attach_detection_sensor_sock_)) {
    zmqpp::message request;
    attach_detection_sensor_sock_.receive(request);
    simulation_api_schema::AttachDetectionSensorResponse response;
    attach_detection_sensor_func_(
      toProto<simulation_api_schema::AttachDetectionSensorRequest>(request), response);
    auto msg = toZMQ(response);
    attach_detection_sensor_sock_.send(msg);
  }
  if (poller_.has_input(update_traffic_lights_sock_)) {
    zmqpp::message request;
    update_traffic_lights_sock_.receive(request);
    simulation_api_schema::UpdateTrafficLightsResponse response;
    update_traffic_lights_func_(
      toProto<simulation_api_schema::UpdateTrafficLightsRequest>(request), response);
    auto msg = toZMQ(response);
    update_traffic_lights_sock_.send(msg);
  }
}
void MultiServer::start_poll()
{
  while (rclcpp::ok()) {
    poll();
  }
}
}  // namespace zeromq
