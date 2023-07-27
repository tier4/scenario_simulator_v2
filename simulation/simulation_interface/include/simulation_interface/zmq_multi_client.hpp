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

#ifndef SIMULATION_INTERFACE__ZMQ_MULTI_CLIENT_HPP_
#define SIMULATION_INTERFACE__ZMQ_MULTI_CLIENT_HPP_

#include <simulation_api_schema.pb.h>

#include <functional>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <simulation_interface/constants.hpp>
#include <string>
#include <thread>
#include <zmqpp/zmqpp.hpp>

namespace zeromq
{
class MultiClient
{
public:
  explicit MultiClient(
    const simulation_interface::TransportProtocol & protocol, const std::string & hostname,
    const unsigned int socket_port);
  ~MultiClient();
  void closeConnection();
  void call(
    const simulation_api_schema::SimulationRequest & req,
    simulation_api_schema::SimulationResponse & res);
  void call(
    const simulation_api_schema::InitializeRequest & req,
    simulation_api_schema::InitializeResponse & res);
  void call(
    const simulation_api_schema::UpdateFrameRequest & req,
    simulation_api_schema::UpdateFrameResponse & res);
  void call(
    const simulation_api_schema::SpawnVehicleEntityRequest & req,
    simulation_api_schema::SpawnVehicleEntityResponse & res);
  void call(
    const simulation_api_schema::SpawnPedestrianEntityRequest & req,
    simulation_api_schema::SpawnPedestrianEntityResponse & res);
  void call(
    const simulation_api_schema::SpawnMiscObjectEntityRequest & req,
    simulation_api_schema::SpawnMiscObjectEntityResponse & res);
  void call(
    const simulation_api_schema::DespawnEntityRequest & req,
    simulation_api_schema::DespawnEntityResponse & res);
  void call(
    const simulation_api_schema::UpdateEntityStatusRequest & req,
    simulation_api_schema::UpdateEntityStatusResponse & res);
  void call(
    const simulation_api_schema::AttachLidarSensorRequest & req,
    simulation_api_schema::AttachLidarSensorResponse & res);
  void call(
    const simulation_api_schema::AttachDetectionSensorRequest & req,
    simulation_api_schema::AttachDetectionSensorResponse & res);
  void call(
    const simulation_api_schema::AttachOccupancyGridSensorRequest & req,
    simulation_api_schema::AttachOccupancyGridSensorResponse & res);
  void call(
    const simulation_api_schema::UpdateTrafficLightsRequest & req,
    simulation_api_schema::UpdateTrafficLightsResponse & res);

  const simulation_interface::TransportProtocol protocol;
  const std::string hostname;

private:
  zmqpp::context context_;
  const zmqpp::socket_type type_;
  zmqpp::socket socket_;

  bool is_running = true;
};
}  // namespace zeromq

#endif  // SIMULATION_INTERFACE__ZMQ_MULTI_CLIENT_HPP_
