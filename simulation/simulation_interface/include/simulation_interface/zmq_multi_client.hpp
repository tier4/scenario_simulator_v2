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

  auto call(const simulation_api_schema::SimulationRequest &)
    -> simulation_api_schema::SimulationResponse;

  auto call(const simulation_api_schema::InitializeRequest &)
    -> simulation_api_schema::InitializeResponse;

  auto call(const simulation_api_schema::UpdateFrameRequest &)
    -> simulation_api_schema::UpdateFrameResponse;

  auto call(const simulation_api_schema::UpdateStepTimeRequest &)
    -> simulation_api_schema::UpdateStepTimeResponse;

  auto call(const simulation_api_schema::SpawnVehicleEntityRequest &)
    -> simulation_api_schema::SpawnVehicleEntityResponse;

  auto call(const simulation_api_schema::SpawnPedestrianEntityRequest &)
    -> simulation_api_schema::SpawnPedestrianEntityResponse;

  auto call(const simulation_api_schema::SpawnMiscObjectEntityRequest &)
    -> simulation_api_schema::SpawnMiscObjectEntityResponse;

  auto call(const simulation_api_schema::DespawnEntityRequest &)
    -> simulation_api_schema::DespawnEntityResponse;

  auto call(const simulation_api_schema::UpdateEntityStatusRequest &)
    -> simulation_api_schema::UpdateEntityStatusResponse;

  auto call(const simulation_api_schema::AttachImuSensorRequest &)
    -> simulation_api_schema::AttachImuSensorResponse;

  auto call(const simulation_api_schema::AttachLidarSensorRequest &)
    -> simulation_api_schema::AttachLidarSensorResponse;

  auto call(const simulation_api_schema::AttachDetectionSensorRequest &)
    -> simulation_api_schema::AttachDetectionSensorResponse;

  auto call(const simulation_api_schema::AttachOccupancyGridSensorRequest &)
    -> simulation_api_schema::AttachOccupancyGridSensorResponse;

  auto call(const simulation_api_schema::UpdateTrafficLightsRequest &)
    -> simulation_api_schema::UpdateTrafficLightsResponse;

  auto call(const simulation_api_schema::AttachPseudoTrafficLightDetectorRequest &)
    -> simulation_api_schema::AttachPseudoTrafficLightDetectorResponse;

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
