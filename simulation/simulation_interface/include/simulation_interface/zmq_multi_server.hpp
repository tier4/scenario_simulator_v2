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
#include <tuple>
#include <zmqpp/zmqpp.hpp>

namespace zeromq
{
class MultiServer
{
public:
  template <typename... Ts>
  explicit MultiServer(
    const simulation_interface::TransportProtocol & protocol,
    const simulation_interface::HostName & hostname, const unsigned int socket_port, Ts &&... xs)
  : context_(zmqpp::context()),
    type_(zmqpp::socket_type::reply),
    socket_(context_, type_),
    functions_(std::forward<decltype(xs)>(xs)...)
  {
    socket_.bind(simulation_interface::getEndPoint(protocol, hostname, socket_port));
    poller_.add(socket_);
    thread_ = std::thread(&MultiServer::start_poll, this);
  }

  ~MultiServer();

private:
  void poll();
  void start_poll();
  std::thread thread_;
  const zmqpp::context context_;
  const zmqpp::socket_type type_;
  zmqpp::poller poller_;
  zmqpp::socket socket_;

#define DEFINE_FUNCTION_TYPE(TYPENAME)                                      \
  using TYPENAME = std::function<simulation_api_schema::TYPENAME##Response( \
    const simulation_api_schema::TYPENAME##Request &)>

  DEFINE_FUNCTION_TYPE(Initialize);
  DEFINE_FUNCTION_TYPE(UpdateFrame);
  DEFINE_FUNCTION_TYPE(SpawnVehicleEntity);
  DEFINE_FUNCTION_TYPE(SpawnPedestrianEntity);
  DEFINE_FUNCTION_TYPE(SpawnMiscObjectEntity);
  DEFINE_FUNCTION_TYPE(DespawnEntity);
  DEFINE_FUNCTION_TYPE(UpdateEntityStatus);
  DEFINE_FUNCTION_TYPE(AttachImuSensor);
  DEFINE_FUNCTION_TYPE(AttachLidarSensor);
  DEFINE_FUNCTION_TYPE(AttachDetectionSensor);
  DEFINE_FUNCTION_TYPE(AttachOccupancyGridSensor);
  DEFINE_FUNCTION_TYPE(UpdateTrafficLights);
  DEFINE_FUNCTION_TYPE(AttachPseudoTrafficLightDetector);
  DEFINE_FUNCTION_TYPE(UpdateStepTime);

#undef DEFINE_FUNCTION_TYPE

  std::tuple<
    Initialize, UpdateFrame, SpawnVehicleEntity, SpawnPedestrianEntity, SpawnMiscObjectEntity,
    DespawnEntity, UpdateEntityStatus, AttachImuSensor, AttachLidarSensor, AttachDetectionSensor,
    AttachOccupancyGridSensor, UpdateTrafficLights, AttachPseudoTrafficLightDetector,
    UpdateStepTime>
    functions_;
};
}  // namespace zeromq

#endif  // SIMULATION_INTERFACE__ZMQ_MULTI_SERVER_HPP_
