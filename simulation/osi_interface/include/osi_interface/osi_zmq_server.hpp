// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef OSI_INTERFACE__OSI_ZMQ_SERVER_HPP_
#define OSI_INTERFACE__OSI_ZMQ_SERVER_HPP_

#include <osi3/osi_groundtruth.pb.h>
#include <osi3/osi_trafficupdate.pb.h>

#include <atomic>
#include <functional>
#include <thread>
#include <zmq.hpp>

namespace osi_interface
{
// ZMQ REP server that receives pure osi3::GroundTruth and sends pure osi3::TrafficUpdate.
// The handler callback is responsible for processing the GroundTruth and returning a TrafficUpdate.
class OsiZmqServer
{
public:
  using Handler = std::function<osi3::TrafficUpdate(const osi3::GroundTruth &)>;

  OsiZmqServer(int port, Handler handler);
  ~OsiZmqServer();

  auto start() -> void;
  auto stop() -> void;

private:
  auto pollLoop() -> void;
  auto pollOnce() -> void;

  zmq::context_t context_;
  zmq::socket_t socket_;
  zmq::pollitem_t poll_item_;
  Handler handler_;
  std::thread thread_;
  std::atomic<bool> running_{false};
};

}  // namespace osi_interface

#endif  // OSI_INTERFACE__OSI_ZMQ_SERVER_HPP_
