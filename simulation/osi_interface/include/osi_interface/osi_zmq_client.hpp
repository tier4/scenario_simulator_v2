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

#ifndef OSI_INTERFACE__OSI_ZMQ_CLIENT_HPP_
#define OSI_INTERFACE__OSI_ZMQ_CLIENT_HPP_

#include <osi3/osi_groundtruth.pb.h>
#include <osi3/osi_trafficupdate.pb.h>

#include <string>
#include <zmq.hpp>

namespace osi_interface
{
// ZMQ REQ client that sends pure osi3::GroundTruth and receives pure osi3::TrafficUpdate.
class OsiZmqClient
{
public:
  OsiZmqClient(const std::string & hostname, int port);
  ~OsiZmqClient();

  auto sendGroundTruth(const osi3::GroundTruth & gt) -> osi3::TrafficUpdate;

  auto close() -> void;

private:
  zmq::context_t context_;
  zmq::socket_t socket_;
  bool is_running_{true};
};

}  // namespace osi_interface

#endif  // OSI_INTERFACE__OSI_ZMQ_CLIENT_HPP_
