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

#include <osi_interface/osi_zmq_client.hpp>
#include <stdexcept>

namespace osi_interface
{
OsiZmqClient::OsiZmqClient(const std::string & hostname, int port)
: context_(1), socket_(context_, zmq::socket_type::req)
{
  const auto endpoint = "tcp://" + hostname + ":" + std::to_string(port);
  socket_.connect(endpoint);
}

OsiZmqClient::~OsiZmqClient() { close(); }

auto OsiZmqClient::sendGroundTruth(const osi3::GroundTruth & gt) -> osi3::TrafficUpdate
{
  if (!is_running_) {
    return {};
  }

  // Serialize GroundTruth
  std::string serialized;
  gt.SerializeToString(&serialized);
  zmq::message_t request(serialized.data(), serialized.size());

  // Send
  socket_.send(request, zmq::send_flags::none);

  // Receive TrafficUpdate
  zmq::message_t reply;
  socket_.recv(reply, zmq::recv_flags::none);

  osi3::TrafficUpdate tu;
  tu.ParseFromArray(reply.data(), static_cast<int>(reply.size()));
  return tu;
}

auto OsiZmqClient::close() -> void
{
  if (is_running_) {
    is_running_ = false;
    socket_.close();
  }
}

}  // namespace osi_interface
