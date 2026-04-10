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

#include <chrono>
#include <osi_interface/osi_zmq_server.hpp>

namespace osi_interface
{
OsiZmqServer::OsiZmqServer(int port, Handler handler)
: context_(1), socket_(context_, zmq::socket_type::rep), handler_(std::move(handler))
{
  const auto endpoint = "tcp://*:" + std::to_string(port);
  socket_.bind(endpoint);
  poll_item_ = zmq::pollitem_t{socket_.handle(), 0, ZMQ_POLLIN, 0};
}

OsiZmqServer::~OsiZmqServer() { stop(); }

auto OsiZmqServer::start() -> void
{
  running_ = true;
  thread_ = std::thread(&OsiZmqServer::pollLoop, this);
}

auto OsiZmqServer::stop() -> void
{
  if (running_) {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }
}

auto OsiZmqServer::pollLoop() -> void
{
  while (running_) {
    pollOnce();
  }
}

auto OsiZmqServer::pollOnce() -> void
{
  constexpr auto timeout = std::chrono::milliseconds(1);
  zmq::poll(&poll_item_, 1, timeout);

  if (poll_item_.revents & ZMQ_POLLIN) {
    // Receive GroundTruth
    zmq::message_t request;
    socket_.recv(request, zmq::recv_flags::none);

    osi3::GroundTruth gt;
    gt.ParseFromArray(request.data(), static_cast<int>(request.size()));

    // Process and get TrafficUpdate
    auto tu = handler_(gt);

    // Send TrafficUpdate
    std::string serialized;
    tu.SerializeToString(&serialized);
    zmq::message_t reply(serialized.data(), serialized.size());
    socket_.send(reply, zmq::send_flags::none);
  }
}

}  // namespace osi_interface
