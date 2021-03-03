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

#ifndef SIMULATION_INTERFACE__ZMQ_SERVER_HPP_
#define SIMULATION_INTERFACE__ZMQ_SERVER_HPP_

#include <simulation_interface/constants.hpp>
#include <zmqpp/zmqpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <functional>

namespace zeromq
{
template<typename ReqType, typename ResType>
class Server
{
public:
  explicit Server(
    const simulation_interface::TransportProtocol & protocol,
    const simulation_interface::HostName & hostname,
    const unsigned int & port,
    std::function<void(const ReqType &, ResType &)> func)
  : Server(
      simulation_interface::enumToString(protocol) +
      "://" + simulation_interface::enumToString(hostname) +
      ":" + std::to_string(port), func) {}
  explicit Server(
    const simulation_interface::TransportProtocol & protocol,
    const std::string & ip_address, const unsigned int & port,
    std::function<void(const ReqType &, ResType &)> func)
  : Server(
      simulation_interface::enumToString(protocol) +
      "://" + ip_address + ":" + std::to_string(port), func) {}
  explicit Server(
    const std::string & ip_address,
    const unsigned int & port,
    std::function<void(const ReqType &, ResType &)> func)
  : Server("tcp://" + ip_address + ":" + std::to_string(port), func) {}
  explicit Server(
    const unsigned int & port,
    std::function<void(const ReqType &, ResType &)> func)
  : Server("tcp://*:" + std::to_string(port), func) {}
  explicit Server(
    const std::string & endpoint,
    std::function<void(const ReqType &, ResType &)> func)
  : endpoint_(endpoint),
    context_(zmqpp::context()),
    type_(zmqpp::socket_type::reply),
    socket_(context_, type_),
    func_(func)
  {
    socket_.bind(endpoint_);
    poller_.add(socket_);
    thread_ = std::thread(&Server::start_poll, this);
  }

private:
  void poll() {
    poller_.poll(0.01);
    if (poller_.has_input(socket_)) {
      zmqpp::message request;
      socket_.receive(request);
      std::string serialized_str = request.get(0);
      ReqType req_proto;
      req_proto.ParseFromString(serialized_str);
      ResType res_proto;
      func_(req_proto, res_proto);
      std::string res_serialized_str;
      if (!res_proto.SerializeToString(&res_serialized_str)) {
        std::cout << __FILE__ << "," << __LINE__ << std::endl;
        throw std::runtime_error("failed to serialize from proto");
      }
      zmqpp::message response;
      response << res_serialized_str;
      socket_.send(response);
    }
  }
  void start_poll()
  {
    while (rclcpp::ok()) {
      poll();
    }
  }
  const std::string endpoint_;
  const zmqpp::context context_;
  const zmqpp::socket_type type_;
  zmqpp::socket socket_;
  zmqpp::poller poller_;
  std::thread thread_;
  std::function<void(const ReqType &, ResType &)> func_;
};
}  // namespace zeromq

#endif  // SIMULATION_INTERFACE__ZMQ_SERVER_HPP_
