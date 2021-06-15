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

#ifndef SIMULATION_INTERFACE__ZMQ_CLIENT_HPP_
#define SIMULATION_INTERFACE__ZMQ_CLIENT_HPP_

#include <functional>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <simulation_interface/constants.hpp>
#include <string>
#include <thread>
#include <zmqpp/zmqpp.hpp>

namespace zeromq
{
template <typename ReqType, typename ResType>
class Client
{
public:
  explicit Client(
    const simulation_interface::TransportProtocol & protocol,
    const simulation_interface::HostName & hostname, const unsigned int & port)
  : Client(
      simulation_interface::enumToString(protocol) + "://" +
      simulation_interface::enumToString(hostname) + ":" + std::to_string(port))
  {
  }
  explicit Client(
    const simulation_interface::TransportProtocol & protocol, const std::string & ip_address,
    const unsigned int & port)
  : Client(
      simulation_interface::enumToString(protocol) + "://" + ip_address + ":" +
      std::to_string(port))
  {
  }
  explicit Client(const std::string & ip_address, const unsigned int & port)
  : Client("tcp://" + ip_address + ":" + std::to_string(port))
  {
  }
  explicit Client(const unsigned int & port) : Client("tcp://localhost:" + std::to_string(port)) {}
  explicit Client(const std::string & endpoint)
  : endpoint_(endpoint),
    context_(zmqpp::context()),
    type_(zmqpp::socket_type::request),
    socket_(context_, type_)
  {
    socket_.connect(endpoint_);
  }
  void call(const ReqType & req, ResType & res)
  {
    /*
    std::chrono::system_clock::time_point start, end;
    start = std::chrono::system_clock::now();
    */
    std::string request_string;
    req.SerializeToString(&request_string);
    zmqpp::message message;
    message << request_string;
    socket_.send(message);
    zmqpp::message buffer;
    socket_.receive(buffer);
    std::string recieved_string = buffer.get(0);
    res.ParseFromString(recieved_string);
    /*
    end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::cout << "endpoint : " << endpoint_ << std::endl;
    std::cout << "transport delay : " << elapsed << " micro seconds" << std::endl;
    */
  }

private:
  const std::string endpoint_;
  const zmqpp::context context_;
  const zmqpp::socket_type type_;
  zmqpp::socket socket_;
};
}  // namespace zeromq

#endif  // SIMULATION_INTERFACE__ZMQ_CLIENT_HPP_
