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

#include <simulation_interface/zmq_server.hpp>
#include <simulation_interface/zmq_client.hpp>

#include <rclcpp/rclcpp.hpp>

#include <simulation_api_schema.pb.h>
#include <string>
#include <chrono>

void callback(
  const simulation_api_schema::InitializeRequest & req,
  simulation_api_schema::InitializeResponse & res)
{
  std::cout << __LINE__ << "," << __FILE__ << std::endl;
  req.PrintDebugString();
  std::cout << __LINE__ << "," << __FILE__ << std::endl;
}

class ExampleNode : public rclcpp::Node
{
public:
  explicit ExampleNode(const rclcpp::NodeOptions & option)
  : Node("example", option),
    server_("tcp://*:5555", callback),
    client_("tcp://localhost:5555")
  {
    using namespace std::chrono_literals;
    update_timer_ = this->create_wall_timer(50ms, std::bind(&ExampleNode::sendRequest, this));
  }
  void sendRequest()
  {
    simulation_api_schema::InitializeRequest request;
    simulation_api_schema::InitializeResponse response;
    client_.call(request, response);
    std::cout << __LINE__ << "," << __FILE__ << std::endl;
    request.PrintDebugString();
    std::cout << __LINE__ << "," << __FILE__ << std::endl;
  }

private:
  rclcpp::TimerBase::SharedPtr update_timer_;
  zeromq::Server<
    simulation_api_schema::InitializeRequest,
    simulation_api_schema::InitializeResponse> server_;
  zeromq::Client<
    simulation_api_schema::InitializeRequest,
    simulation_api_schema::InitializeResponse> client_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<ExampleNode>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
