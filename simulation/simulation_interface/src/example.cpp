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

#include <simulation_interface/conversions.hpp>
#include <simulation_interface/zmq_server.hpp>
#include <simulation_interface/zmq_client.hpp>

#include <rclcpp/rclcpp.hpp>

#include <simulation_api_schema.pb.h>
#include <string>
#include <chrono>
#include <memory>

void callback(
  const simulation_api_schema::UpdateEntityStatusRequest & req,
  simulation_api_schema::UpdateEntityStatusResponse & res)
{
  std::cout << "------ Request ------" << std::endl;
  req.PrintDebugString();
  res = simulation_api_schema::UpdateEntityStatusResponse();
  res.mutable_result()->set_success(true);
  std::cout << "------ Response ------" << std::endl;
  res.PrintDebugString();
}

class ExampleNode : public rclcpp::Node
{
public:
  explicit ExampleNode(const rclcpp::NodeOptions & option)
  : Node("example", option),
    server_(simulation_interface::TransportProtocol::TCP,
      simulation_interface::HostName::ANY, 5555, callback),
    client_(simulation_interface::TransportProtocol::TCP,
      simulation_interface::HostName::LOCLHOST, 5555)
  {
    using namespace std::chrono_literals;
    update_timer_ = this->create_wall_timer(250ms, std::bind(&ExampleNode::sendRequest, this));
  }
  void sendRequest()
  {
    simulation_api_schema::UpdateEntityStatusRequest request;
    openscenario_msgs::msg::EntityStatus status;
    status.name = "test";
    status.type.type = openscenario_msgs::msg::EntityType::EGO;
    openscenario_msgs::EntityStatus proto;
    simulation_interface::toProto(status, proto);
    simulation_api_schema::UpdateEntityStatusResponse response;
    *request.add_status() = proto;
    *request.add_status() = proto;
    *request.add_status() = proto;
    *request.add_status() = proto;
    *request.add_status() = proto;
    client_.call(request, response);
  }

private:
  rclcpp::TimerBase::SharedPtr update_timer_;
  zeromq::Server<
    simulation_api_schema::UpdateEntityStatusRequest,
    simulation_api_schema::UpdateEntityStatusResponse> server_;
  zeromq::Client<
    simulation_api_schema::UpdateEntityStatusRequest,
    simulation_api_schema::UpdateEntityStatusResponse> client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<ExampleNode>(options);
  component->sendRequest();
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
