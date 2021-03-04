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
#include <simulation_interface/zmq_multi_server.hpp>
#include <simulation_interface/zmq_server.hpp>
#include <simulation_interface/zmq_client.hpp>

#include <rclcpp/rclcpp.hpp>

#include <simulation_api_schema.pb.h>
#include <string>
#include <chrono>
#include <memory>

void initialize_callback(
  const simulation_api_schema::InitializeRequest & req,
  simulation_api_schema::InitializeResponse & res) {}

void update_frame_callback(
  const simulation_api_schema::UpdateFrameRequest & req,
  simulation_api_schema::UpdateFrameResponse & res) {}

void update_sensor_frame_callback(
  const simulation_api_schema::UpdateSensorFrameRequest & req,
  simulation_api_schema::UpdateSensorFrameResponse & res) {}

void spawn_vehicle_entity_callback(
  const simulation_api_schema::SpawnVehicleEntityRequest & req,
  simulation_api_schema::SpawnVehicleEntityResponse & res) {}

void spawn_pedestrian_entity_callback(
  const simulation_api_schema::SpawnPedestrianEntityRequest & req,
  simulation_api_schema::SpawnPedestrianEntityResponse & res) {}

void update_entity_status_callback(
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
      simulation_interface::HostName::ANY,
      initialize_callback,
      update_frame_callback,
      update_sensor_frame_callback,
      spawn_vehicle_entity_callback,
      spawn_pedestrian_entity_callback,
      update_entity_status_callback),
    client_(simulation_interface::TransportProtocol::TCP,
      simulation_interface::HostName::LOCLHOST,
      simulation_interface::ports::update_entity_status),
    init_client_(simulation_interface::TransportProtocol::TCP,
      simulation_interface::HostName::LOCLHOST,
      simulation_interface::ports::initialize)
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
    client_.call(request, response);
    auto init_res = simulation_api_schema::InitializeResponse();
    init_client_.call(simulation_api_schema::InitializeRequest(), init_res);
  }

private:
  rclcpp::TimerBase::SharedPtr update_timer_;
  zeromq::MultiServer server_;
  zeromq::Client<
    simulation_api_schema::UpdateEntityStatusRequest,
    simulation_api_schema::UpdateEntityStatusResponse> client_;
  zeromq::Client<
    simulation_api_schema::InitializeRequest,
    simulation_api_schema::InitializeResponse> init_client_;
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
