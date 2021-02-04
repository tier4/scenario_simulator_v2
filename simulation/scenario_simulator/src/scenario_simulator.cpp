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

#include <scenario_simulator/scenario_simulator.hpp>
#include <xmlrpc_interface/conversions.hpp>

#include <string>
#include <vector>
#include <memory>

namespace scenario_simulator
{
ScenarioSimulator::ScenarioSimulator(const rclcpp::NodeOptions & options)
: Node("scenario_simulator", options)
{
  declare_parameter("port", 8080);
  get_parameter("port", port_);

  auto initialize_func = std::bind(
    &ScenarioSimulator::initialize, this,
    std::placeholders::_1, std::placeholders::_2);
  addMethod(xmlrpc_interface::method::initialize, initialize_func);

  auto update_frame_func = std::bind(
    &ScenarioSimulator::updateFrame, this,
    std::placeholders::_1, std::placeholders::_2);
  addMethod(xmlrpc_interface::method::update_frame, update_frame_func);

  auto spawn_vehicle_entity_func = std::bind(
    &ScenarioSimulator::spawnVehicleEntity, this,
    std::placeholders::_1, std::placeholders::_2);
  addMethod(xmlrpc_interface::method::spawn_vehicle_entity, spawn_vehicle_entity_func);

  auto spawn_pedestrian_entity_func = std::bind(
    &ScenarioSimulator::spawnPedestrianEntity, this,
    std::placeholders::_1, std::placeholders::_2);
  addMethod(xmlrpc_interface::method::spawn_pedestrian_entity, spawn_pedestrian_entity_func);

  server_.bindAndListen(port_);
  server_.enableIntrospection(true);
  xmlrpc_thread_ = std::thread(&ScenarioSimulator::runXmlRpc, this);
}

ScenarioSimulator::~ScenarioSimulator()
{
  xmlrpc_thread_.join();
}

void ScenarioSimulator::runXmlRpc()
{
  while (rclcpp::ok()) {
    server_.work(1);
  }
}

void ScenarioSimulator::updateFrame(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  impl_.updateFrame(param, result);
}

void ScenarioSimulator::initialize(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  impl_.initialize(param, result);
}

void ScenarioSimulator::spawnVehicleEntity(
  XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  impl_.spawnVehicleEntity(param, result);
}

void ScenarioSimulator::spawnPedestrianEntity(
  XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  impl_.spawnPedestrianEntity(param, result);
}

void ScenarioSimulator::addMethod(
  std::string name, std::function<void(XmlRpc::XmlRpcValue &,
  XmlRpc::XmlRpcValue &)> func)
{
  auto method_ptr = std::make_shared<scenario_simulator::XmlRpcMethod>(name, &server_);
  method_ptr->setFunction(func);
  methods_[name] = method_ptr;
}
}  // namespace scenario_simulator
