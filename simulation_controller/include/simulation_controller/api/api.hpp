// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SIMULATION_CONTROLLER__API__API_HPP_
#define SIMULATION_CONTROLLER__API__API_HPP_

#include <simulation_controller/api/simulation_api_impl.hpp>
#include <simulation_controller/api/entity_api_impl.hpp>
#include <simulation_controller/entity/entity_manager.hpp>

#include <rclcpp/rclcpp.hpp>
#include <xmlrpcpp/XmlRpcClient.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <memory>

namespace scenario_simulator
{
class API
{
  using EntityManager = simulation_controller::entity::EntityManager;

public:
  template<class NodeT>
  API(NodeT && node)
  {
    std::string address = "127.0.0.1";
    int port = 8080;
    node->declare_parameter("port", 8080);
    node->get_parameter("port", port);

    auto entity_manager_ptr = std::shared_ptr<EntityManager>(new EntityManager(node));
    auto client_ptr =
      std::shared_ptr<XmlRpc::XmlRpcClient>(new XmlRpc::XmlRpcClient(address.c_str(), port));
    simulation =
      std::shared_ptr<SimulationAPIImpl>(new SimulationAPIImpl(client_ptr, entity_manager_ptr));
    entity = std::shared_ptr<EntityAPIImpl>(new EntityAPIImpl(client_ptr, entity_manager_ptr));
  }
  std::shared_ptr<SimulationAPIImpl> simulation;
  std::shared_ptr<EntityAPIImpl> entity;
};
}  // namespace scenario_simulator

#endif  // SIMULATION_CONTROLLER__API__API_HPP_
