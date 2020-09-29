#ifndef SIMULATION_CONTROLLER__XMLRPC_WRAPPER__XMLRPC_WRAPPER_HPP_
#define SIMULATION_CONTROLLER__XMLRPC_WRAPPER__XMLRPC_WRAPPER_HPP_

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

#endif  // SIMULATION_CONTROLLER__XMLRPC_WRAPPER__XMLRPC_WRAPPER_HPP_
