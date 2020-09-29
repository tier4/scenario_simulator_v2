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
    template<class NodeT>
    class API
    {
    using EntityManager = simulation_controller::entity::EntityManager<NodeT>;
    public:
        API(std::string address, int port, NodeT && node)
        {
            auto entity_manager_ptr = std::shared_ptr<EntityManager>(new EntityManager(node));
            auto client_ptr = std::shared_ptr<XmlRpc::XmlRpcClient>
                (new XmlRpc::XmlRpcClient(address.c_str(), port));
            simulation = std::shared_ptr<SimulationAPIImpl<NodeT>>
                (new SimulationAPIImpl<NodeT>(client_ptr, entity_manager_ptr));
            entity = std::shared_ptr<EntityAPIImpl<NodeT>>
                (new EntityAPIImpl<NodeT>(client_ptr, entity_manager_ptr));
        }
        std::shared_ptr<SimulationAPIImpl<NodeT>> simulation;
        std::shared_ptr<EntityAPIImpl<NodeT>> entity;
    };
}  // namespace scenario_simulator

#endif  // SIMULATION_CONTROLLER__XMLRPC_WRAPPER__XMLRPC_WRAPPER_HPP_
