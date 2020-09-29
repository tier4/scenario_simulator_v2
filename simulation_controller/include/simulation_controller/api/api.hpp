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
    public:
        API(std::string address, int port);
        std::shared_ptr<SimulationAPIImpl> simulation;
        std::shared_ptr<EntityAPIImpl> entity;
    private:
        std::shared_ptr<XmlRpc::XmlRpcClient> client_ptr_;
        std::shared_ptr<simulation_controller::entity::EntityManager> entity_manager_ptr_;
    };
}  // namespace scenario_simulator

#endif  // SIMULATION_CONTROLLER__XMLRPC_WRAPPER__XMLRPC_WRAPPER_HPP_
