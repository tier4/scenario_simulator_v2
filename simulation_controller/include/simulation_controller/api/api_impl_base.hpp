#ifndef SIMULATION_CONTROLLER__XMLRPC_WRAPPER__API_IMPL_BASE_HPP_
#define SIMULATION_CONTROLLER__XMLRPC_WRAPPER__API_IMPL_BASE_HPP_

#include <simulation_controller/entity/entity_manager.hpp>
#include <xmlrpcpp/XmlRpcClient.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <memory>

#include <stdexcept>

namespace scenario_simulator
{
    class XmlRpcRuntimeError : public std::runtime_error
    {
    public:
        XmlRpcRuntimeError(const char *message, int res) : error_info_(res), runtime_error(message) {}
    private:
        int error_info_;
    };

    class ExecutionFailedError : public std::runtime_error
    {
    public:
        ExecutionFailedError(XmlRpc::XmlRpcValue value, int res=0) : error_info_(res), runtime_error(value["message"]) {};
        ExecutionFailedError(const char *message, int res=0) : error_info_(res), runtime_error(message) {};
    private:
        int error_info_;
    };
    
    class ApiImplBase
    {
    public:
        ApiImplBase(std::shared_ptr<XmlRpc::XmlRpcClient> client_ptr, std::shared_ptr<simulation_controller::entity::EntityManager> entity_manager_ptr) 
        {
            client_ptr_ = client_ptr;
            entity_manager_ptr_ = entity_manager_ptr;
        };
    protected:
        std::shared_ptr<XmlRpc::XmlRpcClient> client_ptr_;
        std::shared_ptr<simulation_controller::entity::EntityManager> entity_manager_ptr_;
    };
}

#endif  // SIMULATION_CONTROLLER__XMLRPC_WRAPPER__API_IMPL_BASE_HPP_
