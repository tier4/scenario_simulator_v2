#include <scenario_simulator/xmlrpc_method.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator/constants.hpp>

namespace scenario_simulator
{
    XmlRpcMethod::XmlRpcMethod(std::string const &name, XmlRpc::XmlRpcServer *server)
    : XmlRpc::XmlRpcServerMethod(name, server)
    {
    }

    void XmlRpcMethod::setFunction(std::function<void(XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&)> func)
    {
        func_ = func;
    }

    void XmlRpcMethod::execute(XmlRpc::XmlRpcValue &params, XmlRpc::XmlRpcValue &result)
    {
        if(func_)
        {
            auto func = func_.get();
            func(params, result);
            result["result/return_code"] = return_code::SUCCESS;
            result["result/description"] = "function " + _name + " executed";
        }
        else
        {
            XmlRpc::XmlRpcValue error_msg;
            error_msg["result/return_code"] = return_code::FAIL;
            error_msg["result/description"] = "function does not set";   
        }
    }
}  // namespace scenario_simulator
