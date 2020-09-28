#ifndef SIMULATION_CONTROLLER__XMLRPC_WRAPPER__SIMULATION_API_IMPL_HPP_
#define SIMULATION_CONTROLLER__XMLRPC_WRAPPER__SIMULATION_API_IMPL_HPP_

#include <simulation_controller/api/api_impl_base.hpp>
#include <simulation_controller/entity/entity_manager.hpp>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcClient.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>

namespace scenario_simulator
{
    class SimulationAPIImpl : ApiImplBase
    {
    public:
        SimulationAPIImpl(std::shared_ptr<XmlRpc::XmlRpcClient> client_ptr, std::shared_ptr<simulation_controller::entity::EntityManager> entity_manager_ptr)
        : ApiImplBase(client_ptr, entity_manager_ptr) {};
        XmlRpc::XmlRpcValue initialize(double realtime_factor, double step_time, int times_try = 10, int duration_try_in_msec = 1000);
        XmlRpc::XmlRpcValue updateFrame();
        double getCurrentTime() const {return current_time_;}
    private:
        double step_time_;
        double current_time_;
        ros::Publisher marker_pub_;
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        //std::shared_ptr<XmlRpc::XmlRpcClient> client_ptr_;
    };
}  // namespace scenario_simulator

#endif  // SIMULATION_CONTROLLER__XMLRPC_WRAPPER__SIMULATION_API_IMPL_HPP_
