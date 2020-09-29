#ifndef SIMULATION_CONTROLLER__XMLRPC_WRAPPER__SIMULATION_API_IMPL_HPP_
#define SIMULATION_CONTROLLER__XMLRPC_WRAPPER__SIMULATION_API_IMPL_HPP_

#include <simulation_controller/api/api_impl_base.hpp>
#include <simulation_controller/entity/entity_manager.hpp>

#include <rclcpp/rclcpp.hpp>
#include <xmlrpcpp/XmlRpcClient.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <xmlrpcpp/XmlRpcException.h>

namespace scenario_simulator
{
class SimulationAPIImpl : ApiImplBase
{
  using ApiImplBase::entity_manager_ptr_;
  using ApiImplBase::client_ptr_;

public:
  SimulationAPIImpl(
    std::shared_ptr<XmlRpc::XmlRpcClient> client_ptr,
    std::shared_ptr<simulation_controller::entity::EntityManager> entity_manager_ptr)
  : ApiImplBase(client_ptr, entity_manager_ptr) {}
  XmlRpc::XmlRpcValue initialize(
    double realtime_factor, double step_time, int times_try = 10,
    int duration_try_in_msec = 1000)
  {
    //nh_ = ros::NodeHandle("");
    //pnh_ = ros::NodeHandle("~");
    //marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("marker", 10);
    step_time_ = step_time;
    current_time_ = 0.0;
    XmlRpc::XmlRpcValue value;
    value[0][0]["methodName"] = "initialize";
    value[0][0]["params"]["sim/realtime_factor"] = realtime_factor;
    value[0][0]["params"]["sim/step_time"] = step_time;
    XmlRpc::XmlRpcValue result;
    for (int count_try = 0; count_try < times_try; count_try = count_try + 1) {
      try {
        client_ptr_->execute("system.multicall", value, result);
        if (result[0][0].hasMember("sim/initialized")) {
          return result[0][0];
        }
      } catch (...) {
        std::this_thread::sleep_for(std::chrono::milliseconds(duration_try_in_msec));
        continue;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(duration_try_in_msec));
    }
    throw ExecutionFailedError("failed to call initaialize API, xmlrpc timeout");
  }
  XmlRpc::XmlRpcValue updateFrame()
  {
    entity_manager_ptr_->update(current_time_, step_time_);
    XmlRpc::XmlRpcValue value;
    value[0][0]["methodName"] = "update_frame";
    value[0][0]["params"]["runner/current_time"] = current_time_;
    XmlRpc::XmlRpcValue result;
    try {
      client_ptr_->execute("system.multicall", value, result);
    } catch (XmlRpc::XmlRpcException e) {
      // ROS_ERROR("error code : %d, message : %s", e.getCode(), e.getMessage().c_str());
      throw XmlRpcRuntimeError(e.getMessage().c_str(), e.getCode());
    }
    if (!result[0][0].hasMember("sim/update_frame")) {
      throw ExecutionFailedError("there is no sim/update_frmae field in the result");
    }
    if (!result[0][0]["sim/update_frame"]) {
      throw ExecutionFailedError("failed to update simulation frame");
    }
    entity_manager_ptr_->broadcastEntityTransform();
    // marker_pub_.publish(entity_manager_ptr_->generateMarker());
    current_time_ = current_time_ + step_time_;
    return result[0][0];
  }
  double getCurrentTime() const {return current_time_;}

private:
  double step_time_;
  double current_time_;
  /*
  ros::Publisher marker_pub_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  */
  //std::shared_ptr<XmlRpc::XmlRpcClient> client_ptr_;
};
}  // namespace scenario_simulator

#endif  // SIMULATION_CONTROLLER__XMLRPC_WRAPPER__SIMULATION_API_IMPL_HPP_
