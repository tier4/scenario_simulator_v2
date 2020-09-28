#include <simulation_controller/api/simulation_api_impl.hpp>

#include <chrono>
#include <thread>

namespace scenario_simulator
{
    XmlRpc::XmlRpcValue SimulationAPIImpl::initialize(double realtime_factor, double step_time, int times_try, int duration_try_in_msec)
    {
        nh_ = ros::NodeHandle("");
        pnh_ = ros::NodeHandle("~");
        marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("marker", 10);
        step_time_ = step_time;
        current_time_ = 0.0;
        XmlRpc::XmlRpcValue value;
        value[0][0]["methodName"] = "initialize";
        value[0][0]["params"]["sim/realtime_factor"] = realtime_factor;
        value[0][0]["params"]["sim/step_time"] = step_time;
        XmlRpc::XmlRpcValue result;
        for(int count_try = 0; count_try<times_try; count_try = count_try + 1)
        {
            try
            {
                client_ptr_->execute("system.multicall", value, result);
                if(result[0][0].hasMember("sim/initialized"))
                {
                    return result[0][0];
                }
            }
            catch(...)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(duration_try_in_msec));
                continue;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(duration_try_in_msec));
        }
        throw ExecutionFailedError("failed to call initaialize API, xmlrpc timeout");
    }

    XmlRpc::XmlRpcValue SimulationAPIImpl::updateFrame()
    {
        entity_manager_ptr_->update(current_time_, step_time_);
        XmlRpc::XmlRpcValue value;
        value[0][0]["methodName"] = "update_frame";
        value[0][0]["params"]["runner/current_time"] = current_time_;
        XmlRpc::XmlRpcValue result;
        try
        {
            client_ptr_->execute("system.multicall", value, result);
        }
        catch(XmlRpc::XmlRpcException e)
        {
            ROS_ERROR("error code : %d, message : %s", e.getCode(), e.getMessage().c_str());
            throw XmlRpcRuntimeError(e.getMessage().c_str(), e.getCode());
        }
        if(!result[0][0].hasMember("sim/update_frame"))
        {
            throw ExecutionFailedError("there is no sim/update_frmae field in the result");
        }
        if(!result[0][0]["sim/update_frame"])
        {
            throw ExecutionFailedError("failed to update simulation frame");
        }
        entity_manager_ptr_->broadcastEntityTransform();
        marker_pub_.publish(entity_manager_ptr_->generateMarker());
        current_time_ = current_time_ + step_time_;
        return result[0][0];
    }
}  // namespace scenario_simulator
