#include <scenario_simulator/scenario_simulator_impl.hpp>
#include <scenario_simulator/exception.hpp>

#include <quaternion_operation/quaternion_operation.h>

#include <simulation_api/entity/vehicle_parameter.hpp>
#include <pugixml.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>

namespace scenario_simulator
{
ScenarioSimulatorImpl::ScenarioSimulatorImpl()
{
  initialized_ = false;
  current_time_ = 0.0;
}

void ScenarioSimulatorImpl::initialize(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  if (initialized_) {
    ScenarioSimulatorImpl other {};
    std::swap(*this, other);
  }
  initialized_ = true;
  realtime_factor_ = param["sim/realtime_factor"];
  step_time_ = param["sim/step_time"];
  result["sim/initialized"] = initialized_;
  result["message"] = "succeed to initialize simulation";
}

void ScenarioSimulatorImpl::updateFrame(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  if (!initialized_) {
    result["message"] = "simulator have not initialized yet.";
    result["sim/current_time"] = current_time_;
    result["sim/update_frame"] = false;
    return;
  }
  double current_time_in_runner = param["runner/current_time"];
  if (current_time_in_runner != current_time_) {
    result["sim/current_time"] = current_time_;
    result["sim/update_frame"] = false;
    result["message"] = "timestamp of the simulator and runner does not match.";
    return;
  }
  current_time_ = current_time_ + step_time_;
  result["sim/update_frame"] = true;
  result["sim/current_time"] = current_time_;
  result["message"] = "succeed to update frame";
}

void ScenarioSimulatorImpl::setEntityStatus(
  XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  result["success"] = true;
}

void ScenarioSimulatorImpl::getEntityStatus(
  XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  result["success"] = true;
}

void ScenarioSimulatorImpl::spawnEntity(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  result["success"] = true;
}

void ScenarioSimulatorImpl::despawnEntity(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  result["success"] = true;
}
}
