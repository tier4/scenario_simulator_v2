#ifndef  SCENARIO_SIMULATOR__SCENARIO_SIMULATOR_IMPL_HPP_
#define  SCENARIO_SIMULATOR__SCENARIO_SIMULATOR_IMPL_HPP_

#include <simulation_api/entity/entity_manager.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <xmlrpcpp/XmlRpc.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <map>
#include <string>

namespace scenario_simulator
{
class ScenarioSimulatorImpl
{
public:
  ScenarioSimulatorImpl();
  void initialize(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void updateFrame(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void getEntityStatus(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void setEntityStatus(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void spawnEntity(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void despawnEntity(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);

private:
  double realtime_factor_;
  double step_time_;
  double current_time_;
  bool initialized_;
};
}  // namespace scenario_simulator

#endif   // SCENARIO_SIMULATOR__SCENARIO_SIMULATOR_IMPL_HPP_
