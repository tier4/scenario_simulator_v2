// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef  SCENARIO_SIMULATOR__SCENARIO_SIMULATOR_IMPL_HPP_
#define  SCENARIO_SIMULATOR__SCENARIO_SIMULATOR_IMPL_HPP_

#include <simulation_api/entity/entity_manager.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <xmlrpcpp/XmlRpc.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <simulation_api_schema.pb.h>

#include <map>
#include <string>
#include <vector>

namespace scenario_simulator
{
class ScenarioSimulatorImpl
{
public:
  ScenarioSimulatorImpl();
  void initialize(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void updateFrame(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void spawnVehicleEntity(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void spawnPedestrianEntity(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void despawnEntity(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void updateEntityStatus(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);

private:
  std::vector<openscenario_msgs::VehicleParameters> vehicles_;
  std::vector<openscenario_msgs::PedestrianParameters> pedestrians_;
  double realtime_factor_;
  double step_time_;
  double current_time_;
  bool initialized_;
  std::vector<openscenario_msgs::EntityStatus> entity_status_;
};
}  // namespace scenario_simulator

#endif   // SCENARIO_SIMULATOR__SCENARIO_SIMULATOR_IMPL_HPP_
