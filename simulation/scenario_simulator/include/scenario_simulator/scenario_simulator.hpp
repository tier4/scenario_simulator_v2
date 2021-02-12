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

#ifndef SCENARIO_SIMULATOR__SCENARIO_SIMULATOR_HPP_
#define SCENARIO_SIMULATOR__SCENARIO_SIMULATOR_HPP_

#include <scenario_simulator/xmlrpc_method.hpp>

#include <scenario_simulator/sensor_simulation/sensor_simulation.hpp>
#include <scenario_simulator/sensor_simulation/lidar/raycaster.hpp>
#include <scenario_simulator/sensor_simulation/lidar/lidar_sensor.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <xmlrpcpp/XmlRpc.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <map>
#include <memory>
#include <thread>
#include <vector>
#include <string>

namespace scenario_simulator
{
class ScenarioSimulator : public rclcpp::Node
{
public:
  explicit ScenarioSimulator(const rclcpp::NodeOptions & options);
  ~ScenarioSimulator();

private:
  XmlRpc::XmlRpcServer server_;
  int port_;
  std::map<std::string, std::shared_ptr<scenario_simulator::XmlRpcMethod>> methods_;
  void updateFrame(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void initialize(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void spawnVehicleEntity(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void spawnPedestrianEntity(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void despawnEntity(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void updateEntityStatus(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void attachLidarSensor(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void attachDetectionSensor(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void updateSensorFrame(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result);
  void addMethod(
    std::string name, std::function<void(XmlRpc::XmlRpcValue &,
    XmlRpc::XmlRpcValue &)> func);
  void runXmlRpc();
  std::thread xmlrpc_thread_;
  std::vector<openscenario_msgs::VehicleParameters> ego_vehicles_;
  std::vector<openscenario_msgs::VehicleParameters> vehicles_;
  std::vector<openscenario_msgs::PedestrianParameters> pedestrians_;
  double realtime_factor_;
  double step_time_;
  double current_time_;
  bool initialized_;
  std::vector<openscenario_msgs::EntityStatus> entity_status_;
  SensorSimulation sensor_sim_;
};
}  // namespace scenario_simulator

#endif  // SCENARIO_SIMULATOR__SCENARIO_SIMULATOR_HPP_
