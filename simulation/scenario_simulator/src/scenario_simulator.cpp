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

#include <scenario_simulator/scenario_simulator.hpp>
#include <scenario_simulator/exception.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <simulation_interface/conversions.hpp>

#include <quaternion_operation/quaternion_operation.h>

#include <pugixml.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>
#include <vector>
#include <string>

namespace scenario_simulator
{
ScenarioSimulator::ScenarioSimulator(const rclcpp::NodeOptions & options)
: Node("scenario_simulator", options), sensor_sim_(get_clock()),
  initialize_server_(simulation_interface::TransportProtocol::TCP,
    simulation_interface::HostName::ANY,
    simulation_interface::ports::initialize,
    std::bind(&ScenarioSimulator::initialize, this,
    std::placeholders::_1, std::placeholders::_2)),
  update_frame_server_(simulation_interface::TransportProtocol::TCP,
    simulation_interface::HostName::ANY,
    simulation_interface::ports::update_frame,
    std::bind(&ScenarioSimulator::updateFrame, this,
    std::placeholders::_1, std::placeholders::_2)),
  update_entity_status_server_(simulation_interface::TransportProtocol::TCP,
    simulation_interface::HostName::ANY,
    simulation_interface::ports::update_entity_status,
    std::bind(&ScenarioSimulator::updateEntityStatus, this,
    std::placeholders::_1, std::placeholders::_2)),
  spawn_vehicle_entity_server_(simulation_interface::TransportProtocol::TCP,
    simulation_interface::HostName::ANY,
    simulation_interface::ports::spawn_vehicle_entity,
    std::bind(&ScenarioSimulator::spawnVehicleEntity, this,
    std::placeholders::_1, std::placeholders::_2)),
  spawn_pedestrian_entity_server_(simulation_interface::TransportProtocol::TCP,
    simulation_interface::HostName::ANY,
    simulation_interface::ports::spawn_pedestrian_entity,
    std::bind(&ScenarioSimulator::spawnPedestrianEntity, this,
    std::placeholders::_1, std::placeholders::_2)),
  despawn_entity_server_(simulation_interface::TransportProtocol::TCP,
    simulation_interface::HostName::ANY,
    simulation_interface::ports::despawn_entity,
    std::bind(&ScenarioSimulator::despawnEntity, this,
    std::placeholders::_1, std::placeholders::_2)),
  attach_detection_sensor_server_(simulation_interface::TransportProtocol::TCP,
    simulation_interface::HostName::ANY,
    simulation_interface::ports::attach_detection_sensor,
    std::bind(&ScenarioSimulator::attachDetectionSensor, this,
    std::placeholders::_1, std::placeholders::_2)),
  attach_lidar_sensor_server_(simulation_interface::TransportProtocol::TCP,
    simulation_interface::HostName::ANY,
    simulation_interface::ports::attach_lidar_sensor,
    std::bind(&ScenarioSimulator::attachLidarSensor, this,
    std::placeholders::_1, std::placeholders::_2))
{
  declare_parameter("port", 8080);
  get_parameter("port", port_);
}

ScenarioSimulator::~ScenarioSimulator() {}

void ScenarioSimulator::initialize(
  const simulation_api_schema::InitializeRequest & req,
  simulation_api_schema::InitializeResponse & res)
{
  initialized_ = true;
  realtime_factor_ = req.realtime_factor();
  step_time_ = req.step_time();
  res = simulation_api_schema::InitializeResponse();
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("succeed to initialize simulation");
  ego_vehicles_ = {};
  vehicles_ = {};
  pedestrians_ = {};
}

void ScenarioSimulator::updateFrame(
  const simulation_api_schema::UpdateFrameRequest & req,
  simulation_api_schema::UpdateFrameResponse & res)
{
  res = simulation_api_schema::UpdateFrameResponse();
  if (!initialized_) {
    res.mutable_result()->set_description("simulator have not initialized yet.");
    res.mutable_result()->set_success(false);
    return;
  }
  current_time_ = req.current_time();
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("succeed to update frame");
}

void ScenarioSimulator::updateEntityStatus(
  const simulation_api_schema::UpdateEntityStatusRequest & req,
  simulation_api_schema::UpdateEntityStatusResponse & res)
{
  entity_status_ = {};
  for (const auto proto : req.status()) {
    entity_status_.emplace_back(proto);
  }
  res = simulation_api_schema::UpdateEntityStatusResponse();
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("");
}

void ScenarioSimulator::spawnVehicleEntity(
  const simulation_api_schema::SpawnVehicleEntityRequest & req,
  simulation_api_schema::SpawnVehicleEntityResponse & res)
{
  if (ego_vehicles_.size() != 0 && req.is_ego()) {
    throw SimulationRuntimeError("multi ego does not support");
  }
  if (req.is_ego()) {
    ego_vehicles_.emplace_back(req.parameters());
  } else {
    vehicles_.emplace_back(req.parameters());
  }
  res = simulation_api_schema::SpawnVehicleEntityResponse();
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("");
}

void ScenarioSimulator::spawnPedestrianEntity(
  const simulation_api_schema::SpawnPedestrianEntityRequest & req,
  simulation_api_schema::SpawnPedestrianEntityResponse & res)
{
  pedestrians_.emplace_back(req.parameters());
  res = simulation_api_schema::SpawnPedestrianEntityResponse();
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("");
}

void ScenarioSimulator::despawnEntity(
  const simulation_api_schema::DespawnEntityRequest & req,
  simulation_api_schema::DespawnEntityResponse & res)
{
  bool found = false;
  res = simulation_api_schema::DespawnEntityResponse();
  std::vector<openscenario_msgs::VehicleParameters> vehicles;
  for (const auto vehicle : vehicles_) {
    if (vehicle.name() != req.name()) {
      vehicles.emplace_back(vehicle);
    } else {
      found = true;
    }
  }
  vehicles_ = vehicles;
  std::vector<openscenario_msgs::PedestrianParameters> pedestrians;
  for (const auto pedestrian : pedestrians_) {
    if (pedestrian.name() != req.name()) {
      pedestrians.emplace_back(pedestrian);
    } else {
      found = true;
    }
  }
  pedestrians_ = pedestrians;
  if (found) {
    res.mutable_result()->set_success(true);
  } else {
    res.mutable_result()->set_success(false);
  }
}

void ScenarioSimulator::attachDetectionSensor(
  const simulation_api_schema::AttachDetectionSensorRequest & req,
  simulation_api_schema::AttachDetectionSensorResponse & res)
{
  const auto pub = this->create_publisher<
    autoware_perception_msgs::msg::DynamicObjectArray>(
    req.configuration().topic_name(), 1);
  sensor_sim_.attachDetectionSensor(req.configuration(), pub);
  res = simulation_api_schema::AttachDetectionSensorResponse();
  res.mutable_result()->set_success(true);
}

void ScenarioSimulator::attachLidarSensor(
  const simulation_api_schema::AttachLidarSensorRequest & req,
  simulation_api_schema::AttachLidarSensorResponse & res)
{
  const auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    req.configuration().topic_name(), 1);
  sensor_sim_.attachLidarSensor(req.configuration(), pub);
  res = simulation_api_schema::AttachLidarSensorResponse();
  res.mutable_result()->set_success(true);
}

/*
void ScenarioSimulator::updateSensorFrame(XmlRpc::XmlRpcValue &, XmlRpc::XmlRpcValue & result)
{
  sensor_sim_.updateSensorFrame(current_time_, entity_status_);
  simulation_api_schema::UpdateSensorFrameResponse res;
  res.mutable_result()->set_success(true);
  result = XmlRpc::XmlRpcValue();
  result[0] = simulation_interface::serializeToBinValue(res);
}
*/
}  // namespace scenario_simulator

RCLCPP_COMPONENTS_REGISTER_NODE(scenario_simulator::ScenarioSimulator)
