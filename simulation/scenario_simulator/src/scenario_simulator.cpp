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

#include <xmlrpc_interface/conversions.hpp>

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
: Node("scenario_simulator", options), sensor_sim_(get_clock())
{
  declare_parameter("port", 8080);
  get_parameter("port", port_);

  addMethod(
    xmlrpc_interface::method::initialize,
    std::bind(
      &ScenarioSimulator::initialize,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  addMethod(
    xmlrpc_interface::method::update_frame,
    std::bind(
      &ScenarioSimulator::updateFrame,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  addMethod(
    xmlrpc_interface::method::spawn_vehicle_entity,
    std::bind(
      &ScenarioSimulator::spawnVehicleEntity,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  addMethod(
    xmlrpc_interface::method::spawn_pedestrian_entity,
    std::bind(
      &ScenarioSimulator::spawnPedestrianEntity,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  addMethod(
    xmlrpc_interface::method::despawn_entity,
    std::bind(
      &ScenarioSimulator::despawnEntity,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  addMethod(
    xmlrpc_interface::method::update_entity_status,
    std::bind(
      &ScenarioSimulator::updateEntityStatus,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  addMethod(
    xmlrpc_interface::method::attach_lidar_sensor,
    std::bind(
      &ScenarioSimulator::attachLidarSensor,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  addMethod(
    xmlrpc_interface::method::attach_detection_sensor,
    std::bind(
      &ScenarioSimulator::attachDetectionSensor,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  addMethod(
    xmlrpc_interface::method::update_sensor_frame,
    std::bind(
      &ScenarioSimulator::updateSensorFrame,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  server_.bindAndListen(port_);
  server_.enableIntrospection(true);
  xmlrpc_thread_ = std::thread(&ScenarioSimulator::runXmlRpc, this);
}

ScenarioSimulator::~ScenarioSimulator()
{
  xmlrpc_thread_.join();
}

void ScenarioSimulator::runXmlRpc()
{
  while (rclcpp::ok()) {
    server_.work(1);
  }
}

void ScenarioSimulator::initialize(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  initialized_ = true;
  const auto req =
    xmlrpc_interface::deserializeFromBinValue<simulation_api_schema::InitializeRequest>(param);
  realtime_factor_ = req.realtime_factor();
  step_time_ = req.step_time();
  simulation_api_schema::InitializeResponse res;
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("succeed to initialize simulation");
  ego_vehicles_ = {};
  vehicles_ = {};
  pedestrians_ = {};
  result = XmlRpc::XmlRpcValue();
  result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
}

void ScenarioSimulator::updateFrame(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  if (!initialized_) {
    simulation_api_schema::UpdateFrameResponse res;
    res.mutable_result()->set_description("simulator have not initialized yet.");
    res.mutable_result()->set_success(false);
    result = XmlRpc::XmlRpcValue();
    result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
    return;
  }
  const auto req =
    xmlrpc_interface::deserializeFromBinValue<simulation_api_schema::UpdateFrameRequest>(param);
  simulation_api_schema::UpdateFrameResponse res;
  current_time_ = req.current_time();
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("succeed to update frame");
  result = XmlRpc::XmlRpcValue();
  result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
}

void ScenarioSimulator::updateEntityStatus(
  XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  const auto req =
    xmlrpc_interface::deserializeFromBinValue<simulation_api_schema::UpdateEntityStatusRequest>(
    param);
  entity_status_ = {};
  simulation_api_schema::UpdateEntityStatusResponse res;
  for (auto status : req.status()) {
    auto status_ptr = res.mutable_status()->Add();
    status_ptr->set_type(status.type());
    status_ptr->set_time(status.time());
    status_ptr->set_name(status.name());
    *status_ptr->mutable_bounding_box() = status.bounding_box();
    *status_ptr->mutable_action_status() = status.action_status();
    *status_ptr->mutable_pose() = status.pose();
    *status_ptr->mutable_lanelet_pose() = status.lanelet_pose();
    status_ptr->set_lanelet_pose_valid(status.lanelet_pose_valid());
    entity_status_.emplace_back(status);
  }
  result = XmlRpc::XmlRpcValue();
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("");
  result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
}

void ScenarioSimulator::spawnVehicleEntity(
  XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  const auto req =
    xmlrpc_interface::deserializeFromBinValue<simulation_api_schema::SpawnVehicleEntityRequest>(
    param);
  if (ego_vehicles_.size() != 0 && req.is_ego()) {
    throw SimulationRuntimeError("multi ego does not support");
  }
  if (req.is_ego()) {
    ego_vehicles_.emplace_back(req.parameters());
  } else {
    vehicles_.emplace_back(req.parameters());
  }
  simulation_api_schema::SpawnVehicleEntityResponse res;
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("");
  result = XmlRpc::XmlRpcValue();
  result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
}

void ScenarioSimulator::spawnPedestrianEntity(
  XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  const auto req =
    xmlrpc_interface::deserializeFromBinValue<simulation_api_schema::SpawnPedestrianEntityRequest>(
    param);
  pedestrians_.emplace_back(req.parameters());
  simulation_api_schema::SpawnPedestrianEntityResponse res;
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("");
  result = XmlRpc::XmlRpcValue();
  result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
}

void ScenarioSimulator::despawnEntity(XmlRpc::XmlRpcValue & param, XmlRpc::XmlRpcValue & result)
{
  const auto req =
    xmlrpc_interface::deserializeFromBinValue<simulation_api_schema::DespawnEntityRequest>(param);
  bool found = false;
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
  simulation_api_schema::DespawnEntityResponse res;
  if (found) {
    res.mutable_result()->set_success(true);
  } else {
    res.mutable_result()->set_success(false);
  }
  result = XmlRpc::XmlRpcValue();
  result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
}

void ScenarioSimulator::attachDetectionSensor(
  XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  const auto req =
    xmlrpc_interface::deserializeFromBinValue<
    simulation_api_schema::AttachDetectionSensorRequest>(param);
  const auto pub = this->create_publisher<
    autoware_perception_msgs::msg::DynamicObjectArray>(
    req.configuration().topic_name(), 1);
  sensor_sim_.attachDetectionSensor(req.configuration(), pub);
  simulation_api_schema::AttachDetectionSensorResponse res;
  res.mutable_result()->set_success(true);
  result = XmlRpc::XmlRpcValue();
  result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
}

void ScenarioSimulator::attachLidarSensor(
  XmlRpc::XmlRpcValue & param,
  XmlRpc::XmlRpcValue & result)
{
  const auto req =
    xmlrpc_interface::deserializeFromBinValue<
    simulation_api_schema::AttachLidarSensorRequest>(param);
  const auto pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    req.configuration().topic_name(), 1);
  sensor_sim_.attachLidarSensor(req.configuration(), pub);
  simulation_api_schema::AttachLidarSensorResponse res;
  res.mutable_result()->set_success(true);
  result = XmlRpc::XmlRpcValue();
  result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
}

void ScenarioSimulator::updateSensorFrame(XmlRpc::XmlRpcValue &, XmlRpc::XmlRpcValue & result)
{
  sensor_sim_.updateSensorFrame(current_time_, entity_status_);
  simulation_api_schema::UpdateSensorFrameResponse res;
  res.mutable_result()->set_success(true);
  result = XmlRpc::XmlRpcValue();
  result[xmlrpc_interface::key::response] = xmlrpc_interface::serializeToBinValue(res);
}

void ScenarioSimulator::addMethod(
  std::string name, std::function<void(XmlRpc::XmlRpcValue &,
  XmlRpc::XmlRpcValue &)> func)
{
  auto method_ptr = std::make_shared<scenario_simulator::XmlRpcMethod>(name, &server_);
  method_ptr->setFunction(func);
  methods_[name] = method_ptr;
}
}  // namespace scenario_simulator
