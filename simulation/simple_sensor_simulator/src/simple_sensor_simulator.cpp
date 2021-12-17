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

#include <quaternion_operation/quaternion_operation.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <simple_sensor_simulator/exception.hpp>
#include <simple_sensor_simulator/simple_sensor_simulator.hpp>
#include <simulation_interface/conversions.hpp>
#include <string>
#include <utility>
#include <vector>

namespace simple_sensor_simulator
{
ScenarioSimulator::ScenarioSimulator(const rclcpp::NodeOptions & options)
: Node("simple_sensor_simulator", options),
  sensor_sim_(),
  server_(
    simulation_interface::protocol, simulation_interface::HostName::ANY,
    std::bind(&ScenarioSimulator::initialize, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ScenarioSimulator::updateFrame, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &ScenarioSimulator::updateSensorFrame, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &ScenarioSimulator::spawnVehicleEntity, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &ScenarioSimulator::spawnPedestrianEntity, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &ScenarioSimulator::spawnMiscObjectEntity, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &ScenarioSimulator::despawnEntity, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &ScenarioSimulator::updateEntityStatus, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &ScenarioSimulator::attachLidarSensor, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &ScenarioSimulator::attachDetectionSensor, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &ScenarioSimulator::updateTrafficLights, this, std::placeholders::_1, std::placeholders::_2))
{
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
  builtin_interfaces::msg::Time t;
  simulation_interface::toMsg(req.current_ros_time(), t);
  current_ros_time_ = t;
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

void ScenarioSimulator::spawnMiscObjectEntity(
  const simulation_api_schema::SpawnMiscObjectEntityRequest & req,
  simulation_api_schema::SpawnMiscObjectEntityResponse & res)
{
  misc_objects_.emplace_back(req.parameters());
  res = simulation_api_schema::SpawnMiscObjectEntityResponse();
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("");
}

void ScenarioSimulator::despawnEntity(
  const simulation_api_schema::DespawnEntityRequest & req,
  simulation_api_schema::DespawnEntityResponse & res)
{
  bool found = false;
  res = simulation_api_schema::DespawnEntityResponse();
  std::vector<traffic_simulator_msgs::VehicleParameters> vehicles;
  for (const auto vehicle : vehicles_) {
    if (vehicle.name() != req.name()) {
      vehicles.emplace_back(vehicle);
    } else {
      found = true;
    }
  }
  vehicles_ = vehicles;
  std::vector<traffic_simulator_msgs::PedestrianParameters> pedestrians;
  for (const auto pedestrian : pedestrians_) {
    if (pedestrian.name() != req.name()) {
      pedestrians.emplace_back(pedestrian);
    } else {
      found = true;
    }
  }
  pedestrians_ = pedestrians;
  std::vector<traffic_simulator_msgs::MiscObjectParameters> misc_objects;
  for (const auto misc_object : misc_objects_) {
    if (misc_object.name() != req.name()) {
      misc_objects.emplace_back(misc_object);
    } else {
      found = true;
    }
  }
  misc_objects_ = misc_objects;
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
  sensor_sim_.attachDetectionSensor(current_time_, req.configuration(), *this);
  res = simulation_api_schema::AttachDetectionSensorResponse();
  res.mutable_result()->set_success(true);
}

void ScenarioSimulator::attachLidarSensor(
  const simulation_api_schema::AttachLidarSensorRequest & req,
  simulation_api_schema::AttachLidarSensorResponse & res)
{
  sensor_sim_.attachLidarSensor(current_time_, req.configuration(), *this);
  res = simulation_api_schema::AttachLidarSensorResponse();
  res.mutable_result()->set_success(true);
}

void ScenarioSimulator::updateSensorFrame(
  const simulation_api_schema::UpdateSensorFrameRequest & req,
  simulation_api_schema::UpdateSensorFrameResponse & res)
{
  constexpr double e = std::numeric_limits<double>::epsilon();
  if (std::abs(req.current_time() - current_time_) > e) {
    res.mutable_result()->set_success(false);
    res.mutable_result()->set_description("timestamp does not match");
  }
  builtin_interfaces::msg::Time t;
  simulation_interface::toMsg(req.current_ros_time(), t);
  current_ros_time_ = t;
  sensor_sim_.updateSensorFrame(current_time_, current_ros_time_, entity_status_);
  res = simulation_api_schema::UpdateSensorFrameResponse();
  res.mutable_result()->set_success(true);
}

void ScenarioSimulator::updateTrafficLights(
  const simulation_api_schema::UpdateTrafficLightsRequest & req,
  simulation_api_schema::UpdateTrafficLightsResponse & res)
{
  // TODO: handle traffic lights in simple simulator
  (void)req;
  res = simulation_api_schema::UpdateTrafficLightsResponse();
  res.mutable_result()->set_success(true);
}
}  // namespace simple_sensor_simulator

RCLCPP_COMPONENTS_REGISTER_NODE(simple_sensor_simulator::ScenarioSimulator)
