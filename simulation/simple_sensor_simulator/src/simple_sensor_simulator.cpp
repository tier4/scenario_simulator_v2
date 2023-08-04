// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#include <algorithm>
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
  sensor_sim_(*this),
  server_(
    simulation_interface::protocol, simulation_interface::HostName::ANY, getSocketPort(),
    std::bind(&ScenarioSimulator::initialize, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ScenarioSimulator::updateFrame, this, std::placeholders::_1, std::placeholders::_2),
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
      &ScenarioSimulator::attachOccupancyGridSensor, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &ScenarioSimulator::updateTrafficLights, this, std::placeholders::_1, std::placeholders::_2))
{
}

geographic_msgs::msg::GeoPoint ScenarioSimulator::getOrigin()
{
  geographic_msgs::msg::GeoPoint origin;
  {
    if (!has_parameter("origin_latitude")) {
      declare_parameter("origin_latitude", 0.0);
    }
    if (!has_parameter("origin_longitude")) {
      declare_parameter("origin_longitude", 0.0);
    }
    get_parameter("origin_latitude", origin.latitude);
    get_parameter("origin_longitude", origin.longitude);
  }

  return origin;
}

ScenarioSimulator::~ScenarioSimulator() {}

int ScenarioSimulator::getSocketPort()
{
  if (!has_parameter("port")) declare_parameter("port", 5555);
  return get_parameter("port").as_int();
}

void ScenarioSimulator::initialize(
  const simulation_api_schema::InitializeRequest & req,
  simulation_api_schema::InitializeResponse & res)
{
  initialized_ = true;
  realtime_factor_ = req.realtime_factor();
  step_time_ = req.step_time();
  hdmap_utils_ = std::make_shared<hdmap_utils::HdMapUtils>(req.lanelet2_map_path(), getOrigin());
  res = simulation_api_schema::InitializeResponse();
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("succeed to initialize simulation");
  ego_vehicles_ = {};
  vehicles_ = {};
  pedestrians_ = {};
  misc_objects_ = {};
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
  std::vector<traffic_simulator_msgs::EntityStatus> entity_status;
  std::transform(
    entity_status_.begin(), entity_status_.end(), std::back_inserter(entity_status),
    [this](const auto & map_element) {
      traffic_simulator_msgs::EntityStatus status;
      *status.mutable_pose() = map_element.second.pose();
      *status.mutable_action_status() = map_element.second.action_status();
      *status.mutable_name() = map_element.second.name();
      *status.mutable_type() = map_element.second.type();
      *status.mutable_subtype() = map_element.second.subtype();
      *status.mutable_bounding_box() = getBoundingBox(status.name());
      return status;
    });
  sensor_sim_.updateSensorFrame(
    current_time_, current_ros_time_, entity_status, traffic_signals_states_);
  res.mutable_result()->set_success(true);
  res.mutable_result()->set_description("succeed to update frame");
}

void ScenarioSimulator::updateEntityStatus(
  const simulation_api_schema::UpdateEntityStatusRequest & req,
  simulation_api_schema::UpdateEntityStatusResponse & res)
{
  if (isEgo(req.status().name())) {
    if (ego_entity_simulation_) {
      ego_entity_simulation_->update(
        current_time_ + step_time_, step_time_, req.npc_logic_started());
    }
    simulation_api_schema::EntityStatus status;
    simulation_interface::toProto(ego_entity_simulation_->getStatus(), status);
    entity_status_[req.status().name()] = status;
  } else {
    entity_status_[req.status().name()] = req.status();
  }
  const simulation_api_schema::EntityStatus & updated_entity_status =
    entity_status_[req.status().name()];
  res = simulation_api_schema::UpdateEntityStatusResponse();
  res.mutable_status()->set_name(updated_entity_status.name());
  res.mutable_status()->mutable_action_status()->CopyFrom(updated_entity_status.action_status());
  res.mutable_status()->mutable_pose()->CopyFrom(updated_entity_status.pose());
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
    traffic_simulator_msgs::msg::VehicleParameters parameters;
    simulation_interface::toMsg(req.parameters(), parameters);
    ego_entity_simulation_ = std::make_shared<vehicle_simulation::EgoEntitySimulation>(
      parameters, step_time_, hdmap_utils_);
    traffic_simulator_msgs::msg::EntityStatus initial_status;
    initial_status.name = parameters.name;
    simulation_interface::toMsg(req.pose(), initial_status.pose);
    initial_status.bounding_box = parameters.bounding_box;

    ego_entity_simulation_->fillLaneletDataAndSnapZToLanelet(initial_status);
    ego_entity_simulation_->setInitialStatus(initial_status);
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
  auto remove_despawn_requested_entity_from = [&](auto & v) {
    const auto size = std::size(v);
    v.erase(
      std::remove_if(
        std::begin(v), std::end(v),
        [&](const auto & entity) { return entity.name() == req.name(); }),
      std::end(v));
    return size != std::size(v);  // true if something removed.
  };
  const auto ego_entity_was_removed = remove_despawn_requested_entity_from(ego_vehicles_);
  if (ego_entity_was_removed) {
    ego_entity_simulation_.reset();
  }
  const auto any_entity_was_removed = ego_entity_was_removed or
                                      remove_despawn_requested_entity_from(vehicles_) or
                                      remove_despawn_requested_entity_from(pedestrians_) or
                                      remove_despawn_requested_entity_from(misc_objects_);
  if (any_entity_was_removed) {
    entity_status_.erase(req.name());
  }
  res.mutable_result()->set_success(any_entity_was_removed);
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

void ScenarioSimulator::attachOccupancyGridSensor(
  const simulation_api_schema::AttachOccupancyGridSensorRequest & req,
  simulation_api_schema::AttachOccupancyGridSensorResponse & res)
{
  res = simulation_api_schema::AttachOccupancyGridSensorResponse();
  sensor_sim_.attachOccupancyGridSensor(current_time_, req.configuration(), *this);
  res.mutable_result()->set_success(true);
}

void ScenarioSimulator::updateTrafficLights(
  const simulation_api_schema::UpdateTrafficLightsRequest & req,
  simulation_api_schema::UpdateTrafficLightsResponse & res)
{
  traffic_signals_states_.clear();
  for (const auto & traffic_signal_proto : req.states()) {
    autoware_auto_perception_msgs::msg::TrafficSignal traffic_signal;
    simulation_interface::toMsg(traffic_signal_proto, traffic_signal);
    traffic_signals_states_.emplace_back(traffic_signal);
  }
  res = simulation_api_schema::UpdateTrafficLightsResponse();
  res.mutable_result()->set_success(true);
}

traffic_simulator_msgs::BoundingBox ScenarioSimulator::getBoundingBox(const std::string & name)
{
  for (const auto & ego : ego_vehicles_) {
    if (ego.name() == name) {
      return ego.bounding_box();
    }
  }
  for (const auto & vehicle : vehicles_) {
    if (vehicle.name() == name) {
      return vehicle.bounding_box();
    }
  }
  for (const auto & pedestrian : pedestrians_) {
    if (pedestrian.name() == name) {
      return pedestrian.bounding_box();
    }
  }
  for (const auto & misc_object : misc_objects_) {
    if (misc_object.name() == name) {
      return misc_object.bounding_box();
    }
  }

  THROW_SEMANTIC_ERROR("Entity : ", std::quoted(name), " does not exist");
}

bool ScenarioSimulator::isEgo(const std::string & name)
{
  if (not isEntityExists(name)) {
    THROW_SEMANTIC_ERROR("Entity : ", std::quoted(name), " does not exist");
  }
  for (const auto & ego : ego_vehicles_) {
    if (ego.name() == name) {
      return true;
    }
  }
  return false;
}

bool ScenarioSimulator::isEntityExists(const std::string & name)
{
  if (std::find_if(ego_vehicles_.begin(), ego_vehicles_.end(), [&name](const auto & ego) {
        return ego.name() == name;
      }) != ego_vehicles_.end()) {
    return true;
  }

  if (std::find_if(vehicles_.begin(), vehicles_.end(), [&name](const auto & vehicle) {
        return vehicle.name() == name;
      }) != vehicles_.end()) {
    return true;
  }

  if (std::find_if(pedestrians_.begin(), pedestrians_.end(), [&name](const auto & pedestrian) {
        return pedestrian.name() == name;
      }) != pedestrians_.end()) {
    return true;
  }

  if (std::find_if(misc_objects_.begin(), misc_objects_.end(), [&name](const auto & misc_object) {
        return misc_object.name() == name;
      }) != misc_objects_.end()) {
    return true;
  }

  return false;
}
}  // namespace simple_sensor_simulator

RCLCPP_COMPONENTS_REGISTER_NODE(simple_sensor_simulator::ScenarioSimulator)
