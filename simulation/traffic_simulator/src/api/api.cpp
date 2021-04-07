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

#include <tf2/LinearMath/Quaternion.h>

#include <limits>
#include <memory>
#include <simulation_interface/conversions.hpp>
#include <stdexcept>
#include <string>
#include <traffic_simulator/api/api.hpp>

namespace scenario_simulator
{
void API::setVerbose(bool verbose)
{
  metrics_manager_.setVerbose(verbose);
  entity_manager_ptr_->setVerbose(verbose);
}

bool API::despawn(const std::string & name)
{
  const auto result = entity_manager_ptr_->despawnEntity(name);
  if (!result) {
    return false;
  }
  if (!standalone_mode) {
    simulation_api_schema::DespawnEntityRequest req;
    simulation_api_schema::DespawnEntityResponse res;
    req.set_name(name);
    despawn_entity_client_.call(req, res);
    return res.result().success();
  }
  return true;
}

bool API::spawn(
  const bool is_ego, const std::string & catalog_xml,
  const openscenario_msgs::msg::EntityStatus & status)
{
  pugi::xml_document catalog_xml_doc;
  catalog_xml_doc.load_string(catalog_xml.c_str());
  pugi::xml_node vehicle_node = catalog_xml_doc.child("Vehicle");
  if (vehicle_node != nullptr) {
    const auto params = traffic_simulator::entity::VehicleParameters(catalog_xml_doc).toRosMsg();
    return spawn(is_ego, status.name, params);
  }
  pugi::xml_node pedestrian_node = catalog_xml_doc.child("Pedestrian");
  if (pedestrian_node != nullptr) {
    const auto params = traffic_simulator::entity::PedestrianParameters(catalog_xml_doc).toRosMsg();
    return spawn(is_ego, status.name, params);
  }
  return false;
}

bool API::spawn(
  const bool is_ego, const std::string & name,
  const openscenario_msgs::msg::VehicleParameters & params)
{
  if (is_ego) {
    if (
      !entity_manager_ptr_->entityExists(name) &&
      !entity_manager_ptr_->spawnEntity(
        traffic_simulator::entity::EgoEntity(name, lanelet2_map_osm, step_time_, params))) {
      return false;
    }
    if (standalone_mode) {
      return true;
    }
    simulation_api_schema::SpawnVehicleEntityRequest req;
    simulation_api_schema::SpawnVehicleEntityResponse res;
    req.set_is_ego(true);
    simulation_interface::toProto(params, *req.mutable_parameters());
    spawn_vehicle_entity_client_.call(req, res);
    return res.result().success();
  } else {
    traffic_simulator::entity::VehicleEntity npc(name, params);
    if (!entity_manager_ptr_->spawnEntity(npc)) {
      return false;
    }
    if (standalone_mode) {
      return true;
    }
    simulation_api_schema::SpawnVehicleEntityRequest req;
    simulation_api_schema::SpawnVehicleEntityResponse res;
    req.set_is_ego(false);
    simulation_interface::toProto(params, *req.mutable_parameters());
    spawn_vehicle_entity_client_.call(req, res);
    return res.result().success();
  }
  return false;
}

bool API::spawn(
  const bool is_ego, const std::string & name,
  const openscenario_msgs::msg::PedestrianParameters & params)
{
  if (is_ego) {
    throw traffic_simulator::SimulationRuntimeError("pedestrian should not be ego");
  }
  traffic_simulator::entity::PedestrianEntity pedestrian(name, params);
  if (!entity_manager_ptr_->spawnEntity(pedestrian)) {
    return false;
  }
  if (standalone_mode) {
    return true;
  }
  simulation_api_schema::SpawnPedestrianEntityRequest req;
  simulation_api_schema::SpawnPedestrianEntityResponse res;
  simulation_interface::toProto(params, *req.mutable_parameters());
  spawn_pedestrian_entity_client_.call(req, res);
  return res.result().success();
}

bool API::spawn(const bool is_ego, const std::string & name, const std::string & catalog_xml)
{
  pugi::xml_document catalog_xml_doc;
  catalog_xml_doc.load_string(catalog_xml.c_str());
  pugi::xml_node vehicle_node = catalog_xml_doc.child("Vehicle");
  if (vehicle_node != NULL) {
    const auto params = traffic_simulator::entity::VehicleParameters(catalog_xml_doc).toRosMsg();
    spawn(is_ego, name, params);
  }
  pugi::xml_node pedestrian_node = catalog_xml_doc.child("Pedestrian");
  if (pedestrian_node != NULL) {
    const auto params = traffic_simulator::entity::PedestrianParameters(catalog_xml_doc).toRosMsg();
    spawn(false, name, params);
  }
  return true;
}

geometry_msgs::msg::Pose API::getEntityPose(const std::string & name)
{
  auto status = getEntityStatus(name);
  return status.pose;
}

openscenario_msgs::msg::EntityStatus API::getEntityStatus(const std::string & name)
{
  auto status = entity_manager_ptr_->getEntityStatus(name);
  if (!status) {
    throw traffic_simulator::SimulationRuntimeError(
      "error occurs while getting entity stauts, target entity : " + name);
  }
  return status.get();
}

bool API::setEntityStatus(
  const std::string & name, const openscenario_msgs::msg::EntityStatus & status)
{
  return entity_manager_ptr_->setEntityStatus(name, status);
}

bool API::setEntityStatus(
  const std::string & name, const std::string & reference_entity_name,
  const geometry_msgs::msg::Point & relative_position,
  const geometry_msgs::msg::Vector3 & relative_rpy,
  const openscenario_msgs::msg::ActionStatus & action_status)
{
  geometry_msgs::msg::Pose relative_pose;
  relative_pose.position = relative_position;
  relative_pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(relative_rpy);
  return setEntityStatus(name, reference_entity_name, relative_pose, action_status);
}

bool API::setEntityStatus(
  const std::string & name, const std::string & reference_entity_name,
  const geometry_msgs::msg::Pose & relative_pose,
  const openscenario_msgs::msg::ActionStatus & action_status)
{
  const auto pose = entity_manager_ptr_->getMapPose(reference_entity_name, relative_pose);
  openscenario_msgs::msg::EntityStatus status;
  status.time = current_time_;
  status.pose = pose;
  const auto lanelet_pose = entity_manager_ptr_->toLaneletPose(pose);
  status.action_status = action_status;
  if (lanelet_pose) {
    status.lanelet_pose_valid = true;
    status.lanelet_pose = lanelet_pose.get();
  } else {
    status.lanelet_pose_valid = false;
  }
  return entity_manager_ptr_->setEntityStatus(name, status);
}

boost::optional<double> API::getTimeHeadway(const std::string & from, const std::string & to)
{
  if (
    !entity_manager_ptr_->entityStatusSetted(from) ||
    !entity_manager_ptr_->entityStatusSetted(to)) {
    return boost::none;
  }
  geometry_msgs::msg::Pose pose = getRelativePose(from, to);
  if (pose.position.x > 0) {
    return boost::none;
  }
  openscenario_msgs::msg::EntityStatus to_status = getEntityStatus(to);
  double ret = (pose.position.x * -1) / (to_status.action_status.twist.linear.x);
  if (std::isnan(ret)) {
    return std::numeric_limits<double>::infinity();
  }
  return ret;
}

bool API::reachPosition(
  const std::string & name, const geometry_msgs::msg::Pose & target_pose, const double tolerance)
{
  return entity_manager_ptr_->entityStatusSetted(name) &&
         entity_manager_ptr_->reachPosition(name, target_pose, tolerance);
}

bool API::reachPosition(
  const std::string & name, const openscenario_msgs::msg::LaneletPose & target_pose,
  const double tolerance)
{
  return entity_manager_ptr_->entityStatusSetted(name) &&
         entity_manager_ptr_->reachPosition(
           name, target_pose.lanelet_id, target_pose.s, target_pose.offset, tolerance);

  // NOTE: ^ ament_uncrustify says above indentation is so beautiful.
}

bool API::reachPosition(
  const std::string & name, const std::string & target_name, const double tolerance) const
{
  return entity_manager_ptr_->entityStatusSetted(name) &&
         entity_manager_ptr_->entityStatusSetted(target_name) &&
         entity_manager_ptr_->reachPosition(name, target_name, tolerance);
}

bool API::setEntityStatus(
  const std::string & name, const openscenario_msgs::msg::LaneletPose & lanelet_pose,
  const openscenario_msgs::msg::ActionStatus & action_status)
{
  openscenario_msgs::msg::EntityStatus status;
  status.lanelet_pose = lanelet_pose;
  status.lanelet_pose_valid = true;
  status.bounding_box = entity_manager_ptr_->getBoundingBox(name);
  status.pose = entity_manager_ptr_->toMapPose(lanelet_pose);
  status.name = name;
  status.time = getCurrentTime();
  status.action_status = action_status;
  return setEntityStatus(name, status);
}

bool API::setEntityStatus(
  const std::string & name, const geometry_msgs::msg::Pose & map_pose,
  const openscenario_msgs::msg::ActionStatus & action_status)
{
  const auto lanelet_pose = entity_manager_ptr_->toLaneletPose(map_pose);
  openscenario_msgs::msg::EntityStatus status;
  if (lanelet_pose) {
    status.lanelet_pose = lanelet_pose.get();
  } else {
    status.lanelet_pose_valid = false;
  }
  status.pose = map_pose;
  status.name = name;
  status.action_status = action_status;
  status.time = getCurrentTime();
  status.bounding_box = entity_manager_ptr_->getBoundingBox(name);
  return setEntityStatus(name, status);
}

bool API::initialize(double realtime_factor, double step_time)
{
  step_time_ = step_time;
  current_time_ = 0.0;
  if (standalone_mode) {
    return true;
  }
  simulation_api_schema::InitializeRequest req;
  req.set_step_time(step_time);
  req.set_realtime_factor(realtime_factor);
  simulation_api_schema::InitializeResponse res;
  initialize_client_.call(req, res);
  return res.result().success();
}

bool API::attachDetectionSensor(simulation_api_schema::DetectionSensorConfiguration configuration)
{
  if (standalone_mode) {
    return true;
  } else {
    simulation_api_schema::AttachDetectionSensorRequest req;
    simulation_api_schema::AttachDetectionSensorResponse res;
    *req.mutable_configuration() = configuration;
    attach_detection_sensor_client_.call(req, res);
    return res.result().success();
  }
}

bool API::attachLidarSensor(simulation_api_schema::LidarConfiguration configuration)
{
  if (standalone_mode) {
    return true;
  } else {
    simulation_api_schema::AttachLidarSensorRequest req;
    simulation_api_schema::AttachLidarSensorResponse res;
    *req.mutable_configuration() = configuration;
    attach_lidar_sensor_client_.call(req, res);
    return res.result().success();
  }
}

bool API::updateSensorFrame()
{
  if (standalone_mode) {
    return true;
  }
  simulation_api_schema::UpdateSensorFrameRequest req;
  req.set_current_time(current_time_);
  simulation_api_schema::UpdateSensorFrameResponse res;
  update_sensor_frame_client_.call(req, res);
  return res.result().success();
}

bool API::updateEntityStatusInSim()
{
  simulation_api_schema::UpdateEntityStatusRequest req;
  const auto names = entity_manager_ptr_->getEntityNames();
  for (const auto name : names) {
    auto status = entity_manager_ptr_->getEntityStatus(name);
    if (status) {
      openscenario_msgs::EntityStatus proto;
      status.get().name = name;
      simulation_interface::toProto(status.get(), proto);
      *req.add_status() = proto;
    }
  }
  simulation_api_schema::UpdateEntityStatusResponse res;
  update_entity_status_client_.call(req, res);
  for (const auto status : res.status()) {
    auto entity_status = entity_manager_ptr_->getEntityStatus(status.name());
    if (!entity_status) {
      continue;
    }
    openscenario_msgs::msg::EntityStatus status_msg;
    status_msg = entity_status.get();
    geometry_msgs::msg::Pose pose;
    simulation_interface::toMsg(status.pose(), pose);
    status_msg.pose = pose;
    const auto lanelet_pose = entity_manager_ptr_->toLaneletPose(pose);
    if (lanelet_pose) {
      status_msg.lanelet_pose_valid = true;
      status_msg.lanelet_pose = lanelet_pose.get();
    } else {
      status_msg.lanelet_pose_valid = false;
      status_msg.lanelet_pose = openscenario_msgs::msg::LaneletPose();
    }
    simulation_interface::toMsg(status.action_status().twist(), status_msg.action_status.twist);
    simulation_interface::toMsg(status.action_status().accel(), status_msg.action_status.accel);
    entity_manager_ptr_->setEntityStatus(status.name(), status_msg);
  }
  return res.result().success();
}

bool API::updateFrame()
{
  entity_manager_ptr_->update(current_time_, step_time_);
  traffic_controller_ptr_->execute();
  if (!standalone_mode) {
    simulation_api_schema::UpdateFrameRequest req;
    req.set_current_time(current_time_);
    simulation_api_schema::UpdateFrameResponse res;
    update_frame_client_.call(req, res);
    if (!res.result().success()) {
      return false;
    }
    entity_manager_ptr_->broadcastEntityTransform();
    current_time_ = current_time_ + step_time_;
    metrics_manager_.calculate();
    if (!updateEntityStatusInSim()) {
      return false;
    }
    return updateSensorFrame();
  }
  entity_manager_ptr_->broadcastEntityTransform();
  current_time_ = current_time_ + step_time_;
  metrics_manager_.calculate();
  return true;
}
}  // namespace scenario_simulator
