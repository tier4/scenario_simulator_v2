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

#include <simulation_api/api/api.hpp>
#include <xmlrpc_interface/xmlrpc_client.hpp>
#include <xmlrpc_interface/conversions.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <string>
#include <limits>
#include <memory>

namespace scenario_simulator
{
void API::setVerbose(bool verbose)
{
  metrics_manager_.setVerbose(verbose);
  entity_manager_ptr_->setVerbose(verbose);
}

bool API::despawn(const std::string & name)
{
  auto result = entity_manager_ptr_->despawnEntity(name);
  if (!result) {
    return false;
  }
  if (!standalone_mode) {
    simulation_api_schema::DespawnEntityRequest req;
    simulation_api_schema::DespawnEntityResponse res;
    req.set_name(name);
    xmlrpc_interface::call(client_ptr_, xmlrpc_interface::method::despawn_entity, req, res);
    return res.result().success();
  }
  return true;
}

bool API::spawn(
  const bool is_ego,
  const std::string & catalog_xml,
  const openscenario_msgs::msg::EntityStatus & status)
{
  pugi::xml_document catalog_xml_doc;
  catalog_xml_doc.load_string(catalog_xml.c_str());
  pugi::xml_node vehicle_node = catalog_xml_doc.child("Vehicle");
  if (vehicle_node != NULL) {
    const auto params = simulation_api::entity::VehicleParameters(catalog_xml_doc).toRosMsg();
    return spawn(is_ego, status.name, params);
  }
  pugi::xml_node pedestrian_node = catalog_xml_doc.child("Pedestrian");
  if (pedestrian_node != NULL) {
    const auto params = simulation_api::entity::PedestrianParameters(catalog_xml_doc).toRosMsg();
    return spawn(is_ego, status.name, params);
  }
  return false;
}

bool API::spawn(
  const bool is_ego,
  const std::string & name,
  const openscenario_msgs::msg::VehicleParameters & params)
{
  if (is_ego) {
    simulation_api::entity::EgoEntity ego(access_rights_, name, params);
    if (!entity_manager_ptr_->spawnEntity(ego)) {
      return false;
    }
    if (standalone_mode) {
      return true;
    }
    simulation_api_schema::SpawnVehicleEntityRequest req;
    simulation_api_schema::SpawnVehicleEntityResponse res;
    req.set_is_ego(true);
    xmlrpc_interface::toProto(params, *req.mutable_parameters());
    return xmlrpc_interface::call(
      client_ptr_, xmlrpc_interface::method::spawn_vehicle_entity, req,
      res);
  } else {
    simulation_api::entity::VehicleEntity npc(name, params);
    if (!entity_manager_ptr_->spawnEntity(npc)) {
      return false;
    }
    if (standalone_mode) {
      return true;
    }
    simulation_api_schema::SpawnVehicleEntityRequest req;
    simulation_api_schema::SpawnVehicleEntityResponse res;
    req.set_is_ego(false);
    xmlrpc_interface::toProto(params, *req.mutable_parameters());
    return xmlrpc_interface::call(
      client_ptr_, xmlrpc_interface::method::spawn_pedestrian_entity,
      req, res);
  }
  return false;
}

bool API::spawn(
  const bool is_ego,
  const std::string & name,
  const openscenario_msgs::msg::PedestrianParameters & params)
{
  if (is_ego) {
    throw simulation_api::SimulationRuntimeError("pedestrian should not be ego");
  }
  simulation_api::entity::PedestrianEntity pedestrian(name, params);
  if (!entity_manager_ptr_->spawnEntity(pedestrian)) {
    return false;
  }
  if (standalone_mode) {
    return true;
  }
  simulation_api_schema::SpawnPedestrianEntityRequest req;
  simulation_api_schema::SpawnPedestrianEntityResponse res;
  xmlrpc_interface::toProto(params, *req.mutable_parameters());
  return xmlrpc_interface::call(
    client_ptr_, xmlrpc_interface::method::spawn_pedestrian_entity, req,
    res);
}

bool API::spawn(
  const bool is_ego,
  const std::string & name,
  const std::string & catalog_xml)
{
  pugi::xml_document catalog_xml_doc;
  catalog_xml_doc.load_string(catalog_xml.c_str());
  pugi::xml_node vehicle_node = catalog_xml_doc.child("Vehicle");
  if (vehicle_node != NULL) {
    const auto params = simulation_api::entity::VehicleParameters(catalog_xml_doc).toRosMsg();
    spawn(is_ego, name, params);
  }
  pugi::xml_node pedestrian_node = catalog_xml_doc.child("Pedestrian");
  if (pedestrian_node != NULL) {
    const auto params = simulation_api::entity::PedestrianParameters(catalog_xml_doc).toRosMsg();
    spawn(false, name, params);
  }
  return true;
}

openscenario_msgs::msg::EntityStatus API::getEntityStatus(
  const std::string & name)
{
  auto status = entity_manager_ptr_->getEntityStatus(name);
  if (!status) {
    throw simulation_api::SimulationRuntimeError(
            "error occurs while getting entity stauts, target entity : " + name);
  }
  return status.get();
}

bool API::setEntityStatus(
  const std::string & name,
  const openscenario_msgs::msg::EntityStatus & status)
{
  return entity_manager_ptr_->setEntityStatus(name, status);
}

bool API::setEntityStatus(
  const std::string & name,
  const std::string & reference_entity_name,
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
  const std::string & name,
  const std::string & reference_entity_name,
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

void API::requestLaneChange(
  const std::string & name,
  const std::int64_t to_lanelet_id)
{
  if (entity_manager_ptr_->isEgo(name)) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    // TODO(yamacir-kit): setLaneChangeApproval(msg);
    // TODO(yamacir-kit): setLaneChangeForce(msg);
  } else {
    entity_manager_ptr_->requestLaneChange(name, to_lanelet_id);
  }
}

void API::requestLaneChange(
  const std::string & name,
  const simulation_api::entity::Direction & direction)
{
  if (!entity_manager_ptr_->isEgo(name)) {
    entity_manager_ptr_->requestLaneChange(name, direction);
  }
}

void API::setTargetSpeed(
  const std::string & name,
  const double target_speed,
  const bool continuous)
{
  if (entity_manager_ptr_->isEgo(name)) {
    std_msgs::msg::Float32 msg;
    msg.data = target_speed;
    // TODO(yamacir-kit): setVehicleVelocity(msg);
  } else {
    entity_manager_ptr_->setTargetSpeed(name, target_speed, continuous);
  }
}

boost::optional<double> API::getTimeHeadway(
  const std::string & from,
  const std::string & to)
{
  if (!entity_manager_ptr_->entityStatusSetted(from) ||
    !entity_manager_ptr_->entityStatusSetted(to))
  {
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
  const std::string & name,
  const geometry_msgs::msg::Pose & target_pose,
  const double tolerance)
{
  return
    entity_manager_ptr_->entityStatusSetted(name) &&
    entity_manager_ptr_->reachPosition(name, target_pose, tolerance);
}

bool API::reachPosition(
  const std::string & name,
  const openscenario_msgs::msg::LaneletPose & target_pose,
  const double tolerance)
{
  return
    entity_manager_ptr_->entityStatusSetted(name) &&
    entity_manager_ptr_->reachPosition(
    name, target_pose.lanelet_id, target_pose.s, target_pose.offset, tolerance);
}

bool API::reachPosition(
  const std::string & name,
  const std::string & target_name,
  const double tolerance) const
{
  return
    entity_manager_ptr_->entityStatusSetted(name) &&
    entity_manager_ptr_->entityStatusSetted(target_name) &&
    entity_manager_ptr_->reachPosition(name, target_name, tolerance);
}

bool API::setEntityStatus(
  const std::string & name,
  const openscenario_msgs::msg::LaneletPose & lanelet_pose,
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
  const std::string & name,
  const geometry_msgs::msg::Pose & map_pose,
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

openscenario_msgs::msg::EntityStatus API::toStatus(XmlRpc::XmlRpcValue param)
{
  std::string coordinate = param["coordinate"];
  std::string name = param["entity/name"];
  geometry_msgs::msg::Pose pose;
  if (coordinate == "lane") {
    std::string lanelet_id_str = param["lanelet_id"];
    std::int64_t lanelet_id = boost::lexical_cast<std::int64_t>(lanelet_id_str);
    double s = param["s"];
    double offset = param["offset"];
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = param["roll"];
    rpy.y = param["pitch"];
    rpy.z = param["yaw"];
    geometry_msgs::msg::Twist twist;
    twist.linear.x = param["twist/linear/x"];
    twist.linear.y = param["twist/linear/y"];
    twist.linear.z = param["twist/linear/z"];
    twist.angular.x = param["twist/angular/x"];
    twist.angular.y = param["twist/angular/y"];
    twist.angular.z = param["twist/angular/z"];
    geometry_msgs::msg::Accel accel;
    accel.linear.x = param["accel/linear/x"];
    accel.linear.y = param["accel/linear/y"];
    accel.linear.z = param["accel/linear/z"];
    accel.angular.x = param["accel/angular/x"];
    accel.angular.y = param["accel/angular/y"];
    accel.angular.z = param["accel/angular/z"];
    double time = param["time"];
    openscenario_msgs::msg::EntityStatus status;
    status.name = name;
    status.time = time;
    status.lanelet_pose.lanelet_id = lanelet_id;
    status.lanelet_pose.s = s;
    status.lanelet_pose.offset = offset;
    status.lanelet_pose.rpy = rpy;
    status.action_status.twist = twist;
    status.action_status.accel = accel;
    status.pose = entity_manager_ptr_->toMapPose(status.lanelet_pose);
    return status;
  }
  if (coordinate == "world") {
    pose.position.x = param["pose/position/x"];
    pose.position.y = param["pose/position/y"];
    pose.position.z = param["pose/position/z"];
    if (param.hasMember("pose/orientation/x") ||
      param.hasMember("pose/orientation/y") ||
      param.hasMember("pose/orientation/z") ||
      param.hasMember("pose/orientation/w"))
    {
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
    } else {
      pose.orientation.x = param["pose/orientation/x"];
      pose.orientation.y = param["pose/orientation/y"];
      pose.orientation.z = param["pose/orientation/z"];
      pose.orientation.w = param["pose/orientation/w"];
    }
    geometry_msgs::msg::Twist twist;
    twist.linear.x = param["twist/linear/x"];
    twist.linear.y = param["twist/linear/y"];
    twist.linear.z = param["twist/linear/z"];
    twist.angular.x = param["twist/angular/x"];
    twist.angular.y = param["twist/angular/y"];
    twist.angular.z = param["twist/angular/z"];
    geometry_msgs::msg::Accel accel;
    accel.linear.x = param["accel/linear/x"];
    accel.linear.y = param["accel/linear/y"];
    accel.linear.z = param["accel/linear/z"];
    accel.angular.x = param["accel/angular/x"];
    accel.angular.y = param["accel/angular/y"];
    accel.angular.z = param["accel/angular/z"];
    double time = param["time"];
    openscenario_msgs::msg::EntityStatus status;
    status.name = name;
    status.time = time;
    status.action_status.twist = twist;
    status.action_status.accel = accel;
    status.pose = pose;
    const auto lanelet_pose = entity_manager_ptr_->toLaneletPose(pose);
    if (lanelet_pose) {
      status.lanelet_pose_valid = true;
      status.lanelet_pose = lanelet_pose.get();
    } else {
      status.lanelet_pose_valid = false;
    }
    return status;
  }
  throw(scenario_simulator::ExecutionFailedError(
          "coordinate does not match, coordinate : " +
          coordinate));
}

XmlRpc::XmlRpcValue API::toValue(openscenario_msgs::msg::EntityStatus status)
{
  XmlRpc::XmlRpcValue param;
  param["entity/name"] = status.name;
  param["pose/position/x"] = status.pose.position.x;
  param["pose/position/y"] = status.pose.position.y;
  param["pose/position/z"] = status.pose.position.z;
  param["pose/orientation/x"] = status.pose.orientation.x;
  param["pose/orientation/y"] = status.pose.orientation.y;
  param["pose/orientation/z"] = status.pose.orientation.z;
  param["pose/orientation/w"] = status.pose.orientation.w;
  param["twist/linear/x"] = status.action_status.twist.linear.x;
  param["twist/linear/y"] = status.action_status.twist.linear.y;
  param["twist/linear/z"] = status.action_status.twist.linear.z;
  param["twist/angular/x"] = status.action_status.twist.angular.x;
  param["twist/angular/y"] = status.action_status.twist.angular.y;
  param["twist/angular/z"] = status.action_status.twist.angular.z;
  param["accel/linear/x"] = status.action_status.accel.linear.x;
  param["accel/linear/y"] = status.action_status.accel.linear.y;
  param["accel/linear/z"] = status.action_status.accel.linear.z;
  param["accel/angular/x"] = status.action_status.accel.angular.x;
  param["accel/angular/y"] = status.action_status.accel.angular.y;
  param["accel/angular/z"] = status.action_status.accel.angular.z;

  param["lanelet_id"] = std::to_string(status.lanelet_pose.lanelet_id);
  param["s"] = status.lanelet_pose.s;
  param["offset"] = status.lanelet_pose.offset;
  param["roll"] = status.lanelet_pose.rpy.x;
  param["pitch"] = status.lanelet_pose.rpy.y;
  param["yaw"] = status.lanelet_pose.rpy.z;

  param["time"] = status.time;
  return param;
}

bool API::initialize(
  double realtime_factor, double step_time)
{
  current_cmd_ = boost::none;
  current_state_cmd_ = boost::none;
  step_time_ = step_time;
  current_time_ = 0.0;
  if (standalone_mode) {
    return true;
  }
  simulation_api_schema::InitializeRequest req;
  req.set_step_time(step_time);
  req.set_realtime_factor(realtime_factor);
  simulation_api_schema::InitializeResponse res;
  return xmlrpc_interface::call(client_ptr_, xmlrpc_interface::method::initialize, req, res);
}

bool API::attachLidarSensor(
  simulation_api_schema::LidarConfiguration configuration
)
{
  if (standalone_mode) {
    return true;
  }
  simulation_api_schema::AttachLidarSensorRequest req;
  simulation_api_schema::AttachLidarSensorResponse res;
  *req.mutable_configuration() = configuration;
  xmlrpc_interface::call(client_ptr_, xmlrpc_interface::method::attach_lidar_sensor, req, res);
  return res.result().success();
}

bool API::updateSensorFrame()
{
  if (standalone_mode) {
    return true;
  }
  simulation_api_schema::UpdateSensorFrameRequest req;
  req.set_current_time(current_time_);
  simulation_api_schema::UpdateSensorFrameResponse res;
  xmlrpc_interface::call(client_ptr_, xmlrpc_interface::method::update_sensor_frame, req, res);
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
      xmlrpc_interface::toProto(status.get(), proto);
      *req.add_status() = proto;
    }
  }
  simulation_api_schema::UpdateEntityStatusResponse res;
  xmlrpc_interface::call(client_ptr_, xmlrpc_interface::method::update_entity_status, req, res);
  for (const auto status : res.status()) {
    openscenario_msgs::msg::EntityStatus msg;
    xmlrpc_interface::toMsg(status, msg);
    entity_manager_ptr_->setEntityStatus(status.name(), msg);
  }
  return res.result().success();
}

bool API::updateFrame()
{
  entity_manager_ptr_->update(current_time_, step_time_);
  entity_manager_ptr_->setVehicleCommands(current_cmd_, current_state_cmd_);
  if (!standalone_mode) {
    simulation_api_schema::UpdateFrameRequest req;
    req.set_current_time(current_time_);
    simulation_api_schema::UpdateFrameResponse res;
    xmlrpc_interface::call(client_ptr_, xmlrpc_interface::method::update_frame, req, res);
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

void API::vehicleControlCommandCallback(
  autoware_auto_msgs::msg::VehicleControlCommand::SharedPtr msg)
{
  current_cmd_ = *msg;
}

void API::vehicleStateCommandCallback(autoware_auto_msgs::msg::VehicleStateCommand::SharedPtr msg)
{
  current_state_cmd_ = *msg;
}
}  // namespace scenario_simulator
