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
#include <tf2/LinearMath/Quaternion.h>

#include <string>
#include <limits>
#include <memory>

namespace scenario_simulator
{
bool API::despawnEntity(std::string name)
{
  return entity_manager_ptr_->despawnEntity(name);
}

bool API::entityExists(std::string name)
{
  return entity_manager_ptr_->entityExists(name);
}

boost::optional<double> API::getLinearJerk(std::string name)
{
  return entity_manager_ptr_->getLinearJerk(name);
}

void API::setVerbose(bool verbose)
{
  metrics_manager_.setVerbose(verbose);
  entity_manager_ptr_->setVerbose(verbose);
}

const boost::optional<openscenario_msgs::msg::LaneletPose> API::toLaneletPose(
  geometry_msgs::msg::Pose pose) const
{
  return entity_manager_ptr_->toLaneletPose(pose);
}

const geometry_msgs::msg::Pose API::toMapPose(
  const openscenario_msgs::msg::LaneletPose lanelet_pose)
const
{
  return entity_manager_ptr_->toMapPose(lanelet_pose);
}

void API::setDriverModel(std::string name, const openscenario_msgs::msg::DriverModel & model)
{
  entity_manager_ptr_->setDriverModel(name, model);
}

bool API::spawn(
  bool is_ego,
  std::string catalog_xml,
  openscenario_msgs::msg::EntityStatus status)
{
  XmlRpc::XmlRpcValue value, status_value;
  status_value = toValue(status);
  value[0][0]["methodName"] = "spawn_entity";
  value[0][0]["params"] = status_value;
  value[0][0]["params"]["entity/is_ego"] = is_ego;
  value[0][0]["params"]["entity/catalog_xml"] = catalog_xml;
  pugi::xml_document catalog_xml_doc;
  catalog_xml_doc.load_string(catalog_xml.c_str());
  pugi::xml_node vehicle_node = catalog_xml_doc.child("Vehicle");
  // catalog_xml_doc.has("Vehicle");
  if (vehicle_node != NULL) {
    if (is_ego) {
      simulation_api::entity::EgoEntity ego(status.name, status, catalog_xml_doc);
      if (!entity_manager_ptr_->spawnEntity(ego)) {
        return false;
      }
    } else {
      simulation_api::entity::VehicleEntity npc(status.name, status, catalog_xml_doc);
      if (!entity_manager_ptr_->spawnEntity(npc)) {
        return false;
      }
    }
  }
  pugi::xml_node pedestrian_node = catalog_xml_doc.child("Pedestrian");
  if (pedestrian_node != NULL) {
    simulation_api::entity::PedestrianEntity pedestrian(status.name, status, catalog_xml_doc);
    if (!entity_manager_ptr_->spawnEntity(pedestrian)) {
      return false;
    }
  }

  XmlRpc::XmlRpcValue result;
  try {
    client_ptr_->execute("system.multicall", value, result);
  } catch (XmlRpc::XmlRpcException e) {
    throw XmlRpcRuntimeError(e.getMessage().c_str(), e.getCode());
  }
  return result[0][0]["success"];
}

bool API::spawn(
  bool is_ego, std::string name,
  std::string catalog_xml,
  const geometry_msgs::msg::Pose & map_pose,
  const openscenario_msgs::msg::ActionStatus & action_status)
{
  if (!spawn(is_ego, name, catalog_xml)) {
    return false;
  }
  if (!setEntityStatus(name, map_pose, action_status)) {
    return false;
  }
  return true;
}

bool API::spawn(
  bool is_ego, std::string name,
  std::string catalog_xml,
  const openscenario_msgs::msg::LaneletPose & lanelet_pose,
  const openscenario_msgs::msg::ActionStatus & action_status)
{
  if (!spawn(is_ego, name, catalog_xml)) {
    return false;
  }
  if (!setEntityStatus(name, lanelet_pose, action_status)) {
    return false;
  }
  return true;
}

bool API::spawn(
  bool is_ego, std::string name,
  std::string catalog_xml)
{
  XmlRpc::XmlRpcValue value;
  value[0][0]["methodName"] = "spawn_entity";
  value[0][0]["params"]["entity/is_ego"] = is_ego;
  value[0][0]["params"]["entity/catalog_xml"] = catalog_xml;

  pugi::xml_document catalog_xml_doc;
  catalog_xml_doc.load_string(catalog_xml.c_str());
  pugi::xml_node vehicle_node = catalog_xml_doc.child("Vehicle");
  // catalog_xml_doc.has("Vehicle");
  if (vehicle_node != NULL) {
    if (is_ego) {
      simulation_api::entity::EgoEntity ego(name, catalog_xml_doc);
      if (!entity_manager_ptr_->spawnEntity(ego)) {
        return false;
      }
    } else {
      simulation_api::entity::VehicleEntity npc(name, catalog_xml_doc);
      if (!entity_manager_ptr_->spawnEntity(npc)) {
        return false;
      }
    }
  }
  pugi::xml_node pedestrian_node = catalog_xml_doc.child("Pedestrian");
  if (pedestrian_node != NULL) {
    simulation_api::entity::PedestrianEntity pedestrian(name, catalog_xml_doc);
    if (!entity_manager_ptr_->spawnEntity(pedestrian)) {
      return false;
    }
  }

  XmlRpc::XmlRpcValue result;
  try {
    client_ptr_->execute("system.multicall", value, result);
  } catch (XmlRpc::XmlRpcException e) {
    throw XmlRpcRuntimeError(e.getMessage().c_str(), e.getCode());
  }
  return true;
}

bool API::spawn(
  bool is_ego, std::string name,
  simulation_api::entity::VehicleParameters params)
{
  return spawn(is_ego, name, params.toXml());
}

bool API::spawn(
  bool is_ego, std::string name,
  simulation_api::entity::VehicleParameters params,
  const geometry_msgs::msg::Pose & map_pose,
  const openscenario_msgs::msg::ActionStatus & action_status)
{
  if (!spawn(is_ego, name, params)) {
    return false;
  }
  if (!setEntityStatus(name, map_pose, action_status)) {
    return false;
  }
  return true;
}

bool API::spawn(
  bool is_ego, std::string name,
  simulation_api::entity::VehicleParameters params,
  const openscenario_msgs::msg::LaneletPose & lanelet_pose,
  const openscenario_msgs::msg::ActionStatus & action_status)
{
  if (!spawn(is_ego, name, params)) {
    return false;
  }
  if (!setEntityStatus(name, lanelet_pose, action_status)) {
    return false;
  }
  return true;
}

bool API::spawn(
  bool is_ego,
  simulation_api::entity::VehicleParameters params,
  openscenario_msgs::msg::EntityStatus status)
{
  return spawn(is_ego, params.toXml(), status);
}

bool API::spawn(
  bool is_ego,
  simulation_api::entity::PedestrianParameters params,
  openscenario_msgs::msg::EntityStatus status)
{
  return spawn(is_ego, params.toXml(), status);
}

bool API::spawn(
  bool is_ego, std::string name,
  simulation_api::entity::PedestrianParameters params)
{
  return spawn(is_ego, name, params.toXml());
}

bool API::spawn(
  bool is_ego, std::string name,
  simulation_api::entity::PedestrianParameters params,
  const openscenario_msgs::msg::LaneletPose & lanelet_pose,
  const openscenario_msgs::msg::ActionStatus & action_status)
{
  if (!spawn(is_ego, name, params)) {
    return false;
  }
  if (!setEntityStatus(name, lanelet_pose, action_status)) {
    return false;
  }
  return true;
}

bool API::spawn(
  bool is_ego, std::string name,
  simulation_api::entity::PedestrianParameters params,
  const geometry_msgs::msg::Pose & map_pose,
  const openscenario_msgs::msg::ActionStatus & action_status)
{
  if (!spawn(is_ego, name, params)) {
    return false;
  }
  if (!setEntityStatus(name, map_pose, action_status)) {
    return false;
  }
  return true;
}

openscenario_msgs::msg::EntityStatus API::getEntityStatus(
  std::string name)
{
  auto status = entity_manager_ptr_->getEntityStatus(name);
  if (!status) {
    throw simulation_api::SimulationRuntimeError(
            "error occurs while getting entity stauts, target entity : " + name);
  }
  return status.get();
}

bool API::setEntityStatus(std::string name, const openscenario_msgs::msg::EntityStatus & status)
{
  return entity_manager_ptr_->setEntityStatus(name, status);
}

bool API::setEntityStatus(
  std::string name, std::string reference_entity_name,
  const geometry_msgs::msg::Point relative_position,
  const geometry_msgs::msg::Vector3 relative_rpy,
  const openscenario_msgs::msg::ActionStatus action_status)
{
  geometry_msgs::msg::Pose relative_pose;
  relative_pose.position = relative_position;
  relative_pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(relative_rpy);
  return setEntityStatus(name, reference_entity_name, relative_pose, action_status);
}

bool API::setEntityStatus(
  std::string name, std::string reference_entity_name,
  const geometry_msgs::msg::Pose relative_pose,
  const openscenario_msgs::msg::ActionStatus action_status)
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

boost::optional<double> API::getLongitudinalDistance(std::string from, std::string to)
{
  return entity_manager_ptr_->getLongitudinalDistance(from, to);
}

void API::requestAcquirePosition(std::string name, openscenario_msgs::msg::LaneletPose lanelet_pose)
{
  entity_manager_ptr_->requestAcquirePosition(name, lanelet_pose);
}

void API::requestLaneChange(std::string name, std::int64_t to_lanelet_id)
{
  if (entity_manager_ptr_->isEgo(name)) {
    std_msgs::msg::Bool msg;
    msg.data = true;
    setLaneChangeApproval(msg);
    // setLaneChangeForce(msg);
  } else {
    entity_manager_ptr_->requestLaneChange(name, to_lanelet_id);
  }
}

void API::requestLaneChange(std::string name, simulation_api::entity::Direction direction)
{
  if (!entity_manager_ptr_->isEgo(name)) {
    entity_manager_ptr_->requestLaneChange(name, direction);
  }
}

bool API::isInLanelet(std::string name, std::int64_t lanelet_id, double tolerance)
{
  return entity_manager_ptr_->isInLanelet(name, lanelet_id, tolerance);
}

void API::setTargetSpeed(std::string name, double target_speed, bool continuous)
{
  if (entity_manager_ptr_->isEgo(name)) {
    std_msgs::msg::Float32 msg;
    msg.data = target_speed;
    setVehicleVelocity(msg);
  } else {
    entity_manager_ptr_->setTargetSpeed(name, target_speed, continuous);
  }
}

boost::optional<double> API::getTimeHeadway(std::string from, std::string to)
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
geometry_msgs::msg::Pose API::getRelativePose(std::string from, std::string to)
{
  return entity_manager_ptr_->getRelativePose(from, to);
}

geometry_msgs::msg::Pose API::getRelativePose(geometry_msgs::msg::Pose from, std::string to)
{
  return entity_manager_ptr_->getRelativePose(from, to);
}

geometry_msgs::msg::Pose API::getRelativePose(std::string from, geometry_msgs::msg::Pose to)
{
  return entity_manager_ptr_->getRelativePose(from, to);
}

geometry_msgs::msg::Pose API::getRelativePose(
  geometry_msgs::msg::Pose from,
  geometry_msgs::msg::Pose to)
{
  return entity_manager_ptr_->getRelativePose(from, to);
}

bool API::reachPosition(std::string name, geometry_msgs::msg::Pose target_pose, double tolerance)
{
  if (!entity_manager_ptr_->entityStatusSetted(name)) {
    return false;
  }
  return entity_manager_ptr_->reachPosition(name, target_pose, tolerance);
}

bool API::reachPosition(
  std::string name, openscenario_msgs::msg::LaneletPose target_pose,
  double tolerance)
{
  if (!entity_manager_ptr_->entityStatusSetted(name)) {
    return false;
  }
  return entity_manager_ptr_->reachPosition(
    name,
    target_pose.lanelet_id, target_pose.s, target_pose.offset, tolerance);
}

boost::optional<double> API::getStandStillDuration(std::string name) const
{
  return entity_manager_ptr_->getStandStillDuration(name);
}

bool API::checkCollision(std::string name0, std::string name1)
{
  return entity_manager_ptr_->checkCollision(name0, name1);
}

bool API::setEntityStatus(
  std::string name, const openscenario_msgs::msg::LaneletPose & lanelet_pose,
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
  std::string name, const geometry_msgs::msg::Pose & map_pose,
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
XmlRpc::XmlRpcValue API::initialize(
  double realtime_factor, double step_time, int times_try,
  int duration_try_in_msec)
{
  current_cmd_ = boost::none;
  current_state_cmd_ = boost::none;
  step_time_ = step_time;
  current_time_ = 0.0;
  XmlRpc::XmlRpcValue value;
  value[0][0]["methodName"] = "initialize";
  value[0][0]["params"]["sim/realtime_factor"] = realtime_factor;
  value[0][0]["params"]["sim/step_time"] = step_time;
  XmlRpc::XmlRpcValue result;
  for (int count_try = 0; count_try < times_try; count_try = count_try + 1) {
    try {
      client_ptr_->execute("system.multicall", value, result);
      if (result[0][0].hasMember("sim/initialized")) {
        return result[0][0];
      }
    } catch (...) {
      std::this_thread::sleep_for(std::chrono::milliseconds(duration_try_in_msec));
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(duration_try_in_msec));
  }
  throw ExecutionFailedError("failed to call initaialize API, xmlrpc timeout");
}

XmlRpc::XmlRpcValue API::updateFrame()
{
  entity_manager_ptr_->update(current_time_, step_time_);
  entity_manager_ptr_->setVehicleCommands(current_cmd_, current_state_cmd_);
  XmlRpc::XmlRpcValue value;
  value[0][0]["methodName"] = "update_frame";
  value[0][0]["params"]["runner/current_time"] = current_time_;
  XmlRpc::XmlRpcValue result;
  try {
    client_ptr_->execute("system.multicall", value, result);
  } catch (XmlRpc::XmlRpcException e) {
    throw XmlRpcRuntimeError(e.getMessage().c_str(), e.getCode());
  }
  if (!result[0][0].hasMember("sim/update_frame")) {
    throw ExecutionFailedError("there is no sim/update_frmae field in the result");
  }
  if (!result[0][0]["sim/update_frame"]) {
    throw ExecutionFailedError("failed to update simulation frame");
  }
  entity_manager_ptr_->broadcastEntityTransform();
  current_time_ = current_time_ + step_time_;
  metrics_manager_.calculate();
  return result[0][0];
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
