// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#include <string>
#include <limits>

namespace scenario_simulator
{
bool API::spawn(
  bool is_ego, std::string name,
  std::string catalog_xml,
  simulation_api::entity::EntityStatus status)
{
  XmlRpc::XmlRpcValue value, status_value;
  status_value = toValue(name, status);
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
      simulation_api::entity::EgoEntity ego(name, status, catalog_xml_doc);
      if (!entity_manager_ptr_->spawnEntity(ego)) {
        return false;
      }
    } else {
      simulation_api::entity::VehicleEntity npc(name, status, catalog_xml_doc);
      if (!entity_manager_ptr_->spawnEntity(npc)) {
        return false;
      }
    }
  }
  pugi::xml_node pedestrian_node = catalog_xml_doc.child("Pedestrian");
  if (pedestrian_node != NULL) {
    simulation_api::entity::PedestrianEntity pedestrian(name, status, catalog_xml_doc);
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
  return result[0][0]["success"];
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
  simulation_api::entity::EntityStatus status)
{
  return spawn(is_ego, name, params.toXml(), status);
}

bool API::spawn(
  bool is_ego, std::string name,
  simulation_api::entity::PedestrianParameters params,
  simulation_api::entity::EntityStatus status)
{
  return spawn(is_ego, name, params.toXml(), status);
}

bool API::spawn(
  bool is_ego, std::string name,
  simulation_api::entity::PedestrianParameters params)
{
  return spawn(is_ego, name, params.toXml());
}

simulation_api::entity::EntityStatus API::getEntityStatus(
  std::string name,
  simulation_api::entity::CoordinateFrameTypes corrdinate)
{
  auto status = entity_manager_ptr_->getEntityStatus(name, corrdinate);
  if (!status) {
    throw simulation_api::SimulationRuntimeError(
            "error occurs while getting entity stauts, target entity : " + name);
  }
  return status.get();
}

bool API::setEntityStatus(std::string name, const simulation_api::entity::EntityStatus & status)
{
  return entity_manager_ptr_->setEntityStatus(name, status);
}

boost::optional<double> API::getLongitudinalDistance(std::string from, std::string to)
{
  return entity_manager_ptr_->getLongitudinalDistance(from, to);
}

void API::requestAcquirePosition(std::string name, std::int64_t lanelet_id, double s, double offset)
{
  entity_manager_ptr_->requestAcquirePosition(name, lanelet_id, s, offset);
}

void API::requestLaneChange(std::string name, std::int64_t to_lanelet_id)
{
  entity_manager_ptr_->requestLaneChange(name, to_lanelet_id);
}

void API::requestLaneChange(std::string name, simulation_api::entity::Direction direction)
{
  entity_manager_ptr_->requestLaneChange(name, direction);
}

bool API::isInLanelet(std::string name, std::int64_t lanelet_id, double tolerance)
{
  return entity_manager_ptr_->isInLanelet(name, lanelet_id, tolerance);
}

void API::setTargetSpeed(std::string name, double target_speed, bool continuous)
{
  entity_manager_ptr_->setTargetSpeed(name, target_speed, continuous);
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
  simulation_api::entity::EntityStatus to_status = getEntityStatus(to);
  double ret = (pose.position.x * -1) / (to_status.twist.linear.x);
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
  std::string name, std::int64_t lanelet_id, double s, double offset,
  double tolerance)
{
  if (!entity_manager_ptr_->entityStatusSetted(name)) {
    return false;
  }
  return entity_manager_ptr_->reachPosition(name, lanelet_id, s, offset, tolerance);
}
boost::optional<double> API::getStandStillDuration(std::string name) const
{
  return entity_manager_ptr_->getStandStillDuration(name);
}
bool API::checkCollision(std::string name0, std::string name1)
{
  return entity_manager_ptr_->checkCollision(name0, name1);
}

simulation_api::entity::EntityStatus API::toStatus(XmlRpc::XmlRpcValue param)
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
    simulation_api::entity::EntityStatus status(time, lanelet_id, s, offset, rpy, twist,
      accel);
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
    simulation_api::entity::EntityStatus status(time, pose, twist, accel);
    return status;
  }
  throw(scenario_simulator::ExecutionFailedError("coordinate does not match, coordinate : " +
        coordinate));
}
XmlRpc::XmlRpcValue API::toValue(std::string name, simulation_api::entity::EntityStatus status)
{
  if (status.coordinate == simulation_api::entity::CoordinateFrameTypes::WORLD) {
    XmlRpc::XmlRpcValue param;
    param["entity/name"] = name;
    param["coordinate"] = "world";
    param["pose/position/x"] = status.pose.position.x;
    param["pose/position/y"] = status.pose.position.y;
    param["pose/position/z"] = status.pose.position.z;
    param["pose/orientation/x"] = status.pose.orientation.x;
    param["pose/orientation/y"] = status.pose.orientation.y;
    param["pose/orientation/z"] = status.pose.orientation.z;
    param["pose/orientation/w"] = status.pose.orientation.w;
    param["twist/linear/x"] = status.twist.linear.x;
    param["twist/linear/y"] = status.twist.linear.y;
    param["twist/linear/z"] = status.twist.linear.z;
    param["twist/angular/x"] = status.twist.angular.x;
    param["twist/angular/y"] = status.twist.angular.y;
    param["twist/angular/z"] = status.twist.angular.z;
    param["accel/linear/x"] = status.accel.linear.x;
    param["accel/linear/y"] = status.accel.linear.y;
    param["accel/linear/z"] = status.accel.linear.z;
    param["accel/angular/x"] = status.accel.angular.x;
    param["accel/angular/y"] = status.accel.angular.y;
    param["accel/angular/z"] = status.accel.angular.z;
    param["time"] = status.time;
    return param;
  }
  if (status.coordinate == simulation_api::entity::CoordinateFrameTypes::LANE) {
    XmlRpc::XmlRpcValue param;
    param["entity/name"] = name;
    param["coordinate"] = "lane";
    param["lanelet_id"] = std::to_string(status.lanelet_id);
    param["s"] = status.s;
    param["offset"] = status.offset;
    param["roll"] = status.rpy.x;
    param["pitch"] = status.rpy.y;
    param["yaw"] = status.rpy.z;
    param["twist/linear/x"] = status.twist.linear.x;
    param["twist/linear/y"] = status.twist.linear.y;
    param["twist/linear/z"] = status.twist.linear.z;
    param["twist/angular/x"] = status.twist.angular.x;
    param["twist/angular/y"] = status.twist.angular.y;
    param["twist/angular/z"] = status.twist.angular.z;
    param["accel/linear/x"] = status.accel.linear.x;
    param["accel/linear/y"] = status.accel.linear.y;
    param["accel/linear/z"] = status.accel.linear.z;
    param["accel/angular/x"] = status.accel.angular.x;
    param["accel/angular/y"] = status.accel.angular.y;
    param["accel/angular/z"] = status.accel.angular.z;
    param["time"] = status.time;
    return param;
  }
  throw(scenario_simulator::ExecutionFailedError("coordinate does not match"));
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
