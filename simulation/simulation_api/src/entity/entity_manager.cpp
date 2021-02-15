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

#include <simulation_api/entity/entity_manager.hpp>
#include <simulation_api/math/collision.hpp>
#include <simulation_api/helper/helper.hpp>

#include <vector>
#include <string>
#include <limits>
#include <unordered_map>

namespace simulation_api
{
namespace entity
{
bool EntityManager::isStopping(std::string name) const
{
  const auto status = getEntityStatus(name);
  if (!status) {
    throw simulation_api::SimulationRuntimeError("failed to get entity : " + name + " status");
  }
  constexpr double e = std::numeric_limits<double>::epsilon();
  if ((std::fabs(status->action_status.twist.linear.x)) < e) {
    return true;
  }
  return false;
}

void EntityManager::setDriverModel(std::string name, openscenario_msgs::msg::DriverModel model)
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    throw simulation_api::SimulationRuntimeError("entity : " + name + " does not exist");
  }
  if (it->second.type() == typeid(VehicleEntity)) {
    boost::any_cast<VehicleEntity &>(it->second).setDriverModel(model);
  }
  if (it->second.type() == typeid(EgoEntity)) {
    return;
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    throw simulation_api::SimulationRuntimeError("entity : " + name + " pedestrian");
  }
}

const boost::optional<openscenario_msgs::msg::LaneletPose> EntityManager::toLaneletPose(
  geometry_msgs::msg::Pose pose) const
{
  return hdmap_utils_ptr_->toLaneletPose(pose);
}

const geometry_msgs::msg::Pose EntityManager::toMapPose(
  const openscenario_msgs::msg::LaneletPose lanelet_pose) const
{
  return hdmap_utils_ptr_->toMapPose(lanelet_pose).pose;
}

void EntityManager::setVerbose(bool verbose)
{
  verbose_ = verbose;
  for (auto it = entities_.begin(); it != entities_.end(); it++) {
    if (it->second.type() == typeid(VehicleEntity)) {
      boost::any_cast<VehicleEntity &>(it->second).setVerbose(verbose);
    }
    if (it->second.type() == typeid(EgoEntity)) {
      boost::any_cast<EgoEntity &>(it->second).setVerbose(verbose);
    }
    if (it->second.type() == typeid(PedestrianEntity)) {
      boost::any_cast<PedestrianEntity &>(it->second).setVerbose(verbose);
    }
  }
}

bool EntityManager::isEgo(std::string name) const
{
  return getEntityType(name).type == openscenario_msgs::msg::EntityType::EGO;
}

int EntityManager::getNumberOfEgo() const
{
  return std::count_if(
    std::begin(entities_), std::end(entities_),
    [](const auto & each)
    {
      return each.second.type() == typeid(EgoEntity);
    });
}

boost::optional<openscenario_msgs::msg::LaneletPose> EntityManager::getLaneletPose(std::string name)
{
  const auto status = getEntityStatus(name);
  if (!status) {
    return boost::none;
  }
  if (status->lanelet_pose_valid) {
    return status->lanelet_pose;
  }
  return toLaneletPose(status->pose);
}

boost::optional<double> EntityManager::getDistanceToCrosswalk(
  std::string name,
  std::int64_t target_crosswalk_id)
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    return boost::none;
  }
  if (getWaypoints(name).waypoints.size() == 0) {
    return boost::none;
  }
  simulation_api::math::CatmullRomSpline spline(getWaypoints(name).waypoints);
  auto polygon = hdmap_utils_ptr_->getLaneletPolygon(target_crosswalk_id);
  return spline.getCollisionPointIn2D(polygon);
}

boost::optional<double> EntityManager::getSValueInRoute(
  std::string name,
  std::vector<std::int64_t> route)
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    return boost::none;
  }
  const auto lanelet_pose = getLaneletPose(name);
  if (!lanelet_pose) {
    return boost::none;
  }
  double s = 0;
  for (const auto id : route) {
    if (id == lanelet_pose->lanelet_id) {
      s = s + lanelet_pose->s;
      return s;
    } else {
      s = s + hdmap_utils_ptr_->getLaneletLength(id);
    }
  }
  return boost::none;
}

boost::optional<double> EntityManager::getDistanceToStopLine(
  std::string name,
  std::int64_t target_stop_line_id)
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    return boost::none;
  }
  if (getWaypoints(name).waypoints.size() == 0) {
    return boost::none;
  }
  simulation_api::math::CatmullRomSpline spline(getWaypoints(name).waypoints);
  auto polygon = hdmap_utils_ptr_->getStopLinesPolygon(target_stop_line_id);
  return spline.getCollisionPointIn2D(polygon);
}

void EntityManager::requestAcquirePosition(
  std::string name, openscenario_msgs::msg::LaneletPose lanelet_pose)
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    return;
  }
  if (it->second.type() == typeid(VehicleEntity)) {
    boost::any_cast<VehicleEntity &>(it->second).requestAcquirePosition(lanelet_pose);
  }
  if (it->second.type() == typeid(EgoEntity)) {
    boost::any_cast<EgoEntity &>(it->second).requestAcquirePosition(
      (*hdmap_utils_ptr_).toMapPose(lanelet_pose), lanelet_pose);
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    boost::any_cast<PedestrianEntity &>(it->second).requestAcquirePosition(lanelet_pose);
  }
}

void EntityManager::requestLaneChange(std::string name, std::int64_t to_lanelet_id)
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    return;
  }
  if (it->second.type() == typeid(VehicleEntity)) {
    boost::any_cast<VehicleEntity &>(it->second).requestLaneChange(to_lanelet_id);
  }
}

void EntityManager::requestLaneChange(std::string name, Direction direction)
{
  auto status = getEntityStatus(name);
  if (!status) {
    return;
  }
  if (direction == Direction::LEFT) {
    auto target = hdmap_utils_ptr_->getLaneChangeableLenletId(
      status->lanelet_pose.lanelet_id,
      "left");
    if (target) {
      requestLaneChange(name, target.get());
      return;
    }
    return;
  }
  if (direction == Direction::RIGHT) {
    auto target = hdmap_utils_ptr_->getLaneChangeableLenletId(
      status->lanelet_pose.lanelet_id,
      "right");
    if (target) {
      requestLaneChange(name, target.get());
      return;
    }
    return;
  }
}
boost::optional<double> EntityManager::getLongitudinalDistance(
  std::string from, std::string to,
  double max_distance)
{
  if (!entityStatusSetted(from) || !entityStatusSetted(to)) {
    return boost::none;
  }
  auto from_status = getEntityStatus(from);
  auto to_status = getEntityStatus(to);
  if (from_status && to_status) {
    auto dist = hdmap_utils_ptr_->getLongitudinalDistance(
      from_status->lanelet_pose.lanelet_id,
      from_status->lanelet_pose.s,
      to_status->lanelet_pose.lanelet_id, to_status->lanelet_pose.s);
    if (!dist) {
      return boost::none;
    } else {
      if (dist <= max_distance) {
        return dist.get();
      }
      return boost::none;
    }
  }
  return boost::none;
}

geometry_msgs::msg::Pose EntityManager::getMapPose(
  std::string reference_entity_name,
  geometry_msgs::msg::Pose relative_pose)
{
  const auto ref_status = getEntityStatus(reference_entity_name);
  if (!ref_status) {
    throw simulation_api::SimulationRuntimeError(
            "failed to get status of " + reference_entity_name + " entity in getMapPose");
  }
  tf2::Transform ref_transfrom, relative_transform;
  tf2::fromMsg(ref_status->pose, ref_transfrom);
  tf2::fromMsg(relative_pose, relative_transform);
  geometry_msgs::msg::Pose ret;
  tf2::toMsg(ref_transfrom * relative_transform, ret);
  return ret;
}

geometry_msgs::msg::Pose EntityManager::getRelativePose(std::string from, std::string to)
{
  auto from_status = getEntityStatus(from);
  auto to_status = getEntityStatus(to);
  if (!from_status) {
    throw simulation_api::SimulationRuntimeError(
            "failed to get status of " + from + " entity in getRelativePose");
  }
  if (!to_status) {
    throw simulation_api::SimulationRuntimeError(
            "failed to get status of " + to + " entity in getRelativePose");
  }
  auto from_pose = from_status->pose;
  auto to_pose = to_status->pose;
  return getRelativePose(from_pose, to_pose);
}

geometry_msgs::msg::Pose EntityManager::getRelativePose(
  std::string from,
  geometry_msgs::msg::Pose to)
{
  auto from_status = getEntityStatus(from);
  if (!from_status) {
    throw simulation_api::SimulationRuntimeError(
            "failed to get status of " + from + " entity in getRelativePose");
  }
  return getRelativePose(from_status->pose, to);
}

geometry_msgs::msg::Pose EntityManager::getRelativePose(
  geometry_msgs::msg::Pose from,
  std::string to)
{
  auto to_status = getEntityStatus(to);
  if (!to_status) {
    throw simulation_api::SimulationRuntimeError(
            "failed to get status of " + to + " entity in getRelativePose");
  }
  return getRelativePose(from, to_status->pose);
}

geometry_msgs::msg::Pose EntityManager::getRelativePose(
  geometry_msgs::msg::Pose from,
  geometry_msgs::msg::Pose to) const
{
  geometry_msgs::msg::Transform from_translation;
  from_translation.translation.x = from.position.x;
  from_translation.translation.y = from.position.y;
  from_translation.translation.z = from.position.z;
  from_translation.rotation = from.orientation;
  tf2::Transform from_tf;
  tf2::fromMsg(from_translation, from_tf);
  geometry_msgs::msg::Transform to_translation;
  to_translation.translation.x = to.position.x;
  to_translation.translation.y = to.position.y;
  to_translation.translation.z = to.position.z;
  to_translation.rotation = to.orientation;
  tf2::Transform to_tf;
  tf2::fromMsg(to_translation, to_tf);
  tf2::Transform tf_delta;
  tf_delta = from_tf.inverse() * to_tf;
  geometry_msgs::msg::Pose ret;
  tf2::toMsg(tf_delta, ret);
  return ret;
}

const boost::optional<openscenario_msgs::msg::VehicleParameters>
EntityManager::getVehicleParameters(std::string name) const
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    return boost::none;
  }
  if (it->second.type() == typeid(VehicleEntity)) {
    return boost::any_cast<const VehicleEntity &>(it->second).parameters;
  }
  if (it->second.type() == typeid(EgoEntity)) {
    return boost::any_cast<const EgoEntity &>(it->second).parameters;
  }
  return boost::none;
}

bool EntityManager::isInLanelet(std::string name, std::int64_t lanelet_id, double tolerance)
{
  if (!entityStatusSetted(name)) {
    return false;
  }
  double l = hdmap_utils_ptr_->getLaneletLength(lanelet_id);
  auto status = getEntityStatus(name);
  if (!status) {
    return false;
  }
  if (!status->lanelet_pose_valid) {
    return false;
  }
  if (status->lanelet_pose.lanelet_id == lanelet_id) {
    return true;
  } else {
    auto dist0 = hdmap_utils_ptr_->getLongitudinalDistance(
      lanelet_id, l,
      status->lanelet_pose.lanelet_id,
      status->lanelet_pose.s);
    auto dist1 = hdmap_utils_ptr_->getLongitudinalDistance(
      status->lanelet_pose.lanelet_id,
      status->lanelet_pose.s,
      lanelet_id, 0);
    if (dist0) {
      if (dist0.get() < tolerance) {
        return true;
      }
    }
    if (dist1) {
      if (dist1.get() < tolerance) {
        return true;
      }
    }
  }
  return false;
}

const std::vector<std::string> EntityManager::getEntityNames() const
{
  std::vector<std::string> ret;
  for (auto it = entities_.begin(); it != entities_.end(); it++) {
    ret.push_back(it->first);
  }
  return ret;
}

bool EntityManager::setEntityStatus(std::string name, openscenario_msgs::msg::EntityStatus status)
{
  auto it = entities_.find(name);
  status.name = name;
  if (it == entities_.end()) {
    return false;
  }
  if (it->second.type() == typeid(VehicleEntity)) {
    return boost::any_cast<VehicleEntity &>(it->second).setStatus(status);
  }
  if (it->second.type() == typeid(EgoEntity)) {
    return boost::any_cast<EgoEntity &>(it->second).setStatus(status);
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    return boost::any_cast<PedestrianEntity &>(it->second).setStatus(status);
  }
  return false;
}

const boost::optional<openscenario_msgs::msg::EntityStatus> EntityManager::getEntityStatus(
  std::string name) const
{
  auto it = entities_.find(name);
  openscenario_msgs::msg::EntityStatus status_msg;
  if (it == entities_.end()) {
    return boost::none;
  }
  if (it->second.type() == typeid(VehicleEntity)) {
    status_msg = boost::any_cast<const VehicleEntity &>(it->second).getStatus();
  } else if (it->second.type() == typeid(EgoEntity)) {
    status_msg = boost::any_cast<const EgoEntity &>(it->second).getStatus();
  } else if (it->second.type() == typeid(PedestrianEntity)) {
    status_msg = boost::any_cast<const PedestrianEntity &>(it->second).getStatus();
  } else {
    return boost::none;
  }
  status_msg.bounding_box = getBoundingBox(name);
  status_msg.action_status.current_action = getCurrentAction(name);
  switch (getEntityType(name).type) {
    case openscenario_msgs::msg::EntityType::EGO:
      status_msg.type.type = status_msg.type.EGO;
      break;
    case openscenario_msgs::msg::EntityType::VEHICLE:
      status_msg.type.type = status_msg.type.VEHICLE;
      break;
    case openscenario_msgs::msg::EntityType::PEDESTRIAN:
      status_msg.type.type = status_msg.type.PEDESTRIAN;
      break;
  }
  status_msg.time = current_time_;
  status_msg.name = name;
  return status_msg;
}

bool EntityManager::checkCollision(std::string name0, std::string name1)
{
  if (name0 == name1) {
    return false;
  }
  if (!entityStatusSetted(name0)) {
    return false;
  }
  if (!entityStatusSetted(name1)) {
    return false;
  }
  auto status0 = getEntityStatus(name0);
  if (!status0) {
    throw simulation_api::SimulationRuntimeError(
            "failed to calculate map pose : " + name0);
    return false;
  }
  auto status1 = getEntityStatus(name1);
  if (!status1) {
    throw simulation_api::SimulationRuntimeError(
            "failed to calculate map pose : " + name1);
  }
  auto bbox0 = getBoundingBox(name0);
  auto bbox1 = getBoundingBox(name1);
  return simulation_api::math::checkCollision2D(status0->pose, bbox0, status1->pose, bbox1);
}

const openscenario_msgs::msg::BoundingBox EntityManager::getBoundingBox(std::string name) const
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    throw simulation_api::SimulationRuntimeError(
            "error occurs while getting bounding box : " + name);
  }
  if (it->second.type() == typeid(VehicleEntity)) {
    return boost::any_cast<const VehicleEntity &>(it->second).getBoundingBox();
  }
  if (it->second.type() == typeid(EgoEntity)) {
    return boost::any_cast<const EgoEntity &>(it->second).getBoundingBox();
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    return boost::any_cast<const PedestrianEntity &>(it->second).getBoundingBox();
  }
  throw simulation_api::SimulationRuntimeError("error occurs while getting bounding box : " + name);
}

boost::optional<openscenario_msgs::msg::Obstacle> EntityManager::getObstacle(std::string name)
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    throw simulation_api::SimulationRuntimeError(
            "error occurs while getting obstacle : " + name);
  }
  if (it->second.type() == typeid(VehicleEntity)) {
    return boost::any_cast<VehicleEntity &>(it->second).getObstacle();
  }
  if (it->second.type() == typeid(EgoEntity)) {
    return boost::none;
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    return boost::none;
  }
  throw simulation_api::SimulationRuntimeError("error occurs while getting obstacle : " + name);
}

openscenario_msgs::msg::WaypointsArray EntityManager::getWaypoints(std::string name)
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    throw simulation_api::SimulationRuntimeError(
            "error occurs while getting wayoints : " + name);
  }
  if (it->second.type() == typeid(VehicleEntity)) {
    return boost::any_cast<VehicleEntity &>(it->second).getWaypoints();
  }
  if (it->second.type() == typeid(EgoEntity)) {
    return boost::any_cast<EgoEntity &>(it->second).getWaypoints();
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    return openscenario_msgs::msg::WaypointsArray();
  }
  throw simulation_api::SimulationRuntimeError("error occurs while getting waypoints : " + name);
}

boost::optional<double> EntityManager::getLinearJerk(std::string name)
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    throw simulation_api::SimulationRuntimeError(
            "entity " + name + " does not exist");
  }
  if (it->second.type() == typeid(VehicleEntity)) {
    return boost::any_cast<const VehicleEntity &>(it->second).getLinearJerk();
  }
  if (it->second.type() == typeid(EgoEntity)) {
    return boost::any_cast<const EgoEntity &>(it->second).getLinearJerk();
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    return boost::any_cast<const PedestrianEntity &>(it->second).getLinearJerk();
  }
  return boost::none;
}

bool EntityManager::entityStatusSetted(std::string name) const
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    return false;
  }
  if (it->second.type() == typeid(VehicleEntity)) {
    return boost::any_cast<const VehicleEntity &>(it->second).statusSetted();
  }
  if (it->second.type() == typeid(EgoEntity)) {
    return boost::any_cast<const EgoEntity &>(it->second).statusSetted();
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    return boost::any_cast<const PedestrianEntity &>(it->second).statusSetted();
  }
  return false;
}

void EntityManager::setTargetSpeed(std::string name, double target_speed, bool continuous)
{
  auto it = entities_.find(name);
  if (it->second.type() == typeid(VehicleEntity)) {
    boost::any_cast<VehicleEntity &>(it->second).setTargetSpeed(target_speed, continuous);
  }
  if (it->second.type() == typeid(EgoEntity)) {
    boost::any_cast<EgoEntity &>(it->second).setTargetSpeed(target_speed, continuous);
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    boost::any_cast<PedestrianEntity &>(it->second).setTargetSpeed(target_speed, continuous);
  }
}

std::vector<std::int64_t> EntityManager::getRouteLanelets(std::string name, double horizon)
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    throw SimulationRuntimeError("entity " + name + " does not exist");
  }
  if (it->second.type() == typeid(VehicleEntity)) {
    const auto route = boost::any_cast<VehicleEntity &>(it->second).getRouteLanelets(horizon);
    return route;
  }
  if (it->second.type() == typeid(EgoEntity)) {
    const auto route = boost::any_cast<EgoEntity &>(it->second).getRouteLanelets(horizon);
    return route;
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    const auto route = boost::any_cast<PedestrianEntity &>(it->second).getRouteLanelets(horizon);
    return route;
  }
  throw SimulationRuntimeError("entity " + name + " does not matches to entity type.");
}

std::vector<std::int64_t> EntityManager::getConflictingEntityOnRouteLanelets(
  std::string name,
  double horizon)
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    throw SimulationRuntimeError("entity " + name + " does not exist");
  }
  const auto route = getRouteLanelets(name, horizon);
  return hdmap_utils_ptr_->getConflictingCrosswalkIds(route);
}

double EntityManager::getStepTime() const
{
  return step_time_;
}

double EntityManager::getCurrentTime() const
{
  return current_time_;
}

void EntityManager::update(double current_time, double step_time)
{
  std::chrono::system_clock::time_point start, end;
  start = std::chrono::system_clock::now();
  step_time_ = step_time;
  traffic_light_manager_ptr_->update(step_time_);
  current_time_ = current_time;
  if (verbose_) {
    std::cout << "-------------------------- UPDATE --------------------------" << std::endl;
  }
  if (getNumberOfEgo() >= 2) {
    throw SimulationRuntimeError("multi ego simulation does not support yet.");
  }
  setVerbose(verbose_);
  auto type_list = getEntityTypeList();
  std::unordered_map<std::string, openscenario_msgs::msg::EntityStatus> all_status;
  for (auto it = entities_.begin(); it != entities_.end(); it++) {
    if (verbose_) {
      std::cout << "update " << it->first << " behavior" << std::endl;
    }
    if (it->second.type() == typeid(VehicleEntity)) {
      boost::any_cast<VehicleEntity &>(it->second).setEntityTypeList(type_list);
      boost::any_cast<VehicleEntity &>(it->second).onUpdate(current_time, step_time);
      if (boost::any_cast<VehicleEntity &>(it->second).statusSetted()) {
        auto status = boost::any_cast<VehicleEntity &>(it->second).getStatus();
        all_status[boost::any_cast<VehicleEntity &>(it->second).name] = status;
      }
    }
    if (it->second.type() == typeid(EgoEntity)) {
      boost::any_cast<EgoEntity &>(it->second).setEntityTypeList(type_list);
      auto kinematic_state =
        boost::any_cast<EgoEntity &>(it->second).getCurrentKinematicState();
      if (kinematic_state) {
        autoware_auto_msgs::msg::VehicleKinematicState msg;
        msg = kinematic_state.get();
        msg.header.frame_id = "map";
        msg.header.stamp = clock_ptr_->now();
        kinematic_state_pub_ptr_->publish(msg);
      }
      boost::any_cast<EgoEntity &>(it->second).onUpdate(current_time, step_time);
      if (boost::any_cast<EgoEntity &>(it->second).statusSetted()) {
        auto status = boost::any_cast<EgoEntity &>(it->second).getStatus();
        all_status[boost::any_cast<EgoEntity &>(it->second).name] = status;
      }
    }
    if (it->second.type() == typeid(PedestrianEntity)) {
      boost::any_cast<PedestrianEntity &>(it->second).setEntityTypeList(type_list);
      boost::any_cast<PedestrianEntity &>(it->second).onUpdate(current_time, step_time);
      if (boost::any_cast<PedestrianEntity &>(it->second).statusSetted()) {
        auto status = boost::any_cast<PedestrianEntity &>(it->second).getStatus();
        all_status[boost::any_cast<PedestrianEntity &>(it->second).name] = status;
      }
    }
  }
  for (auto it = entities_.begin(); it != entities_.end(); it++) {
    if (it->second.type() == typeid(VehicleEntity)) {
      boost::any_cast<VehicleEntity &>(it->second).setOtherStatus(all_status);
    }
    if (it->second.type() == typeid(EgoEntity)) {
      boost::any_cast<EgoEntity &>(it->second).setOtherStatus(all_status);
    }
    if (it->second.type() == typeid(PedestrianEntity)) {
      boost::any_cast<PedestrianEntity &>(it->second).setOtherStatus(all_status);
    }
  }
  auto entity_type_list = getEntityTypeList();
  openscenario_msgs::msg::EntityStatusWithTrajectoryArray status_array_msg;
  for (const auto & status : all_status) {
    openscenario_msgs::msg::EntityStatusWithTrajectory status_with_traj;
    auto status_msg = status.second;
    status_msg.name = status.first;
    status_msg.bounding_box = getBoundingBox(status.first);
    status_msg.action_status.current_action = getCurrentAction(status.first);
    switch (getEntityType(status.first).type) {
      case openscenario_msgs::msg::EntityType::EGO:
        status_msg.type.type = status_msg.type.EGO;
        break;
      case openscenario_msgs::msg::EntityType::VEHICLE:
        status_msg.type.type = status_msg.type.VEHICLE;
        break;
      case openscenario_msgs::msg::EntityType::PEDESTRIAN:
        status_msg.type.type = status_msg.type.PEDESTRIAN;
        break;
    }
    status_with_traj.waypoint = getWaypoints(status.first);
    const auto obstacle = getObstacle(status.first);
    if (obstacle) {
      status_with_traj.obstacle = obstacle.get();
      status_with_traj.obstacle_find = true;
    } else {
      status_with_traj.obstacle_find = false;
    }
    status_with_traj.status = status_msg;
    status_with_traj.name = status.first;
    status_with_traj.time = current_time + step_time;
    status_array_msg.data.emplace_back(status_with_traj);
  }
  entity_status_array_pub_ptr_->publish(status_array_msg);
  end = std::chrono::system_clock::now();
  double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  if (verbose_) {
    std::cout << "elapsed " << elapsed / 1000 << " secands in update function." << std::endl;
  }
}

void EntityManager::broadcastTransform(geometry_msgs::msg::PoseStamped pose, bool static_transform)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = pose.header.stamp;
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = pose.header.frame_id;
  transform_stamped.transform.translation.x = pose.pose.position.x;
  transform_stamped.transform.translation.y = pose.pose.position.y;
  transform_stamped.transform.translation.z = pose.pose.position.z;
  transform_stamped.transform.rotation = pose.pose.orientation;
  if (static_transform) {
    broadcaster_.sendTransform(transform_stamped);
  } else {
    base_link_broadcaster_.sendTransform(transform_stamped);
  }
}

bool EntityManager::reachPosition(
  std::string name, std::string target_name, double tolerance) const
{
  auto status = getEntityStatus(target_name);
  if (status) {
    return false;
  }
  return reachPosition(name, status->pose, tolerance);
}

bool EntityManager::reachPosition(
  std::string name, geometry_msgs::msg::Pose target_pose,
  double tolerance) const
{
  auto status = getEntityStatus(name);
  if (!status) {
    throw simulation_api::SimulationRuntimeError(
            "error occurs while getting entity stauts, target entity : " + name);
  }
  auto pose = status->pose;
  double dist = std::sqrt(
    std::pow(
      pose.position.x - target_pose.position.x,
      2) + std::pow(pose.position.y - target_pose.position.y, 2) +
    std::pow(pose.position.z - target_pose.position.z, 2));
  return dist < tolerance;
}

bool EntityManager::reachPosition(
  std::string name, std::int64_t lanelet_id, double s, double offset,
  double tolerance) const
{
  openscenario_msgs::msg::LaneletPose lanelet_pose;
  lanelet_pose.lanelet_id = lanelet_id;
  lanelet_pose.s = s;
  lanelet_pose.offset = offset;
  auto target_pose = hdmap_utils_ptr_->toMapPose(lanelet_pose);
  return reachPosition(name, target_pose.pose, tolerance);
}

void EntityManager::broadcastBaseLinkTransform()
{
  // for (const auto & name : getEntityNames()) {
  //   if (getEntityType(name).type == openscenario_msgs::msg::EntityType::EGO) {
  //     auto status = getEntityStatus(name);
  //     if (status) {
  //       geometry_msgs::msg::PoseStamped pose;
  //       pose.pose = status->pose;
  //       pose.header.stamp = clock_ptr_->now();
  //       pose.header.frame_id = "base_link";
  //       broadcastTransform(pose, false);
  //     }
  //     return;
  //   }
  // }
}

void EntityManager::broadcastEntityTransform()
{
  std::vector<std::string> names = getEntityNames();
  for (auto it = names.begin(); it != names.end(); it++) {
    if (entityStatusSetted(*it)) {
      auto status = getEntityStatus(*it);
      if (status) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose = status->pose;
        pose.header.stamp = clock_ptr_->now();
        pose.header.frame_id = *it;
        broadcastTransform(pose);
      }
    }
  }
  // broadcastBaseLinkTransform();
}

const boost::optional<double> EntityManager::getStandStillDuration(std::string name) const
{
  auto it = entities_.find(name);
  if (it->second.type() == typeid(VehicleEntity)) {
    return boost::any_cast<const VehicleEntity &>(it->second).getStandStillDuration();
  }
  if (it->second.type() == typeid(EgoEntity)) {
    return boost::any_cast<const EgoEntity &>(it->second).getStandStillDuration();
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    return boost::any_cast<const PedestrianEntity &>(it->second).getStandStillDuration();
  }
  throw simulation_api::SimulationRuntimeError("entity " + name + "does not exist");
}

const std::string EntityManager::getCurrentAction(std::string name) const
{
  auto it = entities_.find(name);
  if (it->second.type() == typeid(VehicleEntity)) {
    return boost::any_cast<const VehicleEntity &>(it->second).getCurrentAction();
  }
  if (it->second.type() == typeid(EgoEntity)) {
    return boost::any_cast<const EgoEntity &>(it->second).getCurrentAction();
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    return boost::any_cast<const PedestrianEntity &>(it->second).getCurrentAction();
  }
  throw simulation_api::SimulationRuntimeError("entity " + name + "does not exist");
}

openscenario_msgs::msg::EntityType EntityManager::getEntityType(std::string name) const
{
  auto it = entities_.find(name);
  if (it->second.type() == typeid(VehicleEntity)) {
    openscenario_msgs::msg::EntityType type;
    type.type = openscenario_msgs::msg::EntityType::VEHICLE;
    return type;
  }
  if (it->second.type() == typeid(EgoEntity)) {
    openscenario_msgs::msg::EntityType type;
    type.type = openscenario_msgs::msg::EntityType::EGO;
    return type;
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    openscenario_msgs::msg::EntityType type;
    type.type = openscenario_msgs::msg::EntityType::PEDESTRIAN;
    return type;
  }
  throw simulation_api::SimulationRuntimeError("entity " + name + "does not exist");
}

const std::unordered_map<std::string,
  openscenario_msgs::msg::EntityType> EntityManager::getEntityTypeList() const
{
  std::unordered_map<std::string, openscenario_msgs::msg::EntityType> ret;
  for (auto it = entities_.begin(); it != entities_.end(); it++) {
    if (it->second.type() == typeid(VehicleEntity)) {
      openscenario_msgs::msg::EntityType type;
      type.type = openscenario_msgs::msg::EntityType::VEHICLE;
      ret[it->first] = type;
    }
    if (it->second.type() == typeid(EgoEntity)) {
      openscenario_msgs::msg::EntityType type;
      type.type = openscenario_msgs::msg::EntityType::EGO;
      ret[it->first] = type;
    }
    if (it->second.type() == typeid(PedestrianEntity)) {
      openscenario_msgs::msg::EntityType type;
      type.type = openscenario_msgs::msg::EntityType::PEDESTRIAN;
      ret[it->first] = type;
    }
  }
  return ret;
}
}  // namespace entity
}  // namespace simulation_api
