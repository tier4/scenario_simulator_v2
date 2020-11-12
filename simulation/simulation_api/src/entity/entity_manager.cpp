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

#include <simulation_api/entity/entity_manager.hpp>
#include <simulation_api/math/collision.hpp>

#include <vector>
#include <string>
#include <unordered_map>

namespace simulation_api
{
namespace entity
{
void EntityManager::setVerbose(bool verbose)
{
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

int EntityManager::getNumberOfEgo() const
{
  int count = 0;
  for (auto it = entities_.begin(); it != entities_.end(); it++) {
    if (it->second.type() == typeid(EgoEntity)) {
      count = count + 1;
    }
  }
  return count;
}

void EntityManager::requestAcquirePosition(
  std::string name, std::int64_t lanelet_id, double s,
  double offset)
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    return;
  }
  if (it->second.type() == typeid(VehicleEntity)) {
    boost::any_cast<VehicleEntity &>(it->second).requestAcquirePosition(lanelet_id, s, offset);
  }
  if (it->second.type() == typeid(EgoEntity)) {
    boost::any_cast<EgoEntity &>(it->second).requestAcquirePosition(lanelet_id, s, offset);
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    boost::any_cast<PedestrianEntity &>(it->second).requestAcquirePosition(lanelet_id, s, offset);
  }
}

void EntityManager::requestLaneChange(std::string name, std::int64_t to_lanelet_id)
{
  if (getEntityStatusCoordinate(name) == CoordinateFrameTypes::WORLD) {
    return;
  }
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    return;
  }
  if (it->second.type() == typeid(VehicleEntity)) {
    boost::any_cast<VehicleEntity &>(it->second).requestLaneChange(to_lanelet_id);
  }
  if (it->second.type() == typeid(EgoEntity)) {
    boost::any_cast<EgoEntity &>(it->second).requestLaneChange(to_lanelet_id);
  }
}

void EntityManager::requestLaneChange(std::string name, Direction direction)
{
  if (getEntityStatusCoordinate(name) == CoordinateFrameTypes::WORLD) {
    return;
  }
  auto status = getEntityStatus(name, CoordinateFrameTypes::LANE);
  if (!status) {
    return;
  }
  if (direction == Direction::LEFT) {
    auto target = hdmap_utils_ptr_->getLaneChangeableLenletId(status->lanelet_id, "left");
    if (target) {
      requestLaneChange(name, target.get());
      return;
    }
    return;
  }
  if (direction == Direction::RIGHT) {
    auto target = hdmap_utils_ptr_->getLaneChangeableLenletId(status->lanelet_id, "right");
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
  if (getEntityStatusCoordinate(from) == CoordinateFrameTypes::LANE &&
    getEntityStatusCoordinate(to) == CoordinateFrameTypes::LANE)
  {
    auto from_status = getEntityStatus(from, CoordinateFrameTypes::LANE);
    auto to_status = getEntityStatus(to, CoordinateFrameTypes::LANE);
    if (from_status && to_status) {
      auto dist = hdmap_utils_ptr_->getLongitudinalDistance(from_status->lanelet_id,
          from_status->s,
          to_status->lanelet_id, to_status->s);
      if (!dist) {
        return boost::none;
      } else {
        if (dist <= max_distance) {
          return dist.get();
        }
        return boost::none;
      }
    }
  }
  return boost::none;
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

const boost::optional<VehicleParameters> EntityManager::getVehicleParameters(std::string name) const
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
  if (getEntityStatusCoordinate(name) == CoordinateFrameTypes::WORLD) {
    return false;
  }
  double l = hdmap_utils_ptr_->getLaneletLength(lanelet_id);
  auto status = getEntityStatus(name, CoordinateFrameTypes::LANE);
  if (!status) {
    return false;
  }
  if (status->lanelet_id == lanelet_id) {
    std::cout << __FILE__ << "," << __LINE__ << std::endl;
    return true;
  } else {
    auto dist0 = hdmap_utils_ptr_->getLongitudinalDistance(lanelet_id, l, status->lanelet_id,
        status->s);
    auto dist1 = hdmap_utils_ptr_->getLongitudinalDistance(status->lanelet_id, status->s,
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

bool EntityManager::setEntityStatus(std::string name, EntityStatus status)
{
  auto it = entities_.find(name);
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

const CoordinateFrameTypes & EntityManager::getEntityStatusCoordinate(std::string name) const
{
  auto it = entities_.find(name);
  if (it->second.type() == typeid(VehicleEntity)) {
    return boost::any_cast<const VehicleEntity &>(it->second).getStatusCoordinateFrameType();
  }
  if (it->second.type() == typeid(EgoEntity)) {
    return boost::any_cast<const EgoEntity &>(it->second).getStatusCoordinateFrameType();
  }
  throw simulation_api::SimulationRuntimeError(
          "failed to get entity status coordinate, entity name does not match, entity_name : " +
          name);
}

const boost::optional<EntityStatus> EntityManager::getEntityStatus(
  std::string name,
  CoordinateFrameTypes coordinate) const
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    return boost::none;
  }
  if (it->second.type() == typeid(VehicleEntity)) {
    return boost::any_cast<const VehicleEntity &>(it->second).getStatus(coordinate);
  }
  if (it->second.type() == typeid(EgoEntity)) {
    return boost::any_cast<const EgoEntity &>(it->second).getStatus(coordinate);
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    return boost::any_cast<const PedestrianEntity &>(it->second).getStatus(coordinate);
  }
  return boost::none;
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

void EntityManager::update(double current_time, double step_time)
{
  if (getNumberOfEgo() >= 2) {
    throw SimulationRuntimeError("multi ego simulation does not support yet.");
  }
  setVerbose(verbose_);
  auto type_list = getEntityTypeList();
  std::unordered_map<std::string, EntityStatus> all_status;
  for (auto it = entities_.begin(); it != entities_.end(); it++) {
    if (it->second.type() == typeid(VehicleEntity)) {
      boost::any_cast<VehicleEntity &>(it->second).setEntityTypeList(type_list);
      boost::any_cast<VehicleEntity &>(it->second).onUpdate(current_time, step_time);
      if (boost::any_cast<VehicleEntity &>(it->second).statusSetted()) {
        auto status = boost::any_cast<VehicleEntity &>(it->second).getStatus(
          boost::any_cast<VehicleEntity &>(it->second).getStatusCoordinateFrameType());
        all_status[boost::any_cast<VehicleEntity &>(it->second).name] = status;
      }
    }
    if (it->second.type() == typeid(EgoEntity)) {
      boost::any_cast<EgoEntity &>(it->second).setEntityTypeList(type_list);
      boost::any_cast<EgoEntity &>(it->second).onUpdate(current_time, step_time);
      if (boost::any_cast<EgoEntity &>(it->second).statusSetted()) {
        auto status = boost::any_cast<EgoEntity &>(it->second).getStatus(
          boost::any_cast<EgoEntity &>(it->second).getStatusCoordinateFrameType());
        all_status[boost::any_cast<EgoEntity &>(it->second).name] = status;
      }
    }
    if (it->second.type() == typeid(PedestrianEntity)) {
      boost::any_cast<PedestrianEntity &>(it->second).setEntityTypeList(type_list);
      boost::any_cast<PedestrianEntity &>(it->second).onUpdate(current_time, step_time);
      if (boost::any_cast<PedestrianEntity &>(it->second).statusSetted()) {
        auto status = boost::any_cast<PedestrianEntity &>(it->second).getStatus(
          boost::any_cast<PedestrianEntity &>(it->second).getStatusCoordinateFrameType());
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
  openscenario_msgs::msg::EntityStatusArray status_array_msg;
  for (const auto & status : all_status) {
    auto status_msg = status.second.toRosMsg();
    status_msg.name = status.first;
    status_msg.bounding_box = getBoundingBox(status.first);
    status_msg.current_action = getCurrentAction(status.first);
    switch (getEntityType(status.first)) {
      case EntityType::EGO:
        status_msg.type = status_msg.EGO;
        break;
      case EntityType::VEHICLE:
        status_msg.type = status_msg.VEHICLE;
        break;
      case EntityType::PEDESTRIAN:
        status_msg.type = status_msg.PEDESTRIAN;
        break;
    }
    status_array_msg.status.emplace_back(status_msg);
  }
  entity_status_array_pub_ptr_->publish(status_array_msg);
  // entity_marker_pub_ptr_->publish(generateMarker());
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
  std::string name, geometry_msgs::msg::Pose target_pose,
  double tolerance) const
{
  auto status = getEntityStatus(name);
  if (!status) {
    throw simulation_api::SimulationRuntimeError(
            "error occurs while getting entity stauts, target entity : " + name);
  }
  auto pose = status->pose;
  double dist =
    std::sqrt(std::pow(pose.position.x - target_pose.position.x,
      2) + std::pow(pose.position.y - target_pose.position.y, 2) +
      std::pow(pose.position.z - target_pose.position.z, 2));
  if (dist < tolerance) {
    return true;
  }
  return false;
}

bool EntityManager::reachPosition(
  std::string name, std::int64_t lanelet_id, double s, double offset,
  double tolerance) const
{
  geometry_msgs::msg::Vector3 rpy;
  geometry_msgs::msg::Twist twist;
  geometry_msgs::msg::Accel accel;
  auto target_status = simulation_api::entity::EntityStatus(0, lanelet_id, s, offset, rpy,
      twist, accel);
  auto target_pose = hdmap_utils_ptr_->toMapPose(target_status);
  if (!target_pose) {
    throw simulation_api::SimulationRuntimeError("failed to transform into map frame");
  }
  return reachPosition(name, target_pose->pose, tolerance);
}

void EntityManager::broadcastBaseLinkTransform()
{
  std::vector<std::string> names = getEntityNames();
  for (const auto & name : names) {
    if (getEntityType(name) == EntityType::EGO) {
      auto status = getEntityStatus(name);
      if (status) {
        auto pose = hdmap_utils_ptr_->toMapPose(status.get());
        if (pose) {
          pose->header.stamp = clock_ptr_->now();
          pose->header.frame_id = "base_link";
          broadcastTransform(pose.get());
        }
      }
      return;
    }
  }
}

void EntityManager::broadcastEntityTransform()
{
  std::vector<std::string> names = getEntityNames();
  for (auto it = names.begin(); it != names.end(); it++) {
    if (entityStatusSetted(*it)) {
      auto status = getEntityStatus(*it);
      if (status) {
        auto pose = hdmap_utils_ptr_->toMapPose(status.get());
        if (pose) {
          pose->header.stamp = clock_ptr_->now();
          pose->header.frame_id = *it;
          broadcastTransform(pose.get());
        }
      }
    }
  }
  broadcastBaseLinkTransform();
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

EntityType EntityManager::getEntityType(std::string name) const
{
  auto it = entities_.find(name);
  if (it->second.type() == typeid(VehicleEntity)) {
    return EntityType::VEHICLE;
  }
  if (it->second.type() == typeid(EgoEntity)) {
    return EntityType::EGO;
  }
  if (it->second.type() == typeid(PedestrianEntity)) {
    return EntityType::PEDESTRIAN;
  }
  throw simulation_api::SimulationRuntimeError("entity " + name + "does not exist");
}

const std::unordered_map<std::string, EntityType> EntityManager::getEntityTypeList() const
{
  std::unordered_map<std::string, EntityType> ret;
  for (auto it = entities_.begin(); it != entities_.end(); it++) {
    if (it->second.type() == typeid(VehicleEntity)) {
      ret[it->first] = EntityType::VEHICLE;
    }
    if (it->second.type() == typeid(EgoEntity)) {
      ret[it->first] = EntityType::EGO;
    }
    if (it->second.type() == typeid(PedestrianEntity)) {
      ret[it->first] = EntityType::PEDESTRIAN;
    }
  }
  return ret;
}
}  // namespace entity
}  // namespace simulation_api
