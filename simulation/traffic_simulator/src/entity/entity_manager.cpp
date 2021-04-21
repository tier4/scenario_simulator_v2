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

#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <traffic_simulator/entity/entity_manager.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/math/bounding_box.hpp>
#include <traffic_simulator/math/collision.hpp>
#include <unordered_map>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
bool EntityManager::isStopping(const std::string & name) const
{
  const auto status = getEntityStatus(name);
  if (!status) {
    throw traffic_simulator::SimulationRuntimeError("failed to get entity : " + name + " status");
  }
  return std::fabs(status->action_status.twist.linear.x) < std::numeric_limits<double>::epsilon();
}

void EntityManager::setDriverModel(
  const std::string & name, const openscenario_msgs::msg::DriverModel & model)
{
  entities_.at(name)->setDriverModel(model);
}

const geometry_msgs::msg::Pose EntityManager::toMapPose(
  const openscenario_msgs::msg::LaneletPose & lanelet_pose) const
{
  return hdmap_utils_ptr_->toMapPose(lanelet_pose).pose;
}

const std::shared_ptr<hdmap_utils::HdMapUtils> EntityManager::getHdmapUtils()
{
  return hdmap_utils_ptr_;
}

void EntityManager::setVerbose(bool verbose)
{
  verbose_ = verbose;
  for (auto & entity : entities_) {
    entity.second->setVerbose(verbose);
  }
}

bool EntityManager::isEgo(const std::string & name) const
{
  return getEntityType(name).type == openscenario_msgs::msg::EntityType::EGO;
}

std::size_t EntityManager::getNumberOfEgo() const
{
  // std::size_t count = 0;
  // for (auto & entity : entities_) {
  //   if (isEgo(entity.first)) {
  //     ++count;
  //   }
  // }
  // return count;

  return std::count_if(std::begin(entities_), std::end(entities_), [this](const auto & each) {
    return isEgo(each.first);
  });
}

boost::optional<openscenario_msgs::msg::LaneletPose> EntityManager::getLaneletPose(
  const std::string & name)
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
  const std::string & name, const std::int64_t target_crosswalk_id)
{
  const auto it = entities_.find(name);
  if (it == entities_.end()) {
    return boost::none;
  }
  if (getWaypoints(name).waypoints.empty()) {
    return boost::none;
  }
  traffic_simulator::math::CatmullRomSpline spline(getWaypoints(name).waypoints);
  auto polygon = hdmap_utils_ptr_->getLaneletPolygon(target_crosswalk_id);
  return spline.getCollisionPointIn2D(polygon);
}

boost::optional<double> EntityManager::getSValueInRoute(
  const std::string & name, const std::vector<std::int64_t> & route)
{
  const auto it = entities_.find(name);
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
  const std::string & name, const std::int64_t target_stop_line_id)
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    return boost::none;
  }
  if (getWaypoints(name).waypoints.empty()) {
    return boost::none;
  }
  traffic_simulator::math::CatmullRomSpline spline(getWaypoints(name).waypoints);
  auto polygon = hdmap_utils_ptr_->getStopLinePolygon(target_stop_line_id);
  return spline.getCollisionPointIn2D(polygon);
}

void EntityManager::requestAssignRoute(
  const std::string & name, const std::vector<openscenario_msgs::msg::LaneletPose> & waypoints)
{
  entities_.at(name)->requestAssignRoute(waypoints);
}

void EntityManager::requestAcquirePosition(
  const std::string & name, const openscenario_msgs::msg::LaneletPose & lanelet_pose)
{
  entities_.at(name)->requestAcquirePosition(lanelet_pose);
}

void EntityManager::requestLaneChange(const std::string & name, const std::int64_t to_lanelet_id)
{
  entities_.at(name)->requestLaneChange(to_lanelet_id);
}

void EntityManager::requestWalkStraight(const std::string & name)
{
  entities_.at(name)->requestWalkStraight();
}

void EntityManager::requestLaneChange(const std::string & name, const Direction & direction)
{
  auto status = getEntityStatus(name);

  if (status) {
    if (direction == Direction::LEFT) {
      const auto target =
        hdmap_utils_ptr_->getLaneChangeableLenletId(status->lanelet_pose.lanelet_id, "left");
      if (target) {
        requestLaneChange(name, target.get());
      }
    } else if (direction == Direction::RIGHT) {
      const auto target =
        hdmap_utils_ptr_->getLaneChangeableLenletId(status->lanelet_pose.lanelet_id, "right");
      if (target) {
        requestLaneChange(name, target.get());
      }
    }
  }
}

/**
 * @brief
 *
 * @param from from entity name
 * @param to to entity name
 * @retval boost::none bounding box is intersects
 * @retval 0 <= distance between two bounding box
 */
boost::optional<double> EntityManager::getBoundingBoxDistance(
  const std::string & from, const std::string & to)
{
  const auto bbox0 = getBoundingBox(from);
  const auto pose0 = getMapPose(from);
  const auto bbox1 = getBoundingBox(to);
  const auto pose1 = getMapPose(to);
  return math::getPolygonDistance(pose0, bbox0, pose1, bbox1);
}

boost::optional<double> EntityManager::getLongitudinalDistance(
  const std::string & from, const std::string & to, const double max_distance)
{
  if (!entityStatusSet(from) || !entityStatusSet(to)) {
    return boost::none;
  } else {
    const auto from_status = getEntityStatus(from);
    const auto to_status = getEntityStatus(to);
    if (from_status && to_status) {
      const auto distance = hdmap_utils_ptr_->getLongitudinalDistance(
        from_status->lanelet_pose.lanelet_id, from_status->lanelet_pose.s,
        to_status->lanelet_pose.lanelet_id, to_status->lanelet_pose.s);
      return (distance && distance <= max_distance) ? distance : boost::none;
    } else {
      return boost::none;
    }
  }
}

geometry_msgs::msg::Pose EntityManager::getMapPose(const std::string & entity_name)
{
  const auto status = getEntityStatus(entity_name);
  if (!status) {
    throw traffic_simulator::SimulationRuntimeError(
      "failed to get status of " + entity_name + " entity in getMapPose");
  }
  return status->pose;
}

geometry_msgs::msg::Pose EntityManager::getMapPose(
  const std::string & reference_entity_name, const geometry_msgs::msg::Pose & relative_pose)
{
  const auto ref_status = getEntityStatus(reference_entity_name);
  if (!ref_status) {
    throw traffic_simulator::SimulationRuntimeError(
      "failed to get status of " + reference_entity_name + " entity in getMapPose");
  }
  tf2::Transform ref_transfrom, relative_transform;
  tf2::fromMsg(ref_status->pose, ref_transfrom);
  tf2::fromMsg(relative_pose, relative_transform);
  geometry_msgs::msg::Pose ret;
  tf2::toMsg(ref_transfrom * relative_transform, ret);
  return ret;
}

geometry_msgs::msg::Pose EntityManager::getRelativePose(
  const std::string & from, const std::string & to)
{
  const auto from_status = getEntityStatus(from);
  const auto to_status = getEntityStatus(to);
  if (!from_status) {
    throw traffic_simulator::SimulationRuntimeError(
      "failed to get status of " + from + " entity in getRelativePose");
  }
  if (!to_status) {
    throw traffic_simulator::SimulationRuntimeError(
      "failed to get status of " + to + " entity in getRelativePose");
  }
  return getRelativePose(from_status->pose, to_status->pose);
}

geometry_msgs::msg::Pose EntityManager::getRelativePose(
  const std::string & from, const geometry_msgs::msg::Pose & to)
{
  const auto from_status = getEntityStatus(from);
  if (!from_status) {
    throw traffic_simulator::SimulationRuntimeError(
      "failed to get status of " + from + " entity in getRelativePose");
  }
  return getRelativePose(from_status->pose, to);
}

geometry_msgs::msg::Pose EntityManager::getRelativePose(
  const geometry_msgs::msg::Pose & from, const std::string & to)
{
  const auto to_status = getEntityStatus(to);
  if (!to_status) {
    throw traffic_simulator::SimulationRuntimeError(
      "failed to get status of " + to + " entity in getRelativePose");
  }
  return getRelativePose(from, to_status->pose);
}

geometry_msgs::msg::Pose EntityManager::getRelativePose(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to) const
{
  geometry_msgs::msg::Transform from_translation;
  {
    from_translation.translation.x = from.position.x;
    from_translation.translation.y = from.position.y;
    from_translation.translation.z = from.position.z;
    from_translation.rotation = from.orientation;
  }

  tf2::Transform from_tf;
  {
    tf2::fromMsg(from_translation, from_tf);
  }

  geometry_msgs::msg::Transform to_translation;
  {
    to_translation.translation.x = to.position.x;
    to_translation.translation.y = to.position.y;
    to_translation.translation.z = to.position.z;
    to_translation.rotation = to.orientation;
  }

  tf2::Transform to_tf;
  {
    tf2::fromMsg(to_translation, to_tf);
  }

  tf2::Transform tf_delta = from_tf.inverse() * to_tf;

  geometry_msgs::msg::Pose ret;
  {
    tf2::toMsg(tf_delta, ret);
  }

  return ret;
}

const boost::optional<openscenario_msgs::msg::VehicleParameters>
EntityManager::getVehicleParameters(const std::string & name) const
{
  return entities_.at(name)->getVehicleParameters();
}

bool EntityManager::isInLanelet(
  const std::string & name, const std::int64_t lanelet_id, const double tolerance)
{
  if (!entityStatusSet(name)) {
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
      lanelet_id, l, status->lanelet_pose.lanelet_id, status->lanelet_pose.s);
    auto dist1 = hdmap_utils_ptr_->getLongitudinalDistance(
      status->lanelet_pose.lanelet_id, status->lanelet_pose.s, lanelet_id, 0);
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
  std::vector<std::string> names{};
  for (const auto & each : entities_) {
    names.push_back(each.first);
  }
  return names;
}

bool EntityManager::setEntityStatus(
  const std::string & name, openscenario_msgs::msg::EntityStatus status)
{
  status.name = name;  // XXX UGLY CODE
  return entities_.at(name)->setStatus(status);
}

const boost::optional<openscenario_msgs::msg::EntityStatus> EntityManager::getEntityStatus(
  const std::string & name) const
{
  openscenario_msgs::msg::EntityStatus status_msg;
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    return boost::none;
  }
  status_msg = it->second->getStatus();
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

bool EntityManager::checkCollision(const std::string & name0, const std::string & name1)
{
  if (name0 == name1) {
    return false;
  }
  if (!entityStatusSet(name0)) {
    return false;
  }
  if (!entityStatusSet(name1)) {
    return false;
  }
  auto status0 = getEntityStatus(name0);
  if (!status0) {
    throw traffic_simulator::SimulationRuntimeError("failed to calculate map pose : " + name0);
    return false;
  }
  auto status1 = getEntityStatus(name1);
  if (!status1) {
    throw traffic_simulator::SimulationRuntimeError("failed to calculate map pose : " + name1);
  }
  auto bbox0 = getBoundingBox(name0);
  auto bbox1 = getBoundingBox(name1);
  return traffic_simulator::math::checkCollision2D(status0->pose, bbox0, status1->pose, bbox1);
}

const openscenario_msgs::msg::BoundingBox EntityManager::getBoundingBox(
  const std::string & name) const
{
  return entities_.at(name)->getBoundingBox();
}

boost::optional<openscenario_msgs::msg::Obstacle> EntityManager::getObstacle(
  const std::string & name)
{
  if (current_time_ < 0) {
    return boost::none;
  }
  return entities_.at(name)->getObstacle();
}

openscenario_msgs::msg::WaypointsArray EntityManager::getWaypoints(const std::string & name)
{
  return entities_.at(name)->getWaypoints();
}

boost::optional<double> EntityManager::getLinearJerk(const std::string & name) const
{
  return entities_.at(name)->getLinearJerk();
}

bool EntityManager::entityStatusSet(const std::string & name) const
{
  return entities_.at(name)->statusSet();
}

void EntityManager::setTargetSpeed(
  const std::string & name, const double target_speed, const bool continuous)
{
  return entities_.at(name)->setTargetSpeed(target_speed, continuous);
}

std::vector<std::int64_t> EntityManager::getRouteLanelets(
  const std::string & name, const double horizon)
{
  return entities_.at(name)->getRouteLanelets(horizon);
}

std::vector<std::int64_t> EntityManager::getConflictingEntityOnRouteLanelets(
  const std::string & name, const double horizon)
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    throw SimulationRuntimeError("entity " + name + " does not exist");
  }
  const auto route = getRouteLanelets(name, horizon);
  return hdmap_utils_ptr_->getConflictingCrosswalkIds(route);
}

double EntityManager::getStepTime() const noexcept { return step_time_; }

double EntityManager::getCurrentTime() const noexcept { return current_time_; }

void EntityManager::update(const double current_time, const double step_time)
{
  std::chrono::system_clock::time_point start, end;
  start = std::chrono::system_clock::now();
  step_time_ = step_time;
  current_time_ = current_time;
  if (current_time_ >= 0) {
    traffic_light_manager_ptr_->update(step_time_);
  }
  if (verbose_) {
    std::cout << "-------------------------- UPDATE --------------------------" << std::endl;
    std::cout << "current_time : " << current_time_ << std::endl;
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
    it->second->setEntityTypeList(type_list);
    it->second->onUpdate(current_time, step_time);
    if (it->second->statusSet()) {
      auto status = it->second->getStatus();
      all_status.emplace(it->first, status);
    }
  }
  for (auto it = entities_.begin(); it != entities_.end(); it++) {
    it->second->setOtherStatus(all_status);
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

void EntityManager::broadcastTransform(
  const geometry_msgs::msg::PoseStamped & pose, const bool static_transform)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  {
    transform_stamped.header.stamp = pose.header.stamp;
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = pose.header.frame_id;
    transform_stamped.transform.translation.x = pose.pose.position.x;
    transform_stamped.transform.translation.y = pose.pose.position.y;
    transform_stamped.transform.translation.z = pose.pose.position.z;
    transform_stamped.transform.rotation = pose.pose.orientation;
  }

  if (static_transform) {
    broadcaster_.sendTransform(transform_stamped);
  } else {
    base_link_broadcaster_.sendTransform(transform_stamped);
  }
}

bool EntityManager::reachPosition(
  const std::string & name, const std::string & target_name, const double tolerance) const
{
  const auto status = getEntityStatus(target_name);
  return status && reachPosition(name, status->pose, tolerance);
}

bool EntityManager::reachPosition(
  const std::string & name, const geometry_msgs::msg::Pose & target_pose,
  const double tolerance) const
{
  const auto status = getEntityStatus(name);
  if (!status) {
    throw traffic_simulator::SimulationRuntimeError(
      "error occurs while getting entity stauts, target entity : " + name);
  }

  const auto pose = status->pose;

  const double distance = std::sqrt(
    std::pow(pose.position.x - target_pose.position.x, 2) +
    std::pow(pose.position.y - target_pose.position.y, 2) +
    std::pow(pose.position.z - target_pose.position.z, 2));

  return distance < tolerance;
}

bool EntityManager::reachPosition(
  const std::string & name, const std::int64_t lanelet_id, const double s, const double offset,
  const double tolerance) const
{
  openscenario_msgs::msg::LaneletPose lanelet_pose;
  {
    lanelet_pose.lanelet_id = lanelet_id;
    lanelet_pose.s = s;
    lanelet_pose.offset = offset;
  }

  const auto target_pose = hdmap_utils_ptr_->toMapPose(lanelet_pose);

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
    if (entityStatusSet(*it)) {
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

const boost::optional<double> EntityManager::getStandStillDuration(const std::string & name) const
{
  return entities_.at(name)->getStandStillDuration();
}

const std::string EntityManager::getCurrentAction(const std::string & name) const
{
  return entities_.at(name)->getCurrentAction();
}

openscenario_msgs::msg::EntityType EntityManager::getEntityType(const std::string & name) const
{
  return entities_.at(name)->getEntityType();
}

const std::unordered_map<std::string, openscenario_msgs::msg::EntityType>
EntityManager::getEntityTypeList() const
{
  std::unordered_map<std::string, openscenario_msgs::msg::EntityType> ret;
  for (auto it = entities_.begin(); it != entities_.end(); it++) {
    ret.emplace(it->first, it->second->getEntityType());
  }
  return ret;
}
}  // namespace entity
}  // namespace traffic_simulator
