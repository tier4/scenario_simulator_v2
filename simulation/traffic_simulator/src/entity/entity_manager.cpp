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

#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <scenario_simulator_exception/exception.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <traffic_simulator/entity/entity_manager.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/math/bounding_box.hpp>
#include <traffic_simulator/math/collision.hpp>
#include <traffic_simulator/math/transfrom.hpp>
#include <unordered_map>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
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
    THROW_SEMANTIC_ERROR("entity : ", name0, " status does not exist.");
  }
  auto status1 = getEntityStatus(name1);
  if (!status1) {
    THROW_SEMANTIC_ERROR("failed to calculate map pose : " + name1);
  }
  auto bbox0 = getBoundingBox(name0);
  auto bbox1 = getBoundingBox(name1);
  return traffic_simulator::math::checkCollision2D(status0->pose, bbox0, status1->pose, bbox1);
}

bool EntityManager::despawnEntity(const std::string & name)
{
  return entityExists(name) && entities_.erase(name);
}

bool EntityManager::entityExists(const std::string & name)
{
  return entities_.find(name) != std::end(entities_);
}

bool EntityManager::entityStatusSet(const std::string & name) const
{
  return entities_.at(name)->statusSet();
}

auto EntityManager::getBoundingBoxDistance(const std::string & from, const std::string & to)
  -> boost::optional<double>
{
  const auto bbox0 = getBoundingBox(from);
  const auto pose0 = getMapPose(from);
  const auto bbox1 = getBoundingBox(to);
  const auto pose1 = getMapPose(to);
  return math::getPolygonDistance(pose0, bbox0, pose1, bbox1);
}

auto EntityManager::getConflictingEntityOnRouteLanelets(
  const std::string & name, const double horizon) -> std::vector<std::int64_t>
{
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    THROW_SEMANTIC_ERROR("entity : ", name, " does not exist");
  }
  const auto route = getRouteLanelets(name, horizon);
  return hdmap_utils_ptr_->getConflictingCrosswalkIds(route);
}

auto EntityManager::getCurrentTime() const noexcept -> double { return current_time_; }

auto EntityManager::getDistanceToCrosswalk(
  const std::string & name, const std::int64_t target_crosswalk_id) -> boost::optional<double>
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

auto EntityManager::getDistanceToStopLine(
  const std::string & name, const std::int64_t target_stop_line_id) -> boost::optional<double>
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

auto EntityManager::getEntityNames() const -> const std::vector<std::string>
{
  std::vector<std::string> names{};
  for (const auto & each : entities_) {
    names.push_back(each.first);
  }
  return names;
}

auto EntityManager::getEntityStatus(const std::string & name) const
  -> const boost::optional<openscenario_msgs::msg::EntityStatus>
{
  openscenario_msgs::msg::EntityStatus status_msg;
  auto it = entities_.find(name);
  if (it == entities_.end()) {
    THROW_SEMANTIC_ERROR("entity : ", name, " does not exist.");
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

auto EntityManager::getEntityTypeList() const
  -> const std::unordered_map<std::string, openscenario_msgs::msg::EntityType>
{
  std::unordered_map<std::string, openscenario_msgs::msg::EntityType> ret;
  for (auto it = entities_.begin(); it != entities_.end(); it++) {
    ret.emplace(it->first, it->second->getEntityType());
  }
  return ret;
}

auto EntityManager::getHdmapUtils() -> const std::shared_ptr<hdmap_utils::HdMapUtils> &
{
  return hdmap_utils_ptr_;
}

auto EntityManager::getLaneletPose(const std::string & name)
  -> boost::optional<openscenario_msgs::msg::LaneletPose>
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

auto EntityManager::getLongitudinalDistance(
  const std::string & from, const std::string & to, const double max_distance)
  -> boost::optional<double>
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

/**
 * @brief
 *
 * @param from from entity name
 * @param to to entity name
 * @retval boost::none bounding box is intersects
 * @retval 0 <= distance between two bounding box
 */
geometry_msgs::msg::Pose EntityManager::getMapPose(const std::string & entity_name)
{
  const auto status = getEntityStatus(entity_name);
  if (!status) {
    THROW_SEMANTIC_ERROR("entity : ", entity_name, " status is empty");
  }
  return status->pose;
}

geometry_msgs::msg::Pose EntityManager::getMapPose(
  const std::string & reference_entity_name, const geometry_msgs::msg::Pose & relative_pose)
{
  const auto ref_status = getEntityStatus(reference_entity_name);
  if (!ref_status) {
    THROW_SEMANTIC_ERROR("entity : ", reference_entity_name, " status is empty");
  }
  tf2::Transform ref_transform, relative_transform;
  tf2::fromMsg(ref_status->pose, ref_transform);
  tf2::fromMsg(relative_pose, relative_transform);
  geometry_msgs::msg::Pose ret;
  tf2::toMsg(ref_transform * relative_transform, ret);
  return ret;
}

auto EntityManager::getNumberOfEgo() const -> std::size_t
{
  return std::count_if(std::begin(entities_), std::end(entities_), [this](const auto & each) {
    return isEgo(each.first);
  });
}

const std::string EntityManager::getEgoName() const
{
  const auto names = getEntityNames();
  for (const auto name : names) {
    if (isEgo(name)) {
      return name;
    }
  }
  THROW_SEMANTIC_ERROR(
    "const std::string EntityManager::getEgoName(const std::string & name) function was called, "
    "but ego vehicle does not exist");
}

auto EntityManager::getObstacle(const std::string & name)
  -> boost::optional<openscenario_msgs::msg::Obstacle>
{
  if (current_time_ < 0) {
    return boost::none;
  }
  return entities_.at(name)->getObstacle();
}

auto EntityManager::getRelativePose(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to) const
  -> geometry_msgs::msg::Pose
{
  return traffic_simulator::math::getRelativePose(from, to);
}

auto EntityManager::getRelativePose(const geometry_msgs::msg::Pose & from, const std::string & to)
  -> geometry_msgs::msg::Pose
{
  const auto to_status = getEntityStatus(to);
  if (!to_status) {
    THROW_SEMANTIC_ERROR("entity : " + to + " status is empty");
  }
  return getRelativePose(from, to_status->pose);
}

auto EntityManager::getRelativePose(const std::string & from, const geometry_msgs::msg::Pose & to)
  -> geometry_msgs::msg::Pose
{
  const auto from_status = getEntityStatus(from);
  if (!from_status) {
    THROW_SEMANTIC_ERROR("entity : " + from + " status is empty");
  }
  return getRelativePose(from_status->pose, to);
}

auto EntityManager::getRelativePose(const std::string & from, const std::string & to)
  -> geometry_msgs::msg::Pose
{
  const auto from_status = getEntityStatus(from);
  const auto to_status = getEntityStatus(to);
  if (!from_status) {
    THROW_SEMANTIC_ERROR("entity : " + from + " status is empty");
  }
  if (!to_status) {
    THROW_SEMANTIC_ERROR("entity : " + to + " status is empty");
  }
  return getRelativePose(from_status->pose, to_status->pose);
}

auto EntityManager::getStepTime() const noexcept -> double { return step_time_; }

auto EntityManager::getSValueInRoute(
  const std::string & name, const std::vector<std::int64_t> & route) -> boost::optional<double>
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

auto EntityManager::getWaypoints(const std::string & name) -> openscenario_msgs::msg::WaypointsArray
{
  if (current_time_ < 0) {
    return openscenario_msgs::msg::WaypointsArray();
  }
  return entities_.at(name)->getWaypoints();
}

bool EntityManager::isEgo(const std::string & name) const
{
  return getEntityType(name).type == openscenario_msgs::msg::EntityType::EGO;
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

bool EntityManager::isStopping(const std::string & name) const
{
  const auto status = getEntityStatus(name);
  if (!status) {
    THROW_SEMANTIC_ERROR("entity : " + name + " status is empty");
  }
  return std::fabs(status->action_status.twist.linear.x) < std::numeric_limits<double>::epsilon();
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
    THROW_SEMANTIC_ERROR("entity : " + name + " status is empty");
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

void EntityManager::requestLaneChange(const std::string & name, const Direction & direction)
{
  auto status = getEntityStatus(name);

  if (status) {
    if (direction == Direction::LEFT) {
      const auto target =
        hdmap_utils_ptr_->getLaneChangeableLaneletId(status->lanelet_pose.lanelet_id, "left");
      if (target) {
        requestLaneChange(name, target.get());
      }
    } else if (direction == Direction::RIGHT) {
      const auto target =
        hdmap_utils_ptr_->getLaneChangeableLaneletId(status->lanelet_pose.lanelet_id, "right");
      if (target) {
        requestLaneChange(name, target.get());
      }
    }
  }
}

void EntityManager::setTargetSpeed(const std::string & name, double target_speed, bool continuous)
{
  if (isEgo(name) && getCurrentTime() > 0) {
    THROW_SEMANTIC_ERROR("You cannot set target speed to the ego vehicle after starting scenario.");
  }
  return entities_.at(name)->setTargetSpeed(target_speed, continuous);
}

bool EntityManager::setEntityStatus(
  const std::string & name, openscenario_msgs::msg::EntityStatus status)
{
  status.name = name;  // XXX UGLY CODE
  if (isEgo(name) && getCurrentTime() > 0) {
    THROW_SEMANTIC_ERROR(
      "You cannot set entity status to the ego vehicle after starting scenario.");
  }
  return entities_.at(name)->setStatus(status);
}

void EntityManager::setVerbose(bool verbose)
{
  verbose_ = verbose;
  for (auto & entity : entities_) {
    entity.second->setVerbose(verbose);
  }
}

auto EntityManager::toMapPose(const openscenario_msgs::msg::LaneletPose & lanelet_pose) const
  -> const geometry_msgs::msg::Pose
{
  return hdmap_utils_ptr_->toMapPose(lanelet_pose).pose;
}

openscenario_msgs::msg::EntityStatus EntityManager::updateNpcLogic(
  const std::string & name,
  const std::unordered_map<std::string, openscenario_msgs::msg::EntityType> & type_list)
{
  if (verbose_) {
    std::cout << "update " << name << " behavior" << std::endl;
  }
  entities_[name]->setEntityTypeList(type_list);
  entities_[name]->onUpdate(current_time_, step_time_);
  if (entities_[name]->statusSet()) {
    return entities_[name]->getStatus();
  }
  THROW_SIMULATION_ERROR("status of entity ", name, "is empty");
}

void EntityManager::update(const double current_time, const double step_time)
{
  std::chrono::system_clock::time_point start, end;
  start = std::chrono::system_clock::now();
  step_time_ = step_time;
  current_time_ = current_time;
  if (verbose_) {
    std::cout << "-------------------------- UPDATE --------------------------" << std::endl;
    std::cout << "current_time : " << current_time_ << std::endl;
  }
  if (getNumberOfEgo() >= 2) {
    THROW_SEMANTIC_ERROR("multi ego simulation does not support yet");
  }
  if (current_time_ >= 0) {
    traffic_light_manager_ptr_->update(step_time_);
  }
  setVerbose(verbose_);
  auto type_list = getEntityTypeList();
  std::unordered_map<std::string, openscenario_msgs::msg::EntityStatus> all_status;
  const std::vector<std::string> entity_names = getEntityNames();
  for (const auto & entity_name : entity_names) {
    if (entities_[entity_name]->statusSet()) {
      const auto status = updateNpcLogic(entity_name, type_list);
      all_status.emplace(entity_name, status);
    }
  }
  for (auto it = entities_.begin(); it != entities_.end(); it++) {
    it->second->setOtherStatus(all_status);
  }
  auto entity_type_list = getEntityTypeList();
  openscenario_msgs::msg::EntityStatusWithTrajectoryArray status_array_msg;
  for (const auto & status : all_status) {
    // std::cout << "calculating " << status.first << " status" << std::endl;
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
    std::cout << "elapsed " << elapsed / 1000 << " seconds in update function." << std::endl;
  }
}

void EntityManager::updateHdmapMarker()
{
  MarkerArray markers;
  const auto stamp = clock_ptr_->now();
  for (const auto & marker_raw : markers_raw_.markers) {
    visualization_msgs::msg::Marker marker = marker_raw;
    marker.header.stamp = stamp;
    markers.markers.emplace_back(marker);
  }
  lanelet_marker_pub_ptr_->publish(markers);
}
}  // namespace entity
}  // namespace traffic_simulator
