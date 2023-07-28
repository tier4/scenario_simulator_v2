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

#include <cstdint>
#include <geometry/bounding_box.hpp>
#include <geometry/distance.hpp>
#include <geometry/intersection/collision.hpp>
#include <geometry/transform.hpp>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <scenario_simulator_exception/exception.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <traffic_simulator/entity/entity_manager.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/helper/stop_watch.hpp>
#include <unordered_map>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
void EntityManager::broadcastEntityTransform()
{
  std::vector<std::string> names = getEntityNames();
  for (const auto & name : names) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose = getMapPose(name);
    pose.header.stamp = clock_ptr_->now();
    pose.header.frame_id = name;
    broadcastTransform(pose);
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

bool EntityManager::checkCollision(const std::string & name0, const std::string & name1)
{
  return name0 != name1 and
         math::geometry::checkCollision2D(
           getMapPose(name0), getBoundingBox(name0), getMapPose(name1), getBoundingBox(name1));
}

visualization_msgs::msg::MarkerArray EntityManager::makeDebugMarker() const
{
  visualization_msgs::msg::MarkerArray marker;
  for (const auto & entity : entities_) {
    entity.second->appendDebugMarker(marker);
  }
  return marker;
}

bool EntityManager::despawnEntity(const std::string & name)
{
  return entityExists(name) && entities_.erase(name);
}

bool EntityManager::entityExists(const std::string & name)
{
  return entities_.find(name) != std::end(entities_);
}

auto EntityManager::getBoundingBoxDistance(const std::string & from, const std::string & to)
  -> std::optional<double>
{
  return math::geometry::getPolygonDistance(
    getMapPose(from), getBoundingBox(from), getMapPose(to), getBoundingBox(to));
}

auto EntityManager::getCurrentTime() const noexcept -> double { return current_time_; }

auto EntityManager::getDistanceToCrosswalk(
  const std::string & name, const std::int64_t target_crosswalk_id) -> std::optional<double>
{
  if (entities_.find(name) == entities_.end()) {
    return std::nullopt;
  }
  if (getWaypoints(name).waypoints.empty()) {
    return std::nullopt;
  }
  math::geometry::CatmullRomSpline spline(getWaypoints(name).waypoints);
  auto polygon = hdmap_utils_ptr_->getLaneletPolygon(target_crosswalk_id);
  return spline.getCollisionPointIn2D(polygon);
}

auto EntityManager::getDistanceToStopLine(
  const std::string & name, const std::int64_t target_stop_line_id) -> std::optional<double>
{
  if (entities_.find(name) == entities_.end()) {
    return std::nullopt;
  }
  if (getWaypoints(name).waypoints.empty()) {
    return std::nullopt;
  }
  math::geometry::CatmullRomSpline spline(getWaypoints(name).waypoints);
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

auto EntityManager::getEntityStatus(const std::string & name) const -> CanonicalizedEntityStatus
{
  if (const auto iter = entities_.find(name); iter == entities_.end()) {
    THROW_SEMANTIC_ERROR("entity ", std::quoted(name), " does not exist.");
  } else {
    auto entity_status = static_cast<EntityStatus>(iter->second->getStatus());
    entity_status.action_status.current_action = getCurrentAction(name);
    entity_status.time = current_time_;
    return CanonicalizedEntityStatus(entity_status, hdmap_utils_ptr_);
  }
}

auto EntityManager::getEntityTypeList() const
  -> const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType>
{
  std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType> ret;
  for (auto && [name, entity] : entities_) {
    ret.emplace(name, getEntityType(name));
  }
  return ret;
}

auto EntityManager::getHdmapUtils() -> const std::shared_ptr<hdmap_utils::HdMapUtils> &
{
  return hdmap_utils_ptr_;
}

auto EntityManager::getLateralDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to) const
  -> std::optional<double>
{
  return hdmap_utils_ptr_->getLateralDistance(
    static_cast<LaneletPose>(from), static_cast<LaneletPose>(to));
}

auto EntityManager::getLateralDistance(
  const CanonicalizedLaneletPose & from, const std::string & to) const -> std::optional<double>
{
  if (const auto to_pose = getLaneletPose(to)) {
    return getLateralDistance(from, to_pose.value());
  }
  return std::nullopt;
}

auto EntityManager::getLateralDistance(
  const std::string & from, const CanonicalizedLaneletPose & to) const -> std::optional<double>
{
  if (const auto from_pose = getLaneletPose(from)) {
    return getLateralDistance(from_pose.value(), to);
  }
  return std::nullopt;
}

auto EntityManager::getLateralDistance(const std::string & from, const std::string & to) const
  -> std::optional<double>
{
  const auto from_pose = getLaneletPose(from);
  const auto to_pose = getLaneletPose(to);
  if (from_pose && to_pose) {
    return getLateralDistance(from_pose.value(), to_pose.value());
  }
  return std::nullopt;
}

auto EntityManager::getLateralDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  double matching_distance) const -> std::optional<double>
{
  if (
    std::abs(static_cast<LaneletPose>(from).offset) <= matching_distance &&
    std::abs(static_cast<LaneletPose>(to).offset) <= matching_distance) {
    return getLateralDistance(from, to);
  }
  return std::nullopt;
}

auto EntityManager::getLateralDistance(
  const CanonicalizedLaneletPose & from, const std::string & to, double matching_distance) const
  -> std::optional<double>
{
  if (const auto to_pose = getLaneletPose(to, matching_distance)) {
    return getLateralDistance(from, to_pose.value(), matching_distance);
  }
  return std::nullopt;
}

auto EntityManager::getLateralDistance(
  const std::string & from, const CanonicalizedLaneletPose & to, double matching_distance) const
  -> std::optional<double>
{
  if (const auto from_pose = getLaneletPose(from, matching_distance)) {
    return getLateralDistance(from_pose.value(), to, matching_distance);
  }
  return std::nullopt;
}

auto EntityManager::getLateralDistance(
  const std::string & from, const std::string & to, double matching_distance) const
  -> std::optional<double>
{
  const auto from_pose = getLaneletPose(from, matching_distance);
  const auto to_pose = getLaneletPose(to, matching_distance);
  if (from_pose && to_pose) {
    return getLateralDistance(from_pose.value(), to_pose.value(), matching_distance);
  }
  return std::nullopt;
}

auto EntityManager::getLongitudinalDistance(
  const CanonicalizedLaneletPose & from, const CanonicalizedLaneletPose & to,
  bool include_adjacent_lanelet, bool include_opposite_direction) -> std::optional<double>
{
  if (!include_adjacent_lanelet) {
    auto to_canonicalized = static_cast<LaneletPose>(to);
    if (to.hasAlternativeLaneletPose()) {
      if (
        const auto to_canonicalized_optional = to.getAlternativeLaneletPoseBaseOnShortestRouteFrom(
          static_cast<LaneletPose>(from), hdmap_utils_ptr_)) {
        to_canonicalized = to_canonicalized_optional.value();
      }
    }

    const auto forward_distance =
      hdmap_utils_ptr_->getLongitudinalDistance(static_cast<LaneletPose>(from), to_canonicalized);

    const auto backward_distance =
      hdmap_utils_ptr_->getLongitudinalDistance(to_canonicalized, static_cast<LaneletPose>(from));

    if (forward_distance && backward_distance) {
      return forward_distance.value() > backward_distance.value() ? -backward_distance.value()
                                                                  : forward_distance.value();
    } else if (forward_distance) {
      return forward_distance.value();
    } else if (backward_distance) {
      return -backward_distance.value();
    } else {
      return std::nullopt;
    }
  } else {
    /**
    * @brief hard coded parameter!! 5.0 is a matching distance of the toLaneletPoses function. 
    * A matching distance of about 1.5 lane widths is given as the matching distance to match the Entity present on the adjacent Lanelet.
    */
    auto from_poses = hdmap_utils_ptr_->toLaneletPoses(
      static_cast<geometry_msgs::msg::Pose>(from), static_cast<LaneletPose>(from).lanelet_id, 5.0,
      include_opposite_direction);
    from_poses.emplace_back(from);
    /**
    * @brief hard coded parameter!! 5.0 is a matching distance of the toLaneletPoses function. 
    * A matching distance of about 1.5 lane widths is given as the matching distance to match the Entity present on the adjacent Lanelet.
    */
    auto to_poses = hdmap_utils_ptr_->toLaneletPoses(
      static_cast<geometry_msgs::msg::Pose>(to), static_cast<LaneletPose>(to).lanelet_id, 5.0,
      include_opposite_direction);
    to_poses.emplace_back(to);
    std::vector<double> distances = {};
    for (const auto & from_pose : from_poses) {
      for (const auto & to_pose : to_poses) {
        if (
          const auto distance = getLongitudinalDistance(
            CanonicalizedLaneletPose(from_pose, hdmap_utils_ptr_),
            CanonicalizedLaneletPose(to_pose, hdmap_utils_ptr_), false,
            include_opposite_direction)) {
          distances.emplace_back(distance.value());
        }
      }
    }
    if (distances.empty()) {
      return std::nullopt;
    }
    std::sort(distances.begin(), distances.end(), [](double a, double b) {
      return std::abs(a) < std::abs(b);
    });
    return distances.front();
  }
}

auto EntityManager::getLongitudinalDistance(
  const CanonicalizedLaneletPose & from, const std::string & to, bool include_adjacent_lanelet,
  bool include_opposite_direction) -> std::optional<double>
{
  const auto to_pose = getLaneletPose(to);
  if (!laneMatchingSucceed(to) || !to_pose) {
    return std::nullopt;
  } else {
    return getLongitudinalDistance(
      from, to_pose.value(), include_adjacent_lanelet, include_opposite_direction);
  }
}

auto EntityManager::getLongitudinalDistance(
  const std::string & from, const CanonicalizedLaneletPose & to, bool include_adjacent_lanelet,
  bool include_opposite_direction) -> std::optional<double>
{
  const auto from_pose = getLaneletPose(from);
  if (!laneMatchingSucceed(from) || !from_pose) {
    return std::nullopt;
  } else {
    return getLongitudinalDistance(
      from_pose.value(), to, include_adjacent_lanelet, include_opposite_direction);
  }
}

auto EntityManager::getLongitudinalDistance(
  const std::string & from, const std::string & to, bool include_adjacent_lanelet,
  bool include_opposite_direction) -> std::optional<double>
{
  const auto from_lanelet_pose = getLaneletPose(from);
  const auto to_lanelet_pose = getLaneletPose(to);
  if (
    laneMatchingSucceed(from) and laneMatchingSucceed(to) and from_lanelet_pose and
    to_lanelet_pose) {
    return getLongitudinalDistance(
      from_lanelet_pose.value(), to_lanelet_pose.value(), include_adjacent_lanelet,
      include_opposite_direction);
  } else {
    return std::nullopt;
  }
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
  for (const auto & name : names) {
    if (isEgo(name)) {
      return name;
    }
  }
  THROW_SEMANTIC_ERROR(
    "const std::string EntityManager::getEgoName(const std::string & name) function was called, "
    "but ego vehicle does not exist");
}

auto EntityManager::getObstacle(const std::string & name)
  -> std::optional<traffic_simulator_msgs::msg::Obstacle>
{
  if (!npc_logic_started_) {
    return std::nullopt;
  }
  return entities_.at(name)->getObstacle();
}

auto EntityManager::getRelativePose(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to) const
  -> geometry_msgs::msg::Pose
{
  return math::geometry::getRelativePose(from, to);
}

auto EntityManager::getRelativePose(
  const geometry_msgs::msg::Pose & from, const std::string & to) const -> geometry_msgs::msg::Pose
{
  return getRelativePose(from, getMapPose(to));
}

auto EntityManager::getRelativePose(
  const std::string & from, const geometry_msgs::msg::Pose & to) const -> geometry_msgs::msg::Pose
{
  return getRelativePose(getMapPose(from), to);
}

auto EntityManager::getRelativePose(const std::string & from, const std::string & to) const
  -> geometry_msgs::msg::Pose
{
  return getRelativePose(getMapPose(from), getMapPose(to));
}

auto EntityManager::getRelativePose(
  const geometry_msgs::msg::Pose & from, const CanonicalizedLaneletPose & to) const
  -> geometry_msgs::msg::Pose
{
  return getRelativePose(from, toMapPose(to));
}

auto EntityManager::getRelativePose(
  const CanonicalizedLaneletPose & from, const geometry_msgs::msg::Pose & to) const
  -> geometry_msgs::msg::Pose
{
  return getRelativePose(toMapPose(from), to);
}

auto EntityManager::getRelativePose(
  const std::string & from, const CanonicalizedLaneletPose & to) const -> geometry_msgs::msg::Pose
{
  return getRelativePose(getMapPose(from), toMapPose(to));
}

auto EntityManager::getRelativePose(
  const CanonicalizedLaneletPose & from, const std::string & to) const -> geometry_msgs::msg::Pose
{
  return getRelativePose(toMapPose(from), getMapPose(to));
}

auto EntityManager::getStepTime() const noexcept -> double { return step_time_; }

auto EntityManager::getWaypoints(const std::string & name)
  -> traffic_simulator_msgs::msg::WaypointsArray
{
  if (!npc_logic_started_) {
    return traffic_simulator_msgs::msg::WaypointsArray();
  }
  return entities_.at(name)->getWaypoints();
}

bool EntityManager::isEgo(const std::string & name) const
{
  using traffic_simulator_msgs::msg::EntityType;
  return getEntityType(name).type == EntityType::EGO and
         dynamic_cast<EgoEntity const *>(entities_.at(name).get());
}

bool EntityManager::isEgoSpawned() const
{
  for (const auto & name : getEntityNames()) {
    if (isEgo(name)) {
      return true;
    }
  }
  return false;
}

bool EntityManager::isInLanelet(
  const std::string & name, const std::int64_t lanelet_id, const double tolerance)
{
  const auto status = getEntityStatus(name);
  if (not status.laneMatchingSucceed()) {
    return false;
  }
  if (isSameLaneletId(status, lanelet_id)) {
    return true;
  } else {
    auto dist0 = hdmap_utils_ptr_->getLongitudinalDistance(
      helper::constructLaneletPose(lanelet_id, hdmap_utils_ptr_->getLaneletLength(lanelet_id)),
      status.getLaneletPose());
    auto dist1 = hdmap_utils_ptr_->getLongitudinalDistance(
      status.getLaneletPose(), helper::constructLaneletPose(lanelet_id, 0));
    if (dist0 and dist0.value() < tolerance) {
      return true;
    }
    if (dist1 and dist1.value() < tolerance) {
      return true;
    }
  }
  return false;
}

bool EntityManager::isStopping(const std::string & name) const
{
  return std::fabs(getCurrentTwist(name).linear.x) < std::numeric_limits<double>::epsilon();
}

bool EntityManager::reachPosition(
  const std::string & name, const std::string & target_name, const double tolerance) const
{
  return reachPosition(name, getMapPose(target_name), tolerance);
}

bool EntityManager::reachPosition(
  const std::string & name, const geometry_msgs::msg::Pose & target_pose,
  const double tolerance) const
{
  return math::geometry::getDistance(getMapPose(name), target_pose) < tolerance;
}

bool EntityManager::reachPosition(
  const std::string & name, const CanonicalizedLaneletPose & lanelet_pose,
  const double tolerance) const
{
  return reachPosition(name, static_cast<geometry_msgs::msg::Pose>(lanelet_pose), tolerance);
}

void EntityManager::requestLaneChange(
  const std::string & name, const traffic_simulator::lane_change::Direction & direction)
{
  if (const auto lanelet_pose = getLaneletPose(name)) {
    if (
      const auto target = hdmap_utils_ptr_->getLaneChangeableLaneletId(
        static_cast<LaneletPose>(lanelet_pose.value()).lanelet_id, direction)) {
      requestLaneChange(name, target.value());
    }
  }
}

bool EntityManager::trafficLightsChanged()
{
  return conventional_traffic_light_manager_ptr_->hasAnyLightChanged() or
         v2i_traffic_light_manager_ptr_->hasAnyLightChanged();
}

void EntityManager::requestSpeedChange(
  const std::string & name, double target_speed, bool continuous)
{
  if (isEgo(name) && getCurrentTime() > 0) {
    THROW_SEMANTIC_ERROR("You cannot set target speed to the ego vehicle after starting scenario.");
  }
  return entities_.at(name)->requestSpeedChange(target_speed, continuous);
}

void EntityManager::requestSpeedChange(
  const std::string & name, const double target_speed, const speed_change::Transition transition,
  const speed_change::Constraint constraint, const bool continuous)
{
  if (isEgo(name) && getCurrentTime() > 0) {
    THROW_SEMANTIC_ERROR("You cannot set target speed to the ego vehicle after starting scenario.");
  }
  return entities_.at(name)->requestSpeedChange(target_speed, transition, constraint, continuous);
}

void EntityManager::requestSpeedChange(
  const std::string & name, const speed_change::RelativeTargetSpeed & target_speed, bool continuous)
{
  if (isEgo(name) && getCurrentTime() > 0) {
    THROW_SEMANTIC_ERROR("You cannot set target speed to the ego vehicle after starting scenario.");
  }
  return entities_.at(name)->requestSpeedChange(target_speed, continuous);
}

void EntityManager::requestSpeedChange(
  const std::string & name, const speed_change::RelativeTargetSpeed & target_speed,
  const speed_change::Transition transition, const speed_change::Constraint constraint,
  const bool continuous)
{
  if (isEgo(name) && getCurrentTime() > 0) {
    THROW_SEMANTIC_ERROR("You cannot set target speed to the ego vehicle after starting scenario.");
  }
  return entities_.at(name)->requestSpeedChange(target_speed, transition, constraint, continuous);
}

auto EntityManager::setEntityStatus(
  const std::string & name, const CanonicalizedEntityStatus & status) -> void
{
  if (isEgo(name) && getCurrentTime() > 0) {
    THROW_SEMANTIC_ERROR(
      "You cannot set entity status to the ego vehicle name ", std::quoted(name),
      " after starting scenario.");
  } else {
    entities_.at(name)->setStatus(status);
  }
}

auto EntityManager::setEntityStatusExternally(
  const std::string & name, const CanonicalizedEntityStatus & status) -> void
{
  if (not isEgo(name)) {
    THROW_SEMANTIC_ERROR(
      "You cannot set entity status externally to the vehicle other than ego named ",
      std::quoted(name), ".");
  } else {
    dynamic_cast<EgoEntity *>(entities_[name].get())->setStatusExternally(status);
  }
}

void EntityManager::setVerbose(const bool verbose)
{
  configuration.verbose = verbose;
  for (auto & entity : entities_) {
    entity.second->verbose = verbose;
  }
}

auto EntityManager::toMapPose(const CanonicalizedLaneletPose & lanelet_pose) const
  -> const geometry_msgs::msg::Pose
{
  return static_cast<geometry_msgs::msg::Pose>(lanelet_pose);
}

auto EntityManager::updateNpcLogic(
  const std::string & name,
  const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType> & type_list)
  -> const CanonicalizedEntityStatus &
{
  if (configuration.verbose) {
    std::cout << "update " << name << " behavior" << std::endl;
  }
  entities_[name]->setEntityTypeList(type_list);
  entities_[name]->onUpdate(current_time_, step_time_);
  return entities_[name]->getStatus();
}

void EntityManager::update(const double current_time, const double step_time)
{
  traffic_simulator::helper::StopWatch<std::chrono::milliseconds> stop_watch_update(
    "EntityManager::update", configuration.verbose);
  step_time_ = step_time;
  current_time_ = current_time;
  setVerbose(configuration.verbose);
  if (npc_logic_started_) {
    conventional_traffic_light_marker_publisher_ptr_->createTimer(
      configuration.conventional_traffic_light_publish_rate);
    v2i_traffic_light_publisher_ptr_->createTimer(configuration.v2i_traffic_light_publish_rate);
    v2i_traffic_light_marker_publisher_ptr_->createTimer(
      configuration.v2i_traffic_light_publish_rate);
  }
  auto type_list = getEntityTypeList();
  std::unordered_map<std::string, CanonicalizedEntityStatus> all_status;
  for (auto && [name, entity] : entities_) {
    all_status.emplace(name, entity->getStatus());
  }
  for (auto && [name, entity] : entities_) {
    entity->setOtherStatus(all_status);
  }
  all_status.clear();
  for (auto && [name, entity] : entities_) {
    all_status.emplace(name, updateNpcLogic(name, type_list));
  }
  for (auto && [name, entity] : entities_) {
    entity->setOtherStatus(all_status);
  }
  traffic_simulator_msgs::msg::EntityStatusWithTrajectoryArray status_array_msg;
  for (auto && [name, status] : all_status) {
    traffic_simulator_msgs::msg::EntityStatusWithTrajectory status_with_trajectory;
    status_with_trajectory.waypoint = getWaypoints(name);
    for (const auto & goal : getGoalPoses<geometry_msgs::msg::Pose>(name)) {
      status_with_trajectory.goal_pose.push_back(goal);
    }
    if (const auto obstacle = getObstacle(name); obstacle) {
      status_with_trajectory.obstacle = obstacle.value();
      status_with_trajectory.obstacle_find = true;
    } else {
      status_with_trajectory.obstacle_find = false;
    }
    status_with_trajectory.status = static_cast<EntityStatus>(status);
    status_with_trajectory.name = name;
    status_with_trajectory.time = current_time + step_time;
    status_array_msg.data.emplace_back(status_with_trajectory);
  }
  entity_status_array_pub_ptr_->publish(status_array_msg);
  stop_watch_update.stop();
  if (configuration.verbose) {
    stop_watch_update.print();
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

void EntityManager::startNpcLogic()
{
  npc_logic_started_ = true;
  for (auto it = entities_.begin(); it != entities_.end(); it++) {
    it->second->startNpcLogic();
  }
}

}  // namespace entity
}  // namespace traffic_simulator
