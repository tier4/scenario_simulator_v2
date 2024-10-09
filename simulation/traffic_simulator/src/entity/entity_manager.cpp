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
#include <geometry/vector3/operator.hpp>
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
#include <traffic_simulator/utils/distance.hpp>
#include <unordered_map>
#include <vector>

namespace traffic_simulator
{
namespace entity
{
void EntityManager::broadcastEntityTransform()
{
  static bool is_send = false;
  static geometry_msgs::msg::Pose pose;

  using math::geometry::operator/;
  using math::geometry::operator*;
  using math::geometry::operator+=;
  std::vector<std::string> names = getEntityNames();
  /**
   * @note This part of the process is intended to ensure that frames are issued in a position that makes
   * it as easy as possible to see the entities that will appear in the scenario.
   * In the past, we used to publish the frames of all entities, but that would be too heavy processing,
   * so we publish the average of the coordinates of all entities.
   */
  if (isEgoSpawned()) {
    if (const auto ego = getEntity(getEgoName())) {
      if (!is_send) {
        pose = ego->getMapPose();
        is_send = true;
      }
      broadcastTransform(
        geometry_msgs::build<geometry_msgs::msg::PoseStamped>()
          /**
           * @note This is the intended implementation.
           * It is easier to create rviz config if the name "ego" is fixed,
           * so the frame_id "ego" is issued regardless of the name of the ego entity.
           */
          .header(std_msgs::build<std_msgs::msg::Header>().stamp(clock_ptr_->now()).frame_id("ego"))
          .pose(pose),
        true);
    }
  }
  if (!names.empty()) {
    if (!is_send) {
      pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
               .position(std::accumulate(
                 names.begin(), names.end(), geometry_msgs::msg::Point(),
                 [this, names](geometry_msgs::msg::Point point, const std::string & name) {
                   point += getEntity(name)->getMapPose().position *
                            (1.0 / static_cast<double>(names.size()));
                   return point;
                 }))
               .orientation(geometry_msgs::msg::Quaternion());
      is_send = true;
    }
    broadcastTransform(
      geometry_msgs::build<geometry_msgs::msg::PoseStamped>()
        .header(
          std_msgs::build<std_msgs::msg::Header>().stamp(clock_ptr_->now()).frame_id("entities"))
        .pose(pose),
      true);
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

bool EntityManager::checkCollision(
  const std::string & first_entity_name, const std::string & second_entity_name)
{
  if (first_entity_name != second_entity_name) {
    if (const auto first_entity = getEntity(first_entity_name)) {
      if (const auto second_entity = getEntity(second_entity_name)) {
        return math::geometry::checkCollision2D(
          first_entity->getMapPose(), first_entity->getBoundingBox(), second_entity->getMapPose(),
          second_entity->getBoundingBox());
      }
    }
  }
  return false;
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

auto EntityManager::getEntityNames() const -> const std::vector<std::string>
{
  std::vector<std::string> names{};
  for (const auto & each : entities_) {
    names.push_back(each.first);
  }
  return names;
}

auto EntityManager::getEntity(const std::string & name) const
  -> std::shared_ptr<traffic_simulator::entity::EntityBase>
{
  if (auto it = entities_.find(name); it != entities_.end()) {
    return it->second;
  } else {
    /*
      This method returns nullptr, due to the fact that the interpretation of the scenario operates in
      such a way that checking a condition, e.g. DistanceCondition, is called also for Entities that
      have not yet been spawned. For example, if for DistanceCondition any getEntity() returns
      nullptr, the condition returns a distance equal to NaN. For this reason, throwing an exception
      through getEntity() is not recommended.
    */
    return nullptr;
  }
};

auto EntityManager::getEntityStatus(const std::string & name) const
  -> const CanonicalizedEntityStatus &
{
  if (const auto entity = getEntity(name)) {
    return entity->getCanonicalizedStatus();
  } else {
    THROW_SEMANTIC_ERROR("entity ", std::quoted(name), " does not exist.");
  }
}

auto EntityManager::getHdmapUtils() -> const std::shared_ptr<hdmap_utils::HdMapUtils> &
{
  return hdmap_utils_ptr_;
}

auto EntityManager::getNumberOfEgo() const -> std::size_t
{
  return std::count_if(std::begin(entities_), std::end(entities_), [this](const auto & each) {
    return is<EgoEntity>(each.first);
  });
}

const std::string EntityManager::getEgoName() const
{
  const auto names = getEntityNames();
  for (const auto & name : names) {
    if (is<EgoEntity>(name)) {
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
  if (not npc_logic_started_) {
    return std::nullopt;
  } else {
    return entities_.at(name)->getObstacle();
  }
}

auto EntityManager::getPedestrianParameters(const std::string & name) const
  -> const traffic_simulator_msgs::msg::PedestrianParameters &
{
  if (const auto entity = dynamic_cast<PedestrianEntity const *>(entities_.at(name).get())) {
    return entity->pedestrian_parameters;
  }
  THROW_SIMULATION_ERROR(
    "EntityType: ", getEntityTypename(name), ", does not have pedestrian parameter.",
    "Please check description of the scenario and entity type of the Entity: " + name);
}

auto EntityManager::getVehicleParameters(const std::string & name) const
  -> const traffic_simulator_msgs::msg::VehicleParameters &
{
  if (const auto vehicle = dynamic_cast<VehicleEntity const *>(entities_.at(name).get())) {
    return vehicle->vehicle_parameters;
  }
  THROW_SIMULATION_ERROR(
    "EntityType: ", getEntityTypename(name), ", does not have pedestrian parameter.",
    "Please check description of the scenario and entity type of the Entity: " + name);
}

auto EntityManager::getWaypoints(const std::string & name)
  -> traffic_simulator_msgs::msg::WaypointsArray
{
  if (not npc_logic_started_) {
    return traffic_simulator_msgs::msg::WaypointsArray();
  } else {
    return entities_.at(name)->getWaypoints();
  }
}

bool EntityManager::isEgoSpawned() const
{
  for (const auto & name : getEntityNames()) {
    if (is<EgoEntity>(name)) {
      return true;
    }
  }
  return false;
}

bool EntityManager::isInLanelet(
  const std::string & name, const lanelet::Id lanelet_id, const double tolerance)
{
  if (const auto entity = getEntity(name)) {
    if (const auto canonicalized_lanelet_pose = entity->getCanonicalizedLaneletPose()) {
      return pose::isInLanelet(
        canonicalized_lanelet_pose.value(), lanelet_id, tolerance, hdmap_utils_ptr_);
    }
  }
  return false;
}

bool EntityManager::isStopping(const std::string & name) const
{
  return std::fabs(getCurrentTwist(name).linear.x) < std::numeric_limits<double>::epsilon();
}

void EntityManager::requestLaneChange(
  const std::string & name, const traffic_simulator::lane_change::Direction & direction)
{
  if (const auto entity = getEntity(name); entity && entity->laneMatchingSucceed()) {
    if (
      const auto target = hdmap_utils_ptr_->getLaneChangeableLaneletId(
        entity->getCanonicalizedStatus().getLaneletId(), direction)) {
      requestLaneChange(name, target.value());
    }
  }
}

void EntityManager::resetBehaviorPlugin(
  const std::string & name, const std::string & behavior_plugin_name)
{
  const auto & status = getEntityStatus(name);
  const auto behavior_parameter = getBehaviorParameter(name);
  if (is<EgoEntity>(name)) {
    THROW_SEMANTIC_ERROR(
      "Entity :", name, "is EgoEntity.", "You cannot reset behavior plugin of EgoEntity.");
  } else if (is<MiscObjectEntity>(name)) {
    THROW_SEMANTIC_ERROR(
      "Entity :", name, "is MiscObjectEntity.",
      "You cannot reset behavior plugin of MiscObjectEntity.");
  } else if (is<VehicleEntity>(name)) {
    const auto parameters = getVehicleParameters(name);
    despawnEntity(name);
    spawnEntity<VehicleEntity>(
      name, status.getMapPose(), parameters, status.getTime(), behavior_plugin_name);
  } else if (is<PedestrianEntity>(name)) {
    const auto parameters = getPedestrianParameters(name);
    despawnEntity(name);
    spawnEntity<PedestrianEntity>(
      name, status.getMapPose(), parameters, status.getTime(), behavior_plugin_name);
  } else {
    THROW_SIMULATION_ERROR(
      "Entity :", name, "is unkown entity type.", "Please contact to developer.");
  }
  setLinearJerk(name, status.getLinearJerk());
  setAcceleration(name, status.getAccel());
  setTwist(name, status.getTwist());
  setBehaviorParameter(name, behavior_parameter);
}

auto EntityManager::getCurrentAction(const std::string & name) const -> std::string
{
  if (const auto entity = getEntity(name)) {
    if (not npc_logic_started_ and not is<EgoEntity>(name)) {
      return "waiting";
    } else {
      return entity->getCurrentAction();
    }
  } else {
    THROW_SEMANTIC_ERROR("entity : ", name, "does not exist");
  }
}

bool EntityManager::trafficLightsChanged()
{
  return conventional_traffic_light_manager_ptr_->hasAnyLightChanged() or
         v2i_traffic_light_manager_ptr_->hasAnyLightChanged();
}

void EntityManager::setVerbose(const bool verbose)
{
  configuration.verbose = verbose;
  for (auto & entity : entities_) {
    entity.second->verbose = verbose;
  }
}

auto EntityManager::updateNpcLogic(
  const std::string & name, const double current_time, const double step_time)
  -> const CanonicalizedEntityStatus &
{
  if (configuration.verbose) {
    std::cout << "update " << name << " behavior" << std::endl;
  }
  if (const auto entity = getEntity(name)) {
    // Update npc completely if logic has started, otherwise update Autoware only - if it is Ego
    if (npc_logic_started_) {
      entity->onUpdate(current_time, step_time);
    } else if (const auto ego_entity = std::dynamic_pointer_cast<const EgoEntity>(entity)) {
      ego_entity->updateFieldOperatorApplication();
    }
    return entity->getCanonicalizedStatus();
  } else {
    THROW_SEMANTIC_ERROR("entity ", std::quoted(name), " does not exist.");
  }
}

void EntityManager::update(const double current_time, const double step_time)
{
  traffic_simulator::helper::StopWatch<std::chrono::milliseconds> stop_watch_update(
    "EntityManager::update", configuration.verbose);
  setVerbose(configuration.verbose);
  if (npc_logic_started_) {
    conventional_traffic_light_updater_.createTimer(
      configuration.conventional_traffic_light_publish_rate);
    v2i_traffic_light_updater_.createTimer(configuration.v2i_traffic_light_publish_rate);
  }
  std::unordered_map<std::string, CanonicalizedEntityStatus> all_status;
  for (auto && [name, entity] : entities_) {
    all_status.emplace(name, entity->getCanonicalizedStatus());
  }
  for (auto && [name, entity] : entities_) {
    entity->setOtherStatus(all_status);
  }
  all_status.clear();
  for (auto && [name, entity] : entities_) {
    all_status.emplace(name, updateNpcLogic(name, current_time, step_time));
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
    status_with_trajectory.status.time = current_time + step_time;
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

auto EntityManager::startNpcLogic(const double current_time) -> void
{
  npc_logic_started_ = true;

  for ([[maybe_unused]] auto && [name, entity] : entities_) {
    entity->updateEntityStatusTimestamp(current_time);
  }
}
}  // namespace entity
}  // namespace traffic_simulator
