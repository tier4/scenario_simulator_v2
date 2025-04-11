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

#include <geometry/distance.hpp>
#include <geometry/vector3/operator.hpp>
#include <std_msgs/msg/header.hpp>
#include <traffic_simulator/entity/entity_manager.hpp>
#include <traffic_simulator/helper/stop_watch.hpp>

namespace traffic_simulator
{
namespace entity
{
// global
auto EntityManager::setTrafficLights(const std::shared_ptr<TrafficLights> & traffic_lights_ptr)
  -> void
{
  traffic_lights_ptr_ = traffic_lights_ptr;
}

auto EntityManager::setVerbose(const bool verbose) -> void
{
  configuration_.verbose = verbose;
  for (const auto & [name, entity_ptr] : entities_) {
    entity_ptr->verbose = verbose;
  }
}

auto EntityManager::startNpcLogic(const double current_time) -> void
{
  npc_logic_started_ = true;

  for (const auto & [name, entity_ptr] : entities_) {
    entity_ptr->updateEntityStatusTimestamp(current_time);
  }
}

auto EntityManager::isNpcLogicStarted() const -> bool { return npc_logic_started_; }

auto EntityManager::makeDebugMarker() const -> visualization_msgs::msg::MarkerArray
{
  visualization_msgs::msg::MarkerArray marker;
  for (const auto & [name, entity_ptr] : entities_) {
    entity_ptr->appendDebugMarker(marker);
  }
  return marker;
}

// update
auto EntityManager::update(const double current_time, const double step_time) -> void
{
  helper::StopWatch<std::chrono::milliseconds> stop_watch_update(
    "EntityManager::update", configuration_.verbose);
  setVerbose(configuration_.verbose);
  if (npc_logic_started_) {
    traffic_lights_ptr_->startTrafficLightsUpdate(
      configuration_.conventional_traffic_light_publish_rate,
      configuration_.v2i_traffic_light_publish_rate);
  }
  std::unordered_map<std::string, CanonicalizedEntityStatus> all_status;
  for (const auto & [name, entity_ptr] : entities_) {
    all_status.try_emplace(name, entity_ptr->getCanonicalizedStatus());
  }
  for (const auto & [name, entity_ptr] : entities_) {
    entity_ptr->setOtherStatus(all_status);
  }
  all_status.clear();

  std::shared_ptr<EuclideanDistancesMap> distances = calculateEuclideanDistances();

  for (const auto & [name, entity_ptr] : entities_) {
    all_status.try_emplace(name, updateNpcLogic(name, current_time, step_time, distances));
  }
  for (const auto & [name, entity_ptr] : entities_) {
    entity_ptr->setOtherStatus(all_status);
  }
  traffic_simulator_msgs::msg::EntityStatusWithTrajectoryArray status_array_msg;
  for (const auto & [name, status] : all_status) {
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
  if (configuration_.verbose) {
    stop_watch_update.print();
  }
}

auto EntityManager::updateNpcLogic(
  const std::string & name, const double current_time, const double step_time,
  const std::shared_ptr<EuclideanDistancesMap> & distances) -> const CanonicalizedEntityStatus &
{
  if (configuration_.verbose) {
    std::cout << "update " << name << " behavior" << std::endl;
  }
  auto & entity = getEntity(name);
  // Update npc completely if logic has started, otherwise update Autoware only - if it is Ego
  if (npc_logic_started_) {
    entity.setEuclideanDistancesMap(distances);
    entity.onUpdate(current_time, step_time);
  } else if (entity.is<entity::EgoEntity>()) {
    getEgoEntity(name).updateFieldOperatorApplication();
  }
  return entity.getCanonicalizedStatus();
}

auto EntityManager::updateHdmapMarker() const -> void
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

auto EntityManager::broadcastEntityTransform() -> void
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
  if (const auto ego_name = getFirstEgoName()) {
    if (!is_send) {
      pose = getEntity(ego_name.value()).getMapPose();
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

  if (!names.empty()) {
    if (!is_send) {
      pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
               .position(std::accumulate(
                 names.begin(), names.end(), geometry_msgs::msg::Point(),
                 [this, names](geometry_msgs::msg::Point point, const std::string & name) {
                   point += getEntity(name).getMapPose().position *
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

auto EntityManager::broadcastTransform(
  const geometry_msgs::msg::PoseStamped & pose, const bool static_transform) -> void
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

// ego - checks, getters
auto EntityManager::getNumberOfEgo() const -> std::size_t
{
  return std::count_if(std::begin(entities_), std::end(entities_), [this](const auto & each) {
    return each.second->template is<EgoEntity>();
  });
}

auto EntityManager::isAnyEgoSpawned() const -> bool
{
  return std::any_of(std::begin(entities_), std::end(entities_), [this](const auto & each) {
    return each.second->template is<EgoEntity>();
  });
}

auto EntityManager::getFirstEgoName() const -> std::optional<std::string>
{
  for (const auto & [name, entity_ptr] : entities_) {
    if (entity_ptr->template is<EgoEntity>()) {
      return entity_ptr->getName();
    }
  }
  return std::nullopt;
}

auto EntityManager::getEgoEntity(const std::string & name) -> entity::EgoEntity &
{
  if (const auto it = entities_.find(name); it == entities_.end()) {
    THROW_SEMANTIC_ERROR("Entity ", std::quoted(name), " does not exist.");
  } else if (auto ego_entity_ptr = dynamic_cast<EgoEntity *>(it->second.get()); !ego_entity_ptr) {
    THROW_SEMANTIC_ERROR("Entity ", std::quoted(name), " exists, but it is not ego");
  } else {
    return *ego_entity_ptr;
  }
}

auto EntityManager::getEgoEntity(const std::string & name) const -> const entity::EgoEntity &
{
  if (const auto it = entities_.find(name); it == entities_.end()) {
    THROW_SEMANTIC_ERROR("Entity ", std::quoted(name), " does not exist.");
  } else if (const auto ego_entity_ptr = dynamic_cast<EgoEntity const *>(it->second.get());
             !ego_entity_ptr) {
    THROW_SEMANTIC_ERROR("Entity ", std::quoted(name), " exists, but it is not ego");
  } else {
    return *ego_entity_ptr;
  }
}

// entities - checks, getters
auto EntityManager::isEntityExist(const std::string & name) const -> bool
{
  return entities_.find(name) != std::end(entities_);
}

auto EntityManager::getEntityNames() const -> const std::vector<std::string>
{
  std::vector<std::string> names{};
  for (const auto & [name, entity_ptr] : entities_) {
    names.push_back(name);
  }
  return names;
}

auto EntityManager::getEntity(const std::string & name) -> entity::EntityBase &
{
  if (const auto it = entities_.find(name); it != entities_.end()) {
    return *(it->second);
  } else {
    THROW_SEMANTIC_ERROR("Entity ", std::quoted(name), " does not exist.");
  }
}

auto EntityManager::getEntity(const std::string & name) const -> const entity::EntityBase &
{
  if (const auto it = entities_.find(name); it != entities_.end()) {
    return *(it->second);
  } else {
    THROW_SEMANTIC_ERROR("Entity ", std::quoted(name), " does not exist.");
  }
}

auto EntityManager::getEntityPointer(const std::string & name) const
  -> std::shared_ptr<traffic_simulator::entity::EntityBase>
{
  if (auto it = entities_.find(name); it != entities_.end()) {
    return it->second;
  } else {
    /*
      This method returns nullptr, due to the fact that the interpretation of the scenario operates in
      such a way that checking a condition, e.g. DistanceCondition, is called also for Entities that
      have not yet been spawned. For example, if for DistanceCondition any getEntity() returns
      nullptr, the condition returns a distance equal to NaN. For this reason, using getEntity() with
      throwing an exception is not recommended.
    */
    return nullptr;
  }
};

// entities - respawn, despawn, reset
auto EntityManager::resetBehaviorPlugin(
  const std::string & name, const std::string & behavior_plugin_name) -> void
{
  const auto & reference_entity = getEntity(name);
  const auto & status = reference_entity.getCanonicalizedStatus();
  const auto behavior_parameter = reference_entity.getBehaviorParameter();
  if (reference_entity.is<EgoEntity>()) {
    THROW_SEMANTIC_ERROR(
      "Entity :", name, "is EgoEntity.", "You cannot reset behavior plugin of EgoEntity.");
  } else if (reference_entity.is<MiscObjectEntity>()) {
    THROW_SEMANTIC_ERROR(
      "Entity :", name, "is MiscObjectEntity.",
      "You cannot reset behavior plugin of MiscObjectEntity.");
  } else if (reference_entity.is<VehicleEntity>()) {
    const auto parameters = getVehicleParameters(name);
    despawnEntity(name);
    spawnEntity<VehicleEntity>(
      name, status.getMapPose(), parameters, status.getTime(), behavior_plugin_name);
  } else if (reference_entity.is<PedestrianEntity>()) {
    const auto parameters = getPedestrianParameters(name);
    despawnEntity(name);
    spawnEntity<PedestrianEntity>(
      name, status.getMapPose(), parameters, status.getTime(), behavior_plugin_name);
  } else {
    THROW_SIMULATION_ERROR(
      "Entity :", name, "is unkown entity type.", "Please contact to developer.");
  }
  auto & spawned_entity = getEntity(name);
  spawned_entity.setLinearJerk(status.getLinearJerk());
  spawned_entity.setAcceleration(status.getAccel());
  spawned_entity.setTwist(status.getTwist());
  spawned_entity.setBehaviorParameter(behavior_parameter);
}

auto EntityManager::despawnEntity(const std::string & name) -> bool
{
  return isEntityExist(name) && entities_.erase(name);
}

// traffics, lanelet
auto EntityManager::getHdmapUtils() -> const std::shared_ptr<hdmap_utils::HdMapUtils> &
{
  return hdmap_utils_ptr_;
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
  if (const auto entity_ptr = dynamic_cast<PedestrianEntity const *>(entities_.at(name).get())) {
    return entity_ptr->pedestrian_parameters;
  }
  THROW_SIMULATION_ERROR(
    "EntityType: ", getEntity(name).getEntityTypename(), ", does not have pedestrian parameter.",
    "Please check description of the scenario and entity type of the Entity: " + name);
}

auto EntityManager::getVehicleParameters(const std::string & name) const
  -> const traffic_simulator_msgs::msg::VehicleParameters &
{
  if (const auto vehicle = dynamic_cast<VehicleEntity const *>(entities_.at(name).get())) {
    return vehicle->vehicle_parameters;
  }
  THROW_SIMULATION_ERROR(
    "EntityType: ", getEntity(name).getEntityTypename(), ", does not have pedestrian parameter.",
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

auto EntityManager::calculateEuclideanDistances() -> std::shared_ptr<EuclideanDistancesMap>
{
  std::shared_ptr<EuclideanDistancesMap> distances = std::make_shared<EuclideanDistancesMap>();
  for (auto && [name_from, entity_from] : entities_) {
    for (auto && [name_to, entity_to] : entities_) {
      if (const auto pair = std::minmax(name_from, name_to);
          distances->find(pair) == distances->end() && name_from != name_to) {
        distances->emplace(
          pair, math::geometry::getDistance(entity_from->getMapPose(), entity_to->getMapPose()));
      }
    }
  }
  return distances;
}
}  // namespace entity
}  // namespace traffic_simulator
