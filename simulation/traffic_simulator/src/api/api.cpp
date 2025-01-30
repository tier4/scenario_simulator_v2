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

#include <geometry/intersection/collision.hpp>
#include <geometry/quaternion/quaternion_to_euler.hpp>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator/utils/pose.hpp>

namespace traffic_simulator
{
// global
auto API::init() -> bool
{
  if (not configuration_.standalone_mode) {
    simulation_api_schema::InitializeRequest request;
    request.set_initialize_time(clock_.getCurrentSimulationTime());
    request.set_lanelet2_map_path(configuration_.lanelet2_map_path().string());
    request.set_realtime_factor(clock_.realtime_factor);
    request.set_step_time(clock_.getStepTime());
    simulation_interface::toProto(
      clock_.getCurrentRosTime(), *request.mutable_initialize_ros_time());
    return zeromq_client_.call(request).result().success();
  } else {
    return true;
  }
}

auto API::setVerbose(const bool verbose) -> void { entity_manager_ptr_->setVerbose(verbose); }

auto API::setSimulationStepTime(const double step_time) -> bool
{
  /**
   * @note Pausing the simulation by setting the realtime_factor_ value to 0 is not supported and causes the simulation crash.
   * For that reason, before performing the action, it needs to be ensured that the incoming request data is a positive number.
   */
  if (step_time >= 0.001) {
    clock_.realtime_factor = step_time;
    simulation_api_schema::UpdateStepTimeRequest request;
    request.set_simulation_step_time(clock_.getStepTime());
    return zeromq_client_.call(request).result().success();
  } else {
    return false;
  }
}

auto API::startNpcLogic() -> void
{
  if (entity_manager_ptr_->isNpcLogicStarted()) {
    THROW_SIMULATION_ERROR("NPC logics are already started.");
  } else {
    clock_.start();
    entity_manager_ptr_->startNpcLogic(getCurrentTime());
  }
}

auto API::isNpcLogicStarted() const -> bool { return entity_manager_ptr_->isNpcLogicStarted(); }

auto API::getCurrentTime() const noexcept -> double { return clock_.getCurrentScenarioTime(); }

auto API::closeZMQConnection() -> void { zeromq_client_.closeConnection(); }

// update
auto API::updateTimeInSim() -> bool
{
  simulation_api_schema::UpdateFrameRequest request;
  request.set_current_simulation_time(clock_.getCurrentSimulationTime());
  request.set_current_scenario_time(getCurrentTime());
  simulation_interface::toProto(
    clock_.getCurrentRosTimeAsMsg().clock, *request.mutable_current_ros_time());
  return zeromq_client_.call(request).result().success();
}

auto API::updateEntitiesStatusInSim() -> bool
{
  simulation_api_schema::UpdateEntityStatusRequest req;
  req.set_npc_logic_started(entity_manager_ptr_->isNpcLogicStarted());
  for (const auto & entity_name : entity_manager_ptr_->getEntityNames()) {
    const auto entity = entity_manager_ptr_->getEntity(entity_name);
    const auto entity_status = static_cast<EntityStatus>(entity->getCanonicalizedStatus());
    simulation_interface::toProto(entity_status, *req.add_status());
    if (entity->is<entity::EgoEntity>()) {
      req.set_overwrite_ego_status(entity->isControlledBySimulator());
    }
  }

  simulation_api_schema::UpdateEntityStatusResponse res;
  if (auto res = zeromq_client_.call(req); res.result().success()) {
    for (const auto & res_status : res.status()) {
      auto entity = entity_manager_ptr_->getEntity(res_status.name());
      auto entity_status = static_cast<EntityStatus>(entity->getCanonicalizedStatus());
      simulation_interface::toMsg(res_status.pose(), entity_status.pose);
      simulation_interface::toMsg(res_status.action_status(), entity_status.action_status);

      if (entity->is<entity::EgoEntity>()) {
        entity->setMapPose(entity_status.pose);
        entity->setTwist(entity_status.action_status.twist);
        entity->setAcceleration(entity_status.action_status.accel);
      } else {
        entity->setStatus(entity_status);
      }
    }
    return true;
  }
  return false;
}

auto API::updateTrafficLightsInSim() -> bool
{
  if (traffic_lights_ptr_->isAnyTrafficLightChanged()) {
    auto request =
      traffic_lights_ptr_->getConventionalTrafficLights()->generateUpdateTrafficLightsRequest();
    return zeromq_client_.call(request).result().success();
  }
  /// @todo handle response
  return simulation_api_schema::UpdateTrafficLightsResponse().result().success();
}

auto API::updateFrame() -> bool
{
  if (configuration_.standalone_mode && entity_manager_ptr_->isAnyEgoSpawned()) {
    THROW_SEMANTIC_ERROR("Ego simulation is no longer supported in standalone mode");
  }

  if (!updateEntitiesStatusInSim()) {
    return false;
  }

  entity_manager_ptr_->update(getCurrentTime(), clock_.getStepTime());
  traffic_controller_ptr_->execute(getCurrentTime(), clock_.getStepTime());

  if (not configuration_.standalone_mode) {
    if (!updateTrafficLightsInSim() || !updateTimeInSim()) {
      return false;
    }
  }

  entity_manager_ptr_->broadcastEntityTransform();
  clock_.update();
  clock_pub_->publish(clock_.getCurrentRosTimeAsMsg());
  debug_marker_pub_->publish(entity_manager_ptr_->makeDebugMarker());
  debug_marker_pub_->publish(traffic_controller_ptr_->makeDebugMarker());
  return true;
}

// sensors - attach
auto API::attachImuSensor(
  const std::string &, const simulation_api_schema::ImuSensorConfiguration & configuration) -> bool
{
  simulation_api_schema::AttachImuSensorRequest req;
  *req.mutable_configuration() = configuration;
  return zeromq_client_.call(req).result().success();
}

auto API::attachPseudoTrafficLightDetector(
  const simulation_api_schema::PseudoTrafficLightDetectorConfiguration & configuration) -> bool
{
  simulation_api_schema::AttachPseudoTrafficLightDetectorRequest req;
  *req.mutable_configuration() = configuration;
  return zeromq_client_.call(req).result().success();
}

auto API::attachLidarSensor(const simulation_api_schema::LidarConfiguration & lidar_configuration)
  -> bool
{
  if (configuration_.standalone_mode) {
    return true;
  } else {
    simulation_api_schema::AttachLidarSensorRequest req;
    *req.mutable_configuration() = lidar_configuration;
    return zeromq_client_.call(req).result().success();
  }
}

auto API::attachLidarSensor(
  const std::string & entity_name, const double lidar_sensor_delay,
  const helper::LidarType lidar_type) -> bool
{
  return attachLidarSensor(helper::constructLidarConfiguration(
    lidar_type, entity_name,
    getROS2Parameter<std::string>("architecture_type", "awf/universe/20240605"),
    lidar_sensor_delay));
}

auto API::attachDetectionSensor(
  const simulation_api_schema::DetectionSensorConfiguration & sensor_configuration) -> bool
{
  if (configuration_.standalone_mode) {
    return true;
  } else {
    simulation_api_schema::AttachDetectionSensorRequest req;
    *req.mutable_configuration() = sensor_configuration;
    return zeromq_client_.call(req).result().success();
  }
}

auto API::attachDetectionSensor(
  const std::string & entity_name, double detection_sensor_range, bool detect_all_objects_in_range,
  double pos_noise_stddev, int random_seed, double probability_of_lost,
  double object_recognition_delay) -> bool
{
  return attachDetectionSensor(helper::constructDetectionSensorConfiguration(
    entity_name, getROS2Parameter<std::string>("architecture_type", "awf/universe"), 0.1,
    detection_sensor_range, detect_all_objects_in_range, pos_noise_stddev, random_seed,
    probability_of_lost, object_recognition_delay));
}

auto API::attachOccupancyGridSensor(
  const simulation_api_schema::OccupancyGridSensorConfiguration & sensor_configuration) -> bool
{
  if (configuration_.standalone_mode) {
    return true;
  } else {
    simulation_api_schema::AttachOccupancyGridSensorRequest req;
    *req.mutable_configuration() = sensor_configuration;
    return zeromq_client_.call(req).result().success();
  }
}

// ego - checks, getters
auto API::isAnyEgoSpawned() const -> bool { return entity_manager_ptr_->isAnyEgoSpawned(); }

auto API::getEgoName() const -> const std::string & { return entity_manager_ptr_->getEgoName(); }

auto API::getEgoEntity() const -> std::shared_ptr<entity::EgoEntity>
{
  return entity_manager_ptr_->getEgoEntity();
}

auto API::getEgoEntity(const std::string & name) const -> std::shared_ptr<entity::EgoEntity>
{
  return entity_manager_ptr_->getEgoEntity(name);
}

// entities - checks, getters
auto API::isEntityExist(const std::string & name) const -> bool
{
  return entity_manager_ptr_->isEntityExist(name);
}

auto API::getEntityNames() const -> std::vector<std::string>
{
  return entity_manager_ptr_->getEntityNames();
}

auto API::getEntity(const std::string & name) const -> std::shared_ptr<entity::EntityBase>
{
  return entity_manager_ptr_->getEntity(name);
}

// entities - respawn, despawn, reset
auto API::resetBehaviorPlugin(const std::string & name, const std::string & behavior_plugin_name)
  -> void
{
  return entity_manager_ptr_->resetBehaviorPlugin(name, behavior_plugin_name);
}

auto API::respawn(
  const std::string & name, const geometry_msgs::msg::PoseWithCovarianceStamped & new_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose) -> void
{
  if (new_pose.header.frame_id != "map") {
    throw std::runtime_error("Respawn request with frame id other than map not supported.");
  } else {
    auto ego_entity = entity_manager_ptr_->getEgoEntity(name);
    // set new pose and default action status in EntityManager
    ego_entity->setControlledBySimulator(true);
    ego_entity->setStatus(new_pose.pose.pose);

    // read status from EntityManager, then send it to SimpleSensorSimulator
    simulation_api_schema::UpdateEntityStatusRequest req;
    simulation_interface::toProto(
      static_cast<EntityStatus>(entity_manager_ptr_->getEntity(name)->getCanonicalizedStatus()),
      *req.add_status());
    req.set_npc_logic_started(entity_manager_ptr_->isNpcLogicStarted());
    req.set_overwrite_ego_status(ego_entity->isControlledBySimulator());
    ego_entity->setControlledBySimulator(false);

    // check response
    if (const auto res = zeromq_client_.call(req); not res.result().success()) {
      throw common::SimulationError(
        "UpdateEntityStatus request failed for \"" + name + "\" entity during respawn.");
    } else if (const auto res_status = res.status().begin(); res.status().size() != 1) {
      throw common::SimulationError(
        "Failed to receive the new status of \"" + name + "\" entity after the update request.");
    } else if (const auto res_name = res_status->name(); res_name != name) {
      throw common::SimulationError(
        "Wrong entity status received during respawn. Expected: \"" + name + "\". Received: \"" +
        res_name + "\".");
    } else {
      // if valid, set response in EntityManager, then plan path and engage
      auto entity_status = static_cast<EntityStatus>(ego_entity->getCanonicalizedStatus());
      simulation_interface::toMsg(res_status->pose(), entity_status.pose);
      simulation_interface::toMsg(res_status->action_status(), entity_status.action_status);
      ego_entity->setMapPose(entity_status.pose);
      ego_entity->setTwist(entity_status.action_status.twist);
      ego_entity->setAcceleration(entity_status.action_status.accel);
      ego_entity->requestReplanRoute({goal_pose});
    }
  }
}

auto API::despawn(const std::string & name) -> bool
{
  const auto result = entity_manager_ptr_->despawnEntity(name);
  if (!result) {
    return false;
  }
  if (not configuration_.standalone_mode) {
    simulation_api_schema::DespawnEntityRequest req;
    req.set_name(name);
    return zeromq_client_.call(req).result().success();
  }
  return true;
}

auto API::despawnEntities() -> bool
{
  const auto entities = entity_manager_ptr_->getEntityNames();
  return std::all_of(
    entities.begin(), entities.end(), [&](const auto & entity) { return despawn(entity); });
}

// entities - features
auto API::checkCollision(
  const std::string & first_entity_name, const std::string & second_entity_name) const -> bool
{
  if (first_entity_name != second_entity_name) {
    const auto first_entity = getEntity(first_entity_name);
    const auto second_entity = getEntity(second_entity_name);
    return math::geometry::checkCollision2D(
      first_entity->getMapPose(), first_entity->getBoundingBox(), second_entity->getMapPose(),
      second_entity->getBoundingBox());
  } else {
    return false;
  }
}

auto API::laneletRelativeYaw(
  const std::string & entity_name, const LaneletPose & lanelet_pose) const -> std::optional<double>
{
  if (
    const auto relative_pose =
      pose::relativePose(getEntity(entity_name)->getMapPose(), pose::toMapPose(lanelet_pose))) {
    return math::geometry::convertQuaternionToEulerAngle(relative_pose.value().orientation).z;
  } else {
    return std::nullopt;
  }
}

auto API::timeHeadway(const std::string & from_entity_name, const std::string & to_entity_name)
  -> std::optional<double>
{
  if (from_entity_name != to_entity_name) {
    const auto from_entity = getEntity(from_entity_name);
    const auto to_entity = getEntity(to_entity_name);
    if (auto relative_pose = pose::relativePose(from_entity->getMapPose(), to_entity->getMapPose());
        relative_pose && relative_pose->position.x <= 0) {
      const double time_headway =
        (relative_pose->position.x * -1) / to_entity->getCurrentTwist().linear.x;
      return std::isnan(time_headway) ? std::numeric_limits<double>::infinity() : time_headway;
    }
  }
  return std::nullopt;
}

auto API::boundingBoxDistance(
  const std::string & from_entity_name, const std::string & to_entity_name) -> std::optional<double>
{
  if (from_entity_name != to_entity_name) {
    const auto from_entity = getEntity(from_entity_name);
    const auto to_entity = getEntity(to_entity_name);
    return distance::boundingBoxDistance(
      from_entity->getMapPose(), from_entity->getBoundingBox(), to_entity->getMapPose(),
      to_entity->getBoundingBox());
  } else {
    return std::nullopt;
  }
}

auto API::boundingBoxRelativePose(
  const std::string & from_entity_name, const geometry_msgs::msg::Pose & to_map_pose)
  -> std::optional<geometry_msgs::msg::Pose>
{
  const auto from_entity = getEntity(from_entity_name);
  return pose::boundingBoxRelativePose(
    from_entity->getMapPose(), from_entity->getBoundingBox(), to_map_pose,
    traffic_simulator_msgs::msg::BoundingBox());
}

auto API::boundingBoxRelativePose(
  const std::string & from_entity_name, const std::string & to_entity_name)
  -> std::optional<geometry_msgs::msg::Pose>
{
  if (from_entity_name != to_entity_name) {
    const auto from_entity = getEntity(from_entity_name);
    const auto to_entity = getEntity(to_entity_name);
    return pose::boundingBoxRelativePose(
      from_entity->getMapPose(), from_entity->getBoundingBox(), to_entity->getMapPose(),
      to_entity->getBoundingBox());
  } else {
    return std::nullopt;
  }
}

auto API::relativePose(const std::string & from_entity_name, const std::string & to_entity_name)
  -> std::optional<geometry_msgs::msg::Pose>
{
  if (from_entity_name != to_entity_name) {
    const auto from_entity = getEntity(from_entity_name);
    const auto to_entity = getEntity(to_entity_name);
    return pose::relativePose(from_entity->getMapPose(), to_entity->getMapPose());
  } else {
    return std::nullopt;
  }
}

auto API::relativePose(
  const std::string & from_entity_name, const geometry_msgs::msg::Pose & to_map_pose)
  -> std::optional<geometry_msgs::msg::Pose>
{
  const auto from_entity = getEntity(from_entity_name);
  return pose::relativePose(from_entity->getMapPose(), to_map_pose);
}

auto API::relativePose(
  const geometry_msgs::msg::Pose & from_map_pose, const std::string & to_entity_name)
  -> std::optional<geometry_msgs::msg::Pose>
{
  const auto to_entity = getEntity(to_entity_name);
  return pose::relativePose(from_map_pose, to_entity->getMapPose());
}

auto API::relativeSpeed(const std::string & from_entity_name, const std::string & to_entity_name)
  -> Eigen::Vector3d
{
  auto velocity = [](const auto & entity) -> Eigen::Vector3d {
    auto direction = [](const auto & q) -> Eigen::Vector3d {
      return Eigen::Quaternion(q.w, q.x, q.y, q.z) * Eigen::Vector3d::UnitX();
    };
    return direction(entity->getMapPose().orientation) * entity->getCurrentTwist().linear.x;
  };

  const auto observer = getEntity(from_entity_name);
  const auto observed = getEntity(to_entity_name);
  const Eigen::Matrix3d rotation =
    math::geometry::getRotationMatrix(observer->getMapPose().orientation);
  return rotation.transpose() * velocity(observed) - rotation.transpose() * velocity(observer);
}

auto API::countLaneChanges(
  const std::string & from_entity_name, const std::string & to_entity_name,
  const RoutingConfiguration & routing_configuration) const -> std::optional<std::pair<int, int>>
{
  if (from_entity_name != to_entity_name) {
    const auto from_entity = getEntity(from_entity_name);
    const auto to_entity = getEntity(to_entity_name);
    return traffic_simulator::distance::countLaneChanges(
      from_entity->getCanonicalizedLaneletPose().value(),
      to_entity->getCanonicalizedLaneletPose().value(), routing_configuration, getHdmapUtils());
  } else {
    return std::nullopt;
  }
}

auto API::laneletDistance(
  const std::string & from_entity_name, const std::string & to_entity_name,
  const RoutingConfiguration & routing_configuration) -> LaneletDistance
{
  const auto from_entity = getEntity(from_entity_name);
  const auto to_entity = getEntity(to_entity_name);
  if (from_entity_name != to_entity_name) {
    if (from_entity->isInLanelet() && to_entity->isInLanelet()) {
      return distance::laneletDistance(
        from_entity->getCanonicalizedLaneletPose().value(),
        to_entity->getCanonicalizedLaneletPose().value(), routing_configuration, getHdmapUtils());
    }
  }
  return LaneletDistance();
}

auto API::laneletDistance(
  const std::string & from_entity_name, const LaneletPose & to_lanelet_pose,
  const RoutingConfiguration & routing_configuration) -> LaneletDistance
{
  const auto canonicalized_lanelet_pose = CanonicalizedLaneletPose(to_lanelet_pose);
  const auto from_entity = getEntity(from_entity_name);
  if (from_entity->isInLanelet()) {
    return distance::laneletDistance(
      from_entity->getCanonicalizedLaneletPose().value(), canonicalized_lanelet_pose,
      routing_configuration, getHdmapUtils());
  } else {
    return LaneletDistance();
  }
}

auto API::laneletDistance(
  const LaneletPose & from_lanelet_pose, const std::string & to_entity_name,
  const RoutingConfiguration & routing_configuration) -> LaneletDistance
{
  const auto canonicalized_lanelet_pose = CanonicalizedLaneletPose(from_lanelet_pose);
  const auto to_entity = getEntity(to_entity_name);
  if (to_entity->isInLanelet()) {
    return distance::laneletDistance(
      canonicalized_lanelet_pose, to_entity->getCanonicalizedLaneletPose().value(),
      routing_configuration, getHdmapUtils());
  } else {
    return LaneletDistance();
  }
}

auto API::boundingBoxLaneletDistance(
  const std::string & from_entity_name, const std::string & to_entity_name,
  const RoutingConfiguration & routing_configuration) -> LaneletDistance
{
  const auto from_entity = getEntity(from_entity_name);
  const auto to_entity = getEntity(to_entity_name);
  if (from_entity_name != to_entity_name) {
    if (from_entity->isInLanelet() && to_entity->isInLanelet()) {
      return distance::boundingBoxLaneletDistance(
        from_entity->getCanonicalizedLaneletPose().value(), from_entity->getBoundingBox(),
        to_entity->getCanonicalizedLaneletPose().value(), to_entity->getBoundingBox(),
        routing_configuration, getHdmapUtils());
    }
  }
  return LaneletDistance();
}

auto API::boundingBoxLaneletDistance(
  const std::string & from_entity_name, const LaneletPose & to_lanelet_pose,
  const RoutingConfiguration & routing_configuration) -> LaneletDistance
{
  const auto canonicalized_lanelet_pose = CanonicalizedLaneletPose(to_lanelet_pose);
  const auto from_entity = getEntity(from_entity_name);
  if (from_entity->isInLanelet()) {
    return distance::boundingBoxLaneletDistance(
      from_entity->getCanonicalizedLaneletPose().value(), from_entity->getBoundingBox(),
      canonicalized_lanelet_pose, traffic_simulator_msgs::msg::BoundingBox(), routing_configuration,
      getHdmapUtils());
  } else {
    return LaneletDistance();
  }
}

// traffics, lanelet
auto API::getHdmapUtils() const -> const std::shared_ptr<hdmap_utils::HdMapUtils> &
{
  return entity_manager_ptr_->getHdmapUtils();
}

auto API::getV2ITrafficLights() const -> std::shared_ptr<V2ITrafficLights>
{
  return traffic_lights_ptr_->getV2ITrafficLights();
}

auto API::getConventionalTrafficLights() const -> std::shared_ptr<ConventionalTrafficLights>
{
  return traffic_lights_ptr_->getConventionalTrafficLights();
}

auto API::addTrafficSource(
  const double radius, const double rate, const double speed, const geometry_msgs::msg::Pose & pose,
  const traffic::TrafficSource::Distribution & distribution, const bool allow_spawn_outside_lane,
  const bool require_footprint_fitting, const bool random_orientation, std::optional<int> seed)
  -> void
{
  traffic::TrafficSource::Configuration configuration;
  configuration.allow_spawn_outside_lane = allow_spawn_outside_lane;
  configuration.require_footprint_fitting = require_footprint_fitting;
  configuration.use_random_orientation = random_orientation;

  traffic_controller_ptr_->addModule<traffic::TrafficSource>(
    radius, rate, pose, distribution, seed, getCurrentTime(), configuration,
    entity_manager_ptr_->getHdmapUtils(), [this, speed](const auto & name, auto &&... xs) {
      this->spawn(name, std::forward<decltype(xs)>(xs)...);
      getEntity(name)->setLinearVelocity(speed);
    });
}
}  // namespace traffic_simulator
