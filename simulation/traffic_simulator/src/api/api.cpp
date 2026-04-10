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
#include <osi_interface/osi_entity_conversions.hpp>
#include <osi_interface/osi_traffic_light_conversions.hpp>
#include <traffic_simulator/api/api.hpp>

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
    const auto & entity = entity_manager_ptr_->getEntity(entity_name);
    const auto entity_status = static_cast<EntityStatus>(entity.getCanonicalizedStatus());
    simulation_interface::toProto(entity_status, *req.add_status());
    if (entity.is<entity::EgoEntity>()) {
      req.set_overwrite_ego_status(entity.isControlledBySimulator());
    }
  }

  simulation_api_schema::UpdateEntityStatusResponse res;
  if (auto res = zeromq_client_.call(req); res.result().success()) {
    for (const auto & res_status : res.status()) {
      auto & entity = entity_manager_ptr_->getEntity(res_status.name());
      auto entity_status = static_cast<EntityStatus>(entity.getCanonicalizedStatus());
      simulation_interface::toMsg(res_status.pose(), entity_status.pose);
      simulation_interface::toMsg(res_status.action_status(), entity_status.action_status);

      if (entity.is<entity::EgoEntity>()) {
        entity.setMapPose(entity_status.pose);
        entity.setTwist(entity_status.action_status.twist);
        entity.setAcceleration(entity_status.action_status.accel);
      } else {
        entity.setStatus(entity_status);
      }
    }
    return true;
  }
  return false;
}

auto API::updateTrafficLightsInSim() -> bool
{
  if (traffic_lights_ptr_->isAnyTrafficLightChanged()) {
    const auto request = traffic_lights_ptr_->generateConventionalUpdateRequest();
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

  if (use_osi_protocol_) {
    if (!updateFrameOsi()) {
      return false;
    }
  } else {
    if (!updateEntitiesStatusInSim()) {
      return false;
    }
  }

  entity_manager_ptr_->update(getCurrentTime(), clock_.getStepTime());
  traffic_controller_ptr_->execute(getCurrentTime(), clock_.getStepTime());

  if (not configuration_.standalone_mode && not use_osi_protocol_) {
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

auto API::buildGroundTruth() -> osi3::GroundTruth
{
  std::vector<osi_interface::EntityData> moving_entities;
  std::vector<osi_interface::EntityData> stationary_entities;
  std::string ego_name;

  for (const auto & entity_name : entity_manager_ptr_->getEntityNames()) {
    const auto & entity = entity_manager_ptr_->getEntity(entity_name);
    const auto entity_status = static_cast<EntityStatus>(entity.getCanonicalizedStatus());

    osi_interface::EntityData data;
    data.name = entity_name;
    data.pose = {entity_status.pose.position.x,    entity_status.pose.position.y,
                 entity_status.pose.position.z,    entity_status.pose.orientation.x,
                 entity_status.pose.orientation.y, entity_status.pose.orientation.z,
                 entity_status.pose.orientation.w};
    data.twist = {
      entity_status.action_status.twist.linear.x,  entity_status.action_status.twist.linear.y,
      entity_status.action_status.twist.linear.z,  entity_status.action_status.twist.angular.x,
      entity_status.action_status.twist.angular.y, entity_status.action_status.twist.angular.z};
    data.accel = {
      entity_status.action_status.accel.linear.x,  entity_status.action_status.accel.linear.y,
      entity_status.action_status.accel.linear.z,  entity_status.action_status.accel.angular.x,
      entity_status.action_status.accel.angular.y, entity_status.action_status.accel.angular.z};
    const auto & bb = entity.getBoundingBox();
    data.bounding_box = {bb.center.x,     bb.center.y,     bb.center.z,
                         bb.dimensions.x, bb.dimensions.y, bb.dimensions.z};

    if (entity.is<entity::EgoEntity>()) {
      data.type = osi_interface::EntityType::EGO;
      data.subtype = osi_interface::EntitySubtype::CAR;
      ego_name = entity_name;
      moving_entities.push_back(data);
    } else if (entity.is<entity::VehicleEntity>()) {
      data.type = osi_interface::EntityType::VEHICLE;
      data.subtype = osi_interface::EntitySubtype::CAR;
      moving_entities.push_back(data);
    } else if (entity.is<entity::PedestrianEntity>()) {
      data.type = osi_interface::EntityType::PEDESTRIAN;
      data.subtype = osi_interface::EntitySubtype::PEDESTRIAN;
      moving_entities.push_back(data);
    } else {
      data.type = osi_interface::EntityType::MISC_OBJECT;
      stationary_entities.push_back(data);
    }
  }

  // Traffic lights
  std::vector<osi_interface::TrafficSignalGroup> traffic_signals;
  if (traffic_lights_ptr_->isAnyTrafficLightChanged()) {
    const auto request = traffic_lights_ptr_->generateConventionalUpdateRequest();
    for (const auto & state : request.states()) {
      osi_interface::TrafficSignalGroup signal;
      signal.lanelet_id = state.id();
      for (const auto & rel_id : state.relation_ids()) {
        signal.relation_ids.push_back(rel_id);
      }
      for (const auto & tl : state.traffic_light_status()) {
        osi_interface::TrafficLightBulb bulb;
        bulb.color = static_cast<osi_interface::TrafficLightBulb::Color>(tl.color());
        bulb.shape = static_cast<osi_interface::TrafficLightBulb::Shape>(tl.shape());
        bulb.status = static_cast<osi_interface::TrafficLightBulb::Status>(tl.status());
        bulb.confidence = tl.confidence();
        signal.bulbs.push_back(bulb);
      }
      traffic_signals.push_back(signal);
    }
  }

  return osi_interface::GroundTruthBuilder(entity_id_registry_)
    .setTimestamp(clock_.getCurrentSimulationTime())
    .setHostVehicle(ego_name)
    .setMapReference(configuration_.lanelet2_map_path().string())
    .setMovingEntities(moving_entities)
    .setStationaryEntities(stationary_entities)
    .setTrafficSignals(traffic_signals)
    .build();
}

auto API::updateFrameOsi() -> bool
{
  auto gt = buildGroundTruth();
  auto tu = osi_client_->sendGroundTruth(gt);
  applyTrafficUpdate(tu);
  return true;
}

auto API::applyTrafficUpdate(const osi3::TrafficUpdate & tu) -> void
{
  for (int i = 0; i < tu.update_size(); ++i) {
    const auto & obj = tu.update(i);
    auto name_opt = entity_id_registry_.reverseLookup(obj.id().value());
    if (!name_opt.has_value()) {
      continue;
    }
    const auto & name = *name_opt;
    if (!entity_manager_ptr_->isEntityExist(name)) {
      continue;
    }

    auto & entity = entity_manager_ptr_->getEntity(name);

    // Apply only for ego entities (TPM controls ego dynamics)
    if (entity.is<entity::EgoEntity>() && !entity.isControlledBySimulator()) {
      auto entity_data = osi_interface::fromOsiMovingObject(obj, entity_id_registry_);

      geometry_msgs::msg::Pose pose;
      pose.position.x = entity_data.pose.x;
      pose.position.y = entity_data.pose.y;
      pose.position.z = entity_data.pose.z;
      pose.orientation.x = entity_data.pose.qx;
      pose.orientation.y = entity_data.pose.qy;
      pose.orientation.z = entity_data.pose.qz;
      pose.orientation.w = entity_data.pose.qw;
      entity.setMapPose(pose);

      geometry_msgs::msg::Twist twist;
      twist.linear.x = entity_data.twist.linear_x;
      twist.linear.y = entity_data.twist.linear_y;
      twist.linear.z = entity_data.twist.linear_z;
      entity.setTwist(twist);

      geometry_msgs::msg::Accel accel;
      accel.linear.x = entity_data.accel.linear_x;
      accel.linear.y = entity_data.accel.linear_y;
      accel.linear.z = entity_data.accel.linear_z;
      entity.setAcceleration(accel);
    }
  }
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
    entity_name, getROS2Parameter<std::string>("architecture_type", "awf/universe/20240605"), 0.1,
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

auto API::getFirstEgoName() const -> std::optional<std::string>
{
  return entity_manager_ptr_->getFirstEgoName();
}

auto API::getEgoEntity(const std::string & name) -> entity::EgoEntity &
{
  return entity_manager_ptr_->getEgoEntity(name);
}

auto API::getEgoEntity(const std::string & name) const -> const entity::EgoEntity &
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

auto API::getEntity(const std::string & name) -> entity::EntityBase &
{
  return entity_manager_ptr_->getEntity(name);
}

auto API::getEntity(const std::string & name) const -> const entity::EntityBase &
{
  return entity_manager_ptr_->getEntity(name);
}

auto API::getEntityPointer(const std::string & name) const -> std::shared_ptr<entity::EntityBase>
{
  return entity_manager_ptr_->getEntityPointer(name);
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
    auto & ego_entity = entity_manager_ptr_->getEgoEntity(name);
    // set new pose and default action status in EntityManager
    ego_entity.setControlledBySimulator(true);
    ego_entity.setStatus(new_pose.pose.pose);

    // read status from EntityManager, then send it to SimpleSensorSimulator
    simulation_api_schema::UpdateEntityStatusRequest req;
    simulation_interface::toProto(
      static_cast<EntityStatus>(entity_manager_ptr_->getEntity(name).getCanonicalizedStatus()),
      *req.add_status());
    req.set_npc_logic_started(entity_manager_ptr_->isNpcLogicStarted());
    req.set_overwrite_ego_status(ego_entity.isControlledBySimulator());
    ego_entity.setControlledBySimulator(false);

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
      auto entity_status = static_cast<EntityStatus>(ego_entity.getCanonicalizedStatus());
      simulation_interface::toMsg(res_status->pose(), entity_status.pose);
      simulation_interface::toMsg(res_status->action_status(), entity_status.action_status);
      ego_entity.setMapPose(entity_status.pose);
      ego_entity.setTwist(entity_status.action_status.twist);
      ego_entity.setAcceleration(entity_status.action_status.accel);
      ego_entity.requestReplanRoute({goal_pose});
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
  if (
    first_entity_name != second_entity_name && isEntityExist(first_entity_name) &&
    isEntityExist(second_entity_name)) {
    const auto & first_entity = getEntity(first_entity_name);
    const auto & second_entity = getEntity(second_entity_name);
    return math::geometry::checkCollision2D(
      first_entity.getMapPose(), first_entity.getBoundingBox(), second_entity.getMapPose(),
      second_entity.getBoundingBox());
  } else {
    return false;
  }
}

// traffics, lanelet
auto API::getV2ITrafficLights() const -> std::shared_ptr<V2ITrafficLights>
{
  return traffic_lights_ptr_->getV2ITrafficLights();
}

auto API::getConventionalTrafficLights() const -> std::shared_ptr<ConventionalTrafficLights>
{
  return traffic_lights_ptr_->getConventionalTrafficLights();
}

auto API::getV2IDetectedTrafficLights() const -> std::shared_ptr<DetectedTrafficLights>
{
  return traffic_lights_ptr_->getV2IDetectedTrafficLights();
}

auto API::getConventionalDetectedTrafficLights() const -> std::shared_ptr<DetectedTrafficLights>
{
  return traffic_lights_ptr_->getConventionalDetectedTrafficLights();
}

auto API::setTrafficSignalV2IFeature(const lanelet::Id lanelet_id, const bool enabled) -> void
{
  traffic_lights_ptr_->setV2IFeature(lanelet_id, enabled);
}

auto API::isV2ITrafficLightEnabled(const lanelet::Id lanelet_id) const -> bool
{
  return traffic_lights_ptr_->isV2ITrafficLightEnabled(lanelet_id);
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
    [this, speed](const auto & name, auto &&... xs) {
      this->spawn(name, std::forward<decltype(xs)>(xs)...);
      getEntity(name).setLinearVelocity(speed);
    });
}
}  // namespace traffic_simulator
