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

#include <tf2/LinearMath/Quaternion.h>

#include <geometry/intersection/collision.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <limits>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <stdexcept>
#include <string>
#include <traffic_simulator/api/api.hpp>
#include <traffic_simulator/traffic/traffic_source.hpp>
#include <traffic_simulator/utils/pose.hpp>

namespace traffic_simulator
{
auto API::setVerbose(const bool verbose) -> void { entity_manager_ptr_->setVerbose(verbose); }

auto API::startNpcLogic() -> void
{
  if (entity_manager_ptr_->isNpcLogicStarted()) {
    THROW_SIMULATION_ERROR("NPC logics are already started.");
  } else {
    clock_.start();
    entity_manager_ptr_->startNpcLogic(getCurrentTime());
  }
}

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

auto API::getEntity(const std::string & name) -> entity::EntityBase &
{
  return entity_manager_ptr_->getEntity(name);
}

auto API::getEntity(const std::string & name) const -> const entity::EntityBase &
{
  return entity_manager_ptr_->getEntity(name);
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
    ego_entity.setStatus(new_pose.pose.pose, helper::constructActionStatus());

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

auto API::checkCollision(
  const std::string & first_entity_name, const std::string & second_entity_name) -> bool
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

auto API::addTrafficSource(
  const double radius, const double rate, const double speed, const geometry_msgs::msg::Pose & pose,
  const traffic::TrafficSource::Distribution & distribution, const bool allow_spawn_outside_lane,
  const bool require_footprint_fitting, const bool random_orientation, std::optional<int> seed)
  -> void
{
  traffic_simulator::traffic::TrafficSource::Configuration configuration;
  configuration.allow_spawn_outside_lane = allow_spawn_outside_lane;
  configuration.require_footprint_fitting = require_footprint_fitting;
  configuration.use_random_orientation = random_orientation;

  traffic_controller_ptr_->addModule<traffic_simulator::traffic::TrafficSource>(
    radius, rate, pose, distribution, seed, getCurrentTime(), configuration,
    entity_manager_ptr_->getHdmapUtils(), [this, speed](const auto & name, auto &&... xs) {
      this->spawn(name, std::forward<decltype(xs)>(xs)...);
      getEntity(name).setLinearVelocity(speed);
    });
}
}  // namespace traffic_simulator
