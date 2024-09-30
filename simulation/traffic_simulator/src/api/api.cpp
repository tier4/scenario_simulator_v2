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
void API::setVerbose(const bool verbose) { entity_manager_ptr_->setVerbose(verbose); }

bool API::despawn(const std::string & name)
{
  const auto result = entity_manager_ptr_->despawnEntity(name);
  if (!result) {
    return false;
  }
  if (not configuration.standalone_mode) {
    simulation_api_schema::DespawnEntityRequest req;
    req.set_name(name);
    return zeromq_client_.call(req).result().success();
  }
  return true;
}

bool API::despawnEntities()
{
  auto entities = getEntityNames();
  return std::all_of(
    entities.begin(), entities.end(), [&](const auto & entity) { return despawn(entity); });
}

auto API::respawn(
  const std::string & name, const geometry_msgs::msg::PoseWithCovarianceStamped & new_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose) -> void
{
  if (not entity_manager_ptr_->is<entity::EgoEntity>(name)) {
    throw std::runtime_error("Respawn of any entities other than EGO is not supported.");
  } else if (new_pose.header.frame_id != "map") {
    throw std::runtime_error("Respawn request with frame id other than map not supported.");
  } else {
    // set new pose and default action status in EntityManager
    entity_manager_ptr_->setControlledBySimulator(name, true);
    setEntityStatus(name, new_pose.pose.pose, helper::constructActionStatus());

    // read status from EntityManager, then send it to SimpleSensorSimulator
    simulation_api_schema::UpdateEntityStatusRequest req;
    simulation_interface::toProto(
      static_cast<EntityStatus>(entity_manager_ptr_->getEntityStatus(name)), *req.add_status());
    req.set_npc_logic_started(entity_manager_ptr_->isNpcLogicStarted());
    req.set_overwrite_ego_status(entity_manager_ptr_->isControlledBySimulator(name));
    entity_manager_ptr_->setControlledBySimulator(name, false);

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
      auto entity_status = static_cast<EntityStatus>(entity_manager_ptr_->getEntityStatus(name));
      simulation_interface::toMsg(res_status->pose(), entity_status.pose);
      simulation_interface::toMsg(res_status->action_status(), entity_status.action_status);
      setMapPose(name, entity_status.pose);
      setTwist(name, entity_status.action_status.twist);
      setAcceleration(name, entity_status.action_status.accel);

      entity_manager_ptr_->asFieldOperatorApplication(name).clearRoute();
      entity_manager_ptr_->asFieldOperatorApplication(name).plan({goal_pose});
      entity_manager_ptr_->asFieldOperatorApplication(name).engage();
    }
  }
}

auto API::getEntity(const std::string & name) const -> std::shared_ptr<entity::EntityBase>
{
  return entity_manager_ptr_->getEntity(name);
}

auto API::setEntityStatus(
  const std::string & name, const CanonicalizedLaneletPose & canonicalized_lanelet_pose,
  const traffic_simulator_msgs::msg::ActionStatus & action_status) -> void
{
  if (const auto entity = getEntity(name)) {
    auto status = static_cast<EntityStatus>(entity->getCanonicalizedStatus());
    status.action_status = action_status;
    status.pose = static_cast<geometry_msgs::msg::Pose>(canonicalized_lanelet_pose);
    status.lanelet_pose = static_cast<LaneletPose>(canonicalized_lanelet_pose);
    status.lanelet_pose_valid = true;
    entity->setCanonicalizedStatus(CanonicalizedEntityStatus(status, canonicalized_lanelet_pose));
  } else {
    THROW_SIMULATION_ERROR("Cannot set entity \"", name, "\" status - such entity does not exist.");
  }
}

auto API::setEntityStatus(const std::string & name, const EntityStatus & status) -> void
{
  if (const auto entity = getEntity(name)) {
    entity->setStatus(status);
  } else {
    THROW_SIMULATION_ERROR("Cannot set entity \"", name, "\" status - such entity does not exist.");
  }
}

/// @todo it probably should be moved to SimulatorCore
std::optional<double> API::getTimeHeadway(
  const std::string & from_entity_name, const std::string & to_entity_name)
{
  if (auto from_entity = getEntity(from_entity_name); from_entity) {
    if (auto to_entity = getEntity(to_entity_name); to_entity) {
      if (auto relative_pose =
            pose::relativePose(from_entity->getMapPose(), to_entity->getMapPose());
          relative_pose && relative_pose->position.x <= 0) {
        const double time_headway =
          (relative_pose->position.x * -1) / getCurrentTwist(to_entity_name).linear.x;
        return std::isnan(time_headway) ? std::numeric_limits<double>::infinity() : time_headway;
      }
    }
  }
  return std::nullopt;
}

auto API::setEntityStatus(
  const std::string & name, const LaneletPose & lanelet_pose,
  const traffic_simulator_msgs::msg::ActionStatus & action_status) -> void
{
  if (
    const auto canonicalized_lanelet_pose =
      pose::canonicalize(lanelet_pose, entity_manager_ptr_->getHdmapUtils())) {
    setEntityStatus(name, canonicalized_lanelet_pose.value(), action_status);
  } else {
    std::stringstream ss;
    ss << "Status can not be set. lanelet pose: " << lanelet_pose
       << " cannot be canonicalized for ";
    THROW_SEMANTIC_ERROR(ss.str(), " entity named: ", std::quoted(name), ".");
  }
}

auto API::setEntityStatus(
  const std::string & name, const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::ActionStatus & action_status) -> void
{
  if (const auto entity = getEntity(name)) {
    EntityStatus status = static_cast<EntityStatus>(entity->getCanonicalizedStatus());
    status.pose = map_pose;
    status.action_status = action_status;
    status.lanelet_pose_valid = false;
    setEntityStatus(name, status);
  } else {
    THROW_SIMULATION_ERROR("Cannot set entity \"", name, "\" status - such entity does not exist.");
  }
}

auto API::setEntityStatus(
  const std::string & name, const std::string & reference_entity_name,
  const geometry_msgs::msg::Pose & relative_pose,
  const traffic_simulator_msgs::msg::ActionStatus & action_status) -> void
{
  if (const auto reference_entity = getEntity(reference_entity_name)) {
    setEntityStatus(
      name, pose::transformRelativePoseToGlobal(reference_entity->getMapPose(), relative_pose),
      action_status);
  } else {
    THROW_SIMULATION_ERROR(
      "Cannot get entity \"", reference_entity_name, "\" - such entity does not exist.");
  }
}

auto API::setEntityStatus(
  const std::string & name, const std::string & reference_entity_name,
  const geometry_msgs::msg::Point & relative_position,
  const geometry_msgs::msg::Vector3 & relative_rpy,
  const traffic_simulator_msgs::msg::ActionStatus & action_status) -> void
{
  const auto relative_pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(relative_position)
      .orientation(math::geometry::convertEulerAngleToQuaternion(relative_rpy));
  setEntityStatus(name, reference_entity_name, relative_pose, action_status);
}

auto API::attachImuSensor(
  const std::string &, const simulation_api_schema::ImuSensorConfiguration & configuration) -> bool
{
  simulation_api_schema::AttachImuSensorRequest req;
  *req.mutable_configuration() = configuration;
  return zeromq_client_.call(req).result().success();
}

bool API::attachPseudoTrafficLightDetector(
  const simulation_api_schema::PseudoTrafficLightDetectorConfiguration & configuration)
{
  simulation_api_schema::AttachPseudoTrafficLightDetectorRequest req;
  *req.mutable_configuration() = configuration;
  return zeromq_client_.call(req).result().success();
}

bool API::attachDetectionSensor(
  const simulation_api_schema::DetectionSensorConfiguration & sensor_configuration)
{
  if (configuration.standalone_mode) {
    return true;
  } else {
    simulation_api_schema::AttachDetectionSensorRequest req;
    *req.mutable_configuration() = sensor_configuration;
    return zeromq_client_.call(req).result().success();
  }
}

bool API::attachDetectionSensor(
  const std::string & entity_name, double detection_sensor_range, bool detect_all_objects_in_range,
  double pos_noise_stddev, int random_seed, double probability_of_lost,
  double object_recognition_delay)
{
  return attachDetectionSensor(helper::constructDetectionSensorConfiguration(
    entity_name, getROS2Parameter<std::string>("architecture_type", "awf/universe"), 0.1,
    detection_sensor_range, detect_all_objects_in_range, pos_noise_stddev, random_seed,
    probability_of_lost, object_recognition_delay));
}

bool API::attachOccupancyGridSensor(
  const simulation_api_schema::OccupancyGridSensorConfiguration & sensor_configuration)
{
  if (configuration.standalone_mode) {
    return true;
  } else {
    simulation_api_schema::AttachOccupancyGridSensorRequest req;
    *req.mutable_configuration() = sensor_configuration;
    return zeromq_client_.call(req).result().success();
  }
}

bool API::attachLidarSensor(const simulation_api_schema::LidarConfiguration & lidar_configuration)
{
  if (configuration.standalone_mode) {
    return true;
  } else {
    simulation_api_schema::AttachLidarSensorRequest req;
    *req.mutable_configuration() = lidar_configuration;
    return zeromq_client_.call(req).result().success();
  }
}

bool API::attachLidarSensor(
  const std::string & entity_name, const double lidar_sensor_delay,
  const helper::LidarType lidar_type)
{
  return attachLidarSensor(helper::constructLidarConfiguration(
    lidar_type, entity_name, getROS2Parameter<std::string>("architecture_type", "awf/universe"),
    lidar_sensor_delay));
}

bool API::updateTimeInSim()
{
  simulation_api_schema::UpdateFrameRequest request;
  request.set_current_simulation_time(clock_.getCurrentSimulationTime());
  request.set_current_scenario_time(getCurrentTime());
  simulation_interface::toProto(
    clock_.getCurrentRosTimeAsMsg().clock, *request.mutable_current_ros_time());
  return zeromq_client_.call(request).result().success();
}

bool API::updateTrafficLightsInSim()
{
  if (entity_manager_ptr_->trafficLightsChanged()) {
    auto req = entity_manager_ptr_->generateUpdateRequestForConventionalTrafficLights();
    return zeromq_client_.call(req).result().success();
  }
  /// @todo handle response
  return simulation_api_schema::UpdateTrafficLightsResponse().result().success();
}

bool API::updateEntitiesStatusInSim()
{
  simulation_api_schema::UpdateEntityStatusRequest req;
  req.set_npc_logic_started(entity_manager_ptr_->isNpcLogicStarted());
  for (const auto & entity_name : entity_manager_ptr_->getEntityNames()) {
    const auto entity_status =
      static_cast<EntityStatus>(entity_manager_ptr_->getEntityStatus(entity_name));
    simulation_interface::toProto(entity_status, *req.add_status());
    if (entity_manager_ptr_->is<entity::EgoEntity>(entity_name)) {
      req.set_overwrite_ego_status(entity_manager_ptr_->isControlledBySimulator(entity_name));
    }
  }

  simulation_api_schema::UpdateEntityStatusResponse res;
  if (auto res = zeromq_client_.call(req); res.result().success()) {
    for (const auto & res_status : res.status()) {
      auto entity_name = res_status.name();
      auto entity_status =
        static_cast<EntityStatus>(entity_manager_ptr_->getEntityStatus(entity_name));
      simulation_interface::toMsg(res_status.pose(), entity_status.pose);
      simulation_interface::toMsg(res_status.action_status(), entity_status.action_status);

      if (entity_manager_ptr_->is<entity::EgoEntity>(entity_name)) {
        setMapPose(entity_name, entity_status.pose);
        setTwist(entity_name, entity_status.action_status.twist);
        setAcceleration(entity_name, entity_status.action_status.accel);
      } else {
        setEntityStatus(entity_name, entity_status);
      }
    }
    return true;
  }
  return false;
}

bool API::updateFrame()
{
  if (configuration.standalone_mode && entity_manager_ptr_->isEgoSpawned()) {
    THROW_SEMANTIC_ERROR("Ego simulation is no longer supported in standalone mode");
  }

  if (!updateEntitiesStatusInSim()) {
    return false;
  }

  entity_manager_ptr_->update(getCurrentTime(), clock_.getStepTime());
  traffic_controller_ptr_->execute(getCurrentTime(), clock_.getStepTime());

  if (not configuration.standalone_mode) {
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

void API::startNpcLogic()
{
  if (entity_manager_ptr_->isNpcLogicStarted()) {
    THROW_SIMULATION_ERROR("NPC logics are already started.");
  } else {
    clock_.start();
    entity_manager_ptr_->startNpcLogic(getCurrentTime());
  }
}

void API::requestLaneChange(const std::string & name, const lanelet::Id & lanelet_id)
{
  entity_manager_ptr_->requestLaneChange(name, lanelet_id);
}

void API::requestLaneChange(
  const std::string & name, const traffic_simulator::lane_change::Direction & direction)
{
  entity_manager_ptr_->requestLaneChange(name, direction);
}

void API::requestLaneChange(
  const std::string & name, const traffic_simulator::lane_change::Parameter & parameter)
{
  entity_manager_ptr_->requestLaneChange(name, parameter);
}

void API::requestLaneChange(
  const std::string & name, const traffic_simulator::lane_change::RelativeTarget & target,
  const traffic_simulator::lane_change::TrajectoryShape trajectory_shape,
  const lane_change::Constraint & constraint)
{
  entity_manager_ptr_->requestLaneChange(name, target, trajectory_shape, constraint);
}

void API::requestLaneChange(
  const std::string & name, const traffic_simulator::lane_change::AbsoluteTarget & target,
  const traffic_simulator::lane_change::TrajectoryShape trajectory_shape,
  const lane_change::Constraint & constraint)
{
  entity_manager_ptr_->requestLaneChange(name, target, trajectory_shape, constraint);
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
      setLinearVelocity(name, speed);
    });
}
}  // namespace traffic_simulator
