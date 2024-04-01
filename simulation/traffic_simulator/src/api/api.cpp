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

#include <limits>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <stdexcept>
#include <string>
#include <traffic_simulator/api/api.hpp>

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

auto API::setEntityStatus(const std::string & name, const CanonicalizedEntityStatus & status)
  -> void
{
  entity_manager_ptr_->setEntityStatus(name, status);
}

auto API::setEntityStatus(
  const std::string & name, const std::string & reference_entity_name,
  const geometry_msgs::msg::Point & relative_position,
  const geometry_msgs::msg::Vector3 & relative_rpy,
  const traffic_simulator_msgs::msg::ActionStatus & action_status) -> void
{
  geometry_msgs::msg::Pose relative_pose;
  relative_pose.position = relative_position;
  relative_pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(relative_rpy);
  setEntityStatus(name, reference_entity_name, relative_pose, action_status);
}

auto API::setEntityStatus(
  const std::string & name, const std::string & reference_entity_name,
  const geometry_msgs::msg::Pose & relative_pose,
  const traffic_simulator_msgs::msg::ActionStatus & action_status) -> void
{
  EntityStatus status;
  status.time = getCurrentTime();
  status.pose =
    entity_manager_ptr_->getMapPoseFromRelativePose(reference_entity_name, relative_pose);
  status.action_status = action_status;
  if (
    const auto lanelet_pose = entity_manager_ptr_->toLaneletPose(
      status.pose, getBoundingBox(name), false,
      getDefaultMatchingDistanceForLaneletPoseCalculation(name))) {
    status.lanelet_pose_valid = true;
    status.lanelet_pose = lanelet_pose.value();
  } else {
    status.lanelet_pose_valid = false;
    status.lanelet_pose = traffic_simulator::LaneletPose();
  }
  setEntityStatus(name, canonicalize(status));
}

std::optional<double> API::getTimeHeadway(const std::string & from, const std::string & to)
{
  geometry_msgs::msg::Pose pose = getRelativePose(from, to);
  if (pose.position.x > 0) {
    return std::nullopt;
  }
  double ret = (pose.position.x * -1) / (getCurrentTwist(to).linear.x);
  if (std::isnan(ret)) {
    return std::numeric_limits<double>::infinity();
  }
  return ret;
}

auto API::setEntityStatus(
  const std::string & name, const CanonicalizedLaneletPose & lanelet_pose,
  const traffic_simulator_msgs::msg::ActionStatus & action_status) -> void
{
  EntityStatus status;
  status.time = getCurrentTime();
  status.bounding_box = getBoundingBox(name);
  status.pose = entity_manager_ptr_->toMapPose(lanelet_pose);
  status.name = name;
  status.action_status = action_status;
  status.lanelet_pose = static_cast<LaneletPose>(lanelet_pose);
  status.lanelet_pose_valid = true;
  setEntityStatus(name, canonicalize(status));
}

auto API::setEntityStatus(
  const std::string & name, const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::ActionStatus & action_status) -> void
{
  EntityStatus status;
  status.time = getCurrentTime();
  status.bounding_box = getBoundingBox(name);
  status.pose = map_pose;
  status.name = name;
  status.action_status = action_status;
  if (
    const auto lanelet_pose = entity_manager_ptr_->toLaneletPose(
      map_pose, getBoundingBox(name), false,
      getDefaultMatchingDistanceForLaneletPoseCalculation(name))) {
    status.lanelet_pose = lanelet_pose.value();
  } else {
    status.lanelet_pose_valid = false;
  }
  setEntityStatus(name, canonicalize(status));
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
    entity_name, getParameter<std::string>("architecture_type", "awf/universe"), 0.1,
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
    lidar_type, entity_name, getParameter<std::string>("architecture_type", "awf/universe"),
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
    auto entity_status = entity_manager_ptr_->getEntityStatus(entity_name);
    simulation_interface::toProto(static_cast<EntityStatus>(entity_status), *req.add_status());
    if (entity_manager_ptr_->is<entity::EgoEntity>(entity_name)) {
      req.set_overwrite_ego_status(entity_manager_ptr_->isControlledBySimulator(entity_name));
    }
  }

  simulation_api_schema::UpdateEntityStatusResponse res;
  if (auto res = zeromq_client_.call(req); res.result().success()) {
    for (const auto & res_status : res.status()) {
      auto name = res_status.name();
      auto entity_status = static_cast<EntityStatus>(entity_manager_ptr_->getEntityStatus(name));
      simulation_interface::toMsg(res_status.pose(), entity_status.pose);
      simulation_interface::toMsg(res_status.action_status(), entity_status.action_status);

      if (entity_manager_ptr_->is<entity::EgoEntity>(name)) {
        setMapPose(name, entity_status.pose);
        setTwist(name, entity_status.action_status.twist);
        setAcceleration(name, entity_status.action_status.accel);
      } else {
        setEntityStatus(name, canonicalize(entity_status));
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
  traffic_controller_ptr_->execute();

  if (not configuration.standalone_mode) {
    if (!updateTrafficLightsInSim() || !updateTimeInSim()) {
      return false;
    }
  }

  entity_manager_ptr_->broadcastEntityTransform();
  clock_.update();
  clock_pub_->publish(clock_.getCurrentRosTimeAsMsg());
  debug_marker_pub_->publish(entity_manager_ptr_->makeDebugMarker());
  return true;
}

void API::startNpcLogic()
{
  if (isNpcLogicStarted()) {
    THROW_SIMULATION_ERROR("NPC logics are already started.");
  }
  entity_manager_ptr_->startNpcLogic();
  clock_.start();
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

auto API::canonicalize(const LaneletPose & may_non_canonicalized_lanelet_pose) const
  -> CanonicalizedLaneletPose
{
  return CanonicalizedLaneletPose(
    may_non_canonicalized_lanelet_pose, entity_manager_ptr_->getHdmapUtils());
}

auto API::canonicalize(const EntityStatus & may_non_canonicalized_entity_status) const
  -> CanonicalizedEntityStatus
{
  return CanonicalizedEntityStatus(
    may_non_canonicalized_entity_status, entity_manager_ptr_->getHdmapUtils());
}

auto API::toLaneletPose(const geometry_msgs::msg::Pose & map_pose, bool include_crosswalk) const
  -> std::optional<CanonicalizedLaneletPose>
{
  if (
    const auto pose =
      entity_manager_ptr_->getHdmapUtils()->toLaneletPose(map_pose, include_crosswalk)) {
    return canonicalize(pose.value());
  }
  return std::nullopt;
}
}  // namespace traffic_simulator
