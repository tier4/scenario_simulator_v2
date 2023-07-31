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
    simulation_api_schema::DespawnEntityResponse res;
    req.set_name(name);
    zeromq_client_.call(req, res);
    return res.result().success();
  }
  return true;
}

bool API::despawnEntities()
{
  auto entities = getEntityNames();
  return std::all_of(
    entities.begin(), entities.end(), [&](const auto & entity) { return despawn(entity); });
}

geometry_msgs::msg::Pose API::getEntityPose(const std::string & name)
{
  auto status = getEntityStatus(name);
  return status.pose;
}

traffic_simulator_msgs::msg::EntityStatus API::getEntityStatus(const std::string & name)
{
  return entity_manager_ptr_->getEntityStatus(name);
}

auto API::setEntityStatus(
  const std::string & name, const traffic_simulator_msgs::msg::EntityStatus & status) -> void
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
  const auto pose = entity_manager_ptr_->getMapPose(reference_entity_name, relative_pose);
  traffic_simulator_msgs::msg::EntityStatus status;
  status.time = clock_.getCurrentSimulationTime();
  status.pose = pose;
  const auto lanelet_pose = entity_manager_ptr_->toLaneletPose(
    pose, entity_manager_ptr_->getEntityStatus(reference_entity_name).bounding_box, false);
  status.action_status = action_status;
  if (lanelet_pose) {
    status.lanelet_pose_valid = true;
    status.lanelet_pose = lanelet_pose.value();
  } else {
    status.lanelet_pose_valid = false;
  }
  entity_manager_ptr_->setEntityStatus(name, status);
}

std::optional<double> API::getTimeHeadway(const std::string & from, const std::string & to)
{
  geometry_msgs::msg::Pose pose = getRelativePose(from, to);
  if (pose.position.x > 0) {
    return std::nullopt;
  }
  traffic_simulator_msgs::msg::EntityStatus to_status = getEntityStatus(to);
  double ret = (pose.position.x * -1) / (to_status.action_status.twist.linear.x);
  if (std::isnan(ret)) {
    return std::numeric_limits<double>::infinity();
  }
  return ret;
}

bool API::reachPosition(
  const std::string & name, const geometry_msgs::msg::Pose & target_pose, const double tolerance)
{
  return entity_manager_ptr_->reachPosition(name, target_pose, tolerance);
}

bool API::reachPosition(
  const std::string & name, const traffic_simulator_msgs::msg::LaneletPose & target_pose,
  const double tolerance)
{
  return entity_manager_ptr_->reachPosition(
    name, target_pose.lanelet_id, target_pose.s, target_pose.offset, tolerance);
}

bool API::reachPosition(
  const std::string & name, const std::string & target_name, const double tolerance) const
{
  return entity_manager_ptr_->reachPosition(name, target_name, tolerance);
}

auto API::setEntityStatus(
  const std::string & name, const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose,
  const traffic_simulator_msgs::msg::ActionStatus & action_status) -> void
{
  traffic_simulator_msgs::msg::EntityStatus status;
  status.lanelet_pose = lanelet_pose;
  status.lanelet_pose_valid = true;
  status.bounding_box = entity_manager_ptr_->getEntityStatus(name).bounding_box;
  status.pose = entity_manager_ptr_->toMapPose(lanelet_pose);
  status.name = name;
  const auto current_time = getCurrentTime();
  if (std::isnan(current_time)) {
    status.time = current_time;
  } else {
    status.time = 0;
  }
  status.action_status = action_status;
  setEntityStatus(name, status);
}

auto API::setEntityStatus(
  const std::string & name, const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::ActionStatus & action_status) -> void
{
  const auto lanelet_pose = entity_manager_ptr_->toLaneletPose(
    map_pose, entity_manager_ptr_->getEntityStatus(name).bounding_box, false);
  traffic_simulator_msgs::msg::EntityStatus status;
  if (lanelet_pose) {
    status.lanelet_pose = lanelet_pose.value();
  } else {
    status.lanelet_pose_valid = false;
  }
  status.pose = map_pose;
  status.name = name;
  status.action_status = action_status;
  const auto current_time = getCurrentTime();
  if (std::isnan(current_time)) {
    status.time = current_time;
  } else {
    status.time = 0;
  }
  status.bounding_box = entity_manager_ptr_->getEntityStatus(name).bounding_box;
  setEntityStatus(name, status);
}

bool API::initialize(double realtime_factor, double step_time)
{
  clock_.initialize(-1 * configuration.initialize_duration, step_time);

  if (configuration.standalone_mode) {
    return true;
  } else {
    simulation_api_schema::InitializeRequest req;
    req.set_step_time(step_time);
    req.set_realtime_factor(realtime_factor);
    req.set_initialize_time(clock_.getCurrentSimulationTime());
    simulation_interface::toProto(clock_.getCurrentRosTime(), *req.mutable_initialize_ros_time());
    req.set_lanelet2_map_path(configuration.lanelet2_map_path().string());
    simulation_api_schema::InitializeResponse res;
    zeromq_client_.call(req, res);
    return res.result().success();
  }
}

bool API::attachDetectionSensor(
  const simulation_api_schema::DetectionSensorConfiguration & sensor_configuration)
{
  if (configuration.standalone_mode) {
    return true;
  } else {
    simulation_api_schema::AttachDetectionSensorRequest req;
    simulation_api_schema::AttachDetectionSensorResponse res;
    *req.mutable_configuration() = sensor_configuration;
    zeromq_client_.call(req, res);
    return res.result().success();
  }
}

bool API::attachDetectionSensor(
  const std::string & entity_name, double pos_noise_stddev, double probability_of_lost,
  double object_recognition_delay, int random_seed)
{
  return attachDetectionSensor(helper::constructDetectionSensorConfiguration(
    entity_name, getParameter<std::string>("architecture_type", "awf/universe"), 0.1, 300, false,
    pos_noise_stddev, random_seed, probability_of_lost, object_recognition_delay));
}

bool API::attachOccupancyGridSensor(
  const simulation_api_schema::OccupancyGridSensorConfiguration & sensor_configuration)
{
  if (configuration.standalone_mode) {
    return true;
  } else {
    simulation_api_schema::AttachOccupancyGridSensorRequest req;
    simulation_api_schema::AttachOccupancyGridSensorResponse res;
    *req.mutable_configuration() = sensor_configuration;
    zeromq_client_.call(req, res);
    return res.result().success();
  }
}

bool API::attachLidarSensor(const simulation_api_schema::LidarConfiguration & lidar_configuration)
{
  if (configuration.standalone_mode) {
    return true;
  } else {
    simulation_api_schema::AttachLidarSensorRequest req;
    simulation_api_schema::AttachLidarSensorResponse res;
    *req.mutable_configuration() = lidar_configuration;
    zeromq_client_.call(req, res);
    return res.result().success();
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

bool API::updateTrafficLightsInSim()
{
  simulation_api_schema::UpdateTrafficLightsRequest req;
  simulation_api_schema::UpdateTrafficLightsResponse res;
  if (entity_manager_ptr_->trafficLightsChanged()) {
    for (const auto & [id, traffic_light] : entity_manager_ptr_->getConventionalTrafficLights()) {
      simulation_api_schema::TrafficSignal state;
      simulation_interface::toProto(
        static_cast<autoware_auto_perception_msgs::msg::TrafficSignal>(traffic_light), state);
      *req.add_states() = state;
    }
    zeromq_client_.call(req, res);
  }
  // TODO handle response
  return res.result().success();
}

std::optional<traffic_simulator_msgs::msg::EntityStatus> API::updateEntityStatusInSim(
  const std::string & entity_name, traffic_simulator_msgs::msg::EntityStatus status)
{
  simulation_api_schema::UpdateEntityStatusRequest req;
  simulation_api_schema::EntityStatus proto;
  status.name = entity_name;
  simulation_interface::toProto(status, *req.mutable_status());
  req.set_npc_logic_started(entity_manager_ptr_->isNpcLogicStarted());
  simulation_api_schema::UpdateEntityStatusResponse res;
  zeromq_client_.call(req, res);

  if (res.result().success()) {
    simulation_interface::toMsg(res.status().pose(), status.pose);
    simulation_interface::toMsg(res.status().action_status(), status.action_status);
    // Temporarily deinitialize lanelet pose as it should be correctly filled from here
    status.lanelet_pose_valid = false;
    status.lanelet_pose = traffic_simulator_msgs::msg::LaneletPose();
    return status;
  }
  return std::nullopt;
}

bool API::updateEntityStatusInSim()
{
  bool success = true;
  for (const auto & name : entity_manager_ptr_->getEntityNames()) {
    if (!entity_manager_ptr_->isEgo(name)) {
      success &= static_cast<bool>(
        updateEntityStatusInSim(name, entity_manager_ptr_->getEntityStatus(name)));
    }
  }
  return success;
}

bool API::updateFrame()
{
  if (entity_manager_ptr_->isEgoSpawned()) {
    if (configuration.standalone_mode) {
      THROW_SEMANTIC_ERROR("Ego simulation is no longer supported in standalone mode");
    }
    if (not entity_manager_ptr_->isEgoSpawned()) {
      THROW_SIMULATION_ERROR(
        "This exception is basically not supposed to be sent. Contact the developer as there is "
        "some kind of bug.");
    }

    auto ego_name = entity_manager_ptr_->getEgoName();
    auto ego_status = entity_manager_ptr_->getEntityStatus(ego_name);
    auto ego_status_opt = updateEntityStatusInSim(entity_manager_ptr_->getEgoName(), ego_status);
    if (ego_status_opt) {
      ego_status = *ego_status_opt;
    }
    /// @note apply additional status data (from ll2) to ego_entity_simulation_ for this update
    entity_manager_ptr_->fillLaneletPose(ego_name, ego_status);
    entity_manager_ptr_->setEntityStatusExternally(ego_name, ego_status);
  }

  entity_manager_ptr_->update(clock_.getCurrentSimulationTime(), clock_.getStepTime());
  traffic_controller_ptr_->execute();

  if (not configuration.standalone_mode) {
    if (!updateEntityStatusInSim()) {
      return false;
    }
    if (!updateTrafficLightsInSim()) {
      return false;
    }
    simulation_api_schema::UpdateFrameRequest req;
    req.set_current_time(clock_.getCurrentSimulationTime());
    simulation_interface::toProto(
      clock_.getCurrentRosTimeAsMsg().clock, *req.mutable_current_ros_time());
    simulation_api_schema::UpdateFrameResponse res;
    zeromq_client_.call(req, res);
    if (!res.result().success()) {
      return false;
    }
    entity_manager_ptr_->broadcastEntityTransform();
    clock_.update();
    clock_pub_->publish(clock_.getCurrentRosTimeAsMsg());
    debug_marker_pub_->publish(entity_manager_ptr_->makeDebugMarker());
    return true;
  } else {
    entity_manager_ptr_->broadcastEntityTransform();
    clock_.update();
    clock_pub_->publish(clock_.getCurrentRosTimeAsMsg());
    debug_marker_pub_->publish(entity_manager_ptr_->makeDebugMarker());
    return true;
  }
}

void API::startNpcLogic()
{
  if (isNpcLogicStarted()) {
    THROW_SIMULATION_ERROR("NPC logics are already started.");
  }
  entity_manager_ptr_->startNpcLogic();
  clock_.onNpcLogicStart();
}

void API::requestLaneChange(const std::string & name, const std::int64_t & lanelet_id)
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
}  // namespace traffic_simulator
