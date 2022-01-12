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

#include <tf2/LinearMath/Quaternion.h>

#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <simulation_interface/conversions.hpp>
#include <stdexcept>
#include <string>
#include <traffic_simulator/api/api.hpp>

namespace traffic_simulator
{
metrics::MetricLifecycle API::getMetricLifecycle(const std::string & name)
{
  return metrics_manager_.getLifecycle(name);
}

bool API::metricExists(const std::string & name) { return metrics_manager_.exists(name); }

void API::setVerbose(const bool verbose)
{
  metrics_manager_.setVerbose(verbose);
  entity_manager_ptr_->setVerbose(verbose);
}

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
    despawn_entity_client_.call(req, res);
    return res.result().success();
  }
  return true;
}

bool API::spawn(
  const std::string & name,                                           //
  const traffic_simulator_msgs::msg::VehicleParameters & parameters,  //
  const std::string & behavior)
{
  auto register_to_entity_manager = [&]() {
    if (behavior == VehicleBehavior::autoware()) {
      using traffic_simulator::entity::EgoEntity;
      return entity_manager_ptr_->entityExists(name) or
             entity_manager_ptr_->spawnEntity<EgoEntity>(
               name, configuration, clock_.getStepTime(), parameters);
    } else {
      using traffic_simulator::entity::VehicleEntity;
      return entity_manager_ptr_->spawnEntity<VehicleEntity>(name, parameters, behavior);
    }
  };

  auto register_to_environment_simulator = [&]() {
    if (configuration.standalone_mode) {
      return true;
    } else {
      simulation_api_schema::SpawnVehicleEntityRequest req;
      simulation_api_schema::SpawnVehicleEntityResponse res;
      simulation_interface::toProto(parameters, *req.mutable_parameters());
      req.mutable_parameters()->set_name(name);
      req.set_is_ego(behavior == VehicleBehavior::autoware());
      spawn_vehicle_entity_client_.call(req, res);
      return res.result().success();
    }
  };

  return register_to_entity_manager() and register_to_environment_simulator();
}

bool API::spawn(
  const std::string & name,                                              //
  const traffic_simulator_msgs::msg::PedestrianParameters & parameters,  //
  const std::string & behavior)
{
  auto register_to_entity_manager = [&]() {
    using traffic_simulator::entity::PedestrianEntity;
    return entity_manager_ptr_->spawnEntity<PedestrianEntity>(name, parameters, behavior);
  };

  auto register_to_environment_simulator = [&]() {
    if (configuration.standalone_mode) {
      return true;
    } else {
      simulation_api_schema::SpawnPedestrianEntityRequest req;
      simulation_api_schema::SpawnPedestrianEntityResponse res;
      simulation_interface::toProto(parameters, *req.mutable_parameters());
      req.mutable_parameters()->set_name(name);
      spawn_pedestrian_entity_client_.call(req, res);
      return res.result().success();
    }
  };

  return register_to_entity_manager() and register_to_environment_simulator();
}

bool API::spawn(
  const std::string & name,  //
  const traffic_simulator_msgs::msg::MiscObjectParameters & parameters)
{
  auto register_to_entity_manager = [&]() {
    using traffic_simulator::entity::MiscObjectEntity;
    return entity_manager_ptr_->spawnEntity<MiscObjectEntity>(name, parameters);
  };

  auto register_to_environment_simulator = [&]() {
    if (configuration.standalone_mode) {
      return true;
    } else {
      simulation_api_schema::SpawnMiscObjectEntityRequest req;
      simulation_api_schema::SpawnMiscObjectEntityResponse res;
      simulation_interface::toProto(parameters, *req.mutable_parameters());
      req.mutable_parameters()->set_name(name);
      spawn_misc_object_entity_client_.call(req, res);
      return res.result().success();
    }
  };

  return register_to_entity_manager() and register_to_environment_simulator();
}

geometry_msgs::msg::Pose API::getEntityPose(const std::string & name)
{
  auto status = getEntityStatus(name);
  return status.pose;
}

traffic_simulator_msgs::msg::EntityStatus API::getEntityStatus(const std::string & name)
{
  auto status = entity_manager_ptr_->getEntityStatus(name);
  if (!status) {
    THROW_SEMANTIC_ERROR("entity : ", name, " status is empty");
  }
  return status.get();
}

bool API::setEntityStatus(
  const std::string & name, const traffic_simulator_msgs::msg::EntityStatus & status)
{
  return entity_manager_ptr_->setEntityStatus(name, status);
}

bool API::setEntityStatus(
  const std::string & name, const std::string & reference_entity_name,
  const geometry_msgs::msg::Point & relative_position,
  const geometry_msgs::msg::Vector3 & relative_rpy,
  const traffic_simulator_msgs::msg::ActionStatus & action_status)
{
  geometry_msgs::msg::Pose relative_pose;
  relative_pose.position = relative_position;
  relative_pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(relative_rpy);
  return setEntityStatus(name, reference_entity_name, relative_pose, action_status);
}

bool API::setEntityStatus(
  const std::string & name, const std::string & reference_entity_name,
  const geometry_msgs::msg::Pose & relative_pose,
  const traffic_simulator_msgs::msg::ActionStatus & action_status)
{
  const auto pose = entity_manager_ptr_->getMapPose(reference_entity_name, relative_pose);
  traffic_simulator_msgs::msg::EntityStatus status;
  status.time = clock_.getCurrentSimulationTime();
  status.pose = pose;
  const auto lanelet_pose = entity_manager_ptr_->toLaneletPose(
    pose, entity_manager_ptr_->getBoundingBox(reference_entity_name), false);
  status.action_status = action_status;
  if (lanelet_pose) {
    status.lanelet_pose_valid = true;
    status.lanelet_pose = lanelet_pose.get();
  } else {
    status.lanelet_pose_valid = false;
  }
  return entity_manager_ptr_->setEntityStatus(name, status);
}

boost::optional<double> API::getTimeHeadway(const std::string & from, const std::string & to)
{
  if (!entity_manager_ptr_->entityStatusSet(from) || !entity_manager_ptr_->entityStatusSet(to)) {
    return boost::none;
  }
  geometry_msgs::msg::Pose pose = getRelativePose(from, to);
  if (pose.position.x > 0) {
    return boost::none;
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
  return entity_manager_ptr_->entityStatusSet(name) &&
         entity_manager_ptr_->reachPosition(name, target_pose, tolerance);
}

bool API::reachPosition(
  const std::string & name, const traffic_simulator_msgs::msg::LaneletPose & target_pose,
  const double tolerance)
{
  return entity_manager_ptr_->entityStatusSet(name) &&
         entity_manager_ptr_->reachPosition(
           name, target_pose.lanelet_id, target_pose.s, target_pose.offset, tolerance);
}

bool API::reachPosition(
  const std::string & name, const std::string & target_name, const double tolerance) const
{
  return entity_manager_ptr_->entityStatusSet(name) &&
         entity_manager_ptr_->entityStatusSet(target_name) &&
         entity_manager_ptr_->reachPosition(name, target_name, tolerance);
}

bool API::setEntityStatus(
  const std::string & name, const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose,
  const traffic_simulator_msgs::msg::ActionStatus & action_status)
{
  traffic_simulator_msgs::msg::EntityStatus status;
  status.lanelet_pose = lanelet_pose;
  status.lanelet_pose_valid = true;
  status.bounding_box = entity_manager_ptr_->getBoundingBox(name);
  status.pose = entity_manager_ptr_->toMapPose(lanelet_pose);
  status.name = name;
  status.time = getCurrentTime();
  status.action_status = action_status;
  return setEntityStatus(name, status);
}

bool API::setEntityStatus(
  const std::string & name, const geometry_msgs::msg::Pose & map_pose,
  const traffic_simulator_msgs::msg::ActionStatus & action_status)
{
  const auto lanelet_pose =
    entity_manager_ptr_->toLaneletPose(map_pose, entity_manager_ptr_->getBoundingBox(name), false);
  traffic_simulator_msgs::msg::EntityStatus status;
  if (lanelet_pose) {
    status.lanelet_pose = lanelet_pose.get();
  } else {
    status.lanelet_pose_valid = false;
  }
  status.pose = map_pose;
  status.name = name;
  status.action_status = action_status;
  status.time = getCurrentTime();
  status.bounding_box = entity_manager_ptr_->getBoundingBox(name);
  return setEntityStatus(name, status);
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
    simulation_api_schema::InitializeResponse res;
    initialize_client_.call(req, res);
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
    attach_detection_sensor_client_.call(req, res);
    return res.result().success();
  }
}

bool API::attachDetectionSensor(const std::string & entity_name)
{
  return attachDetectionSensor(helper::constructDetectionSensorConfiguration(
    entity_name, getParameter<std::string>("architecture_type", "tier4/proposal"), 0.1));
}

bool API::attachLidarSensor(const simulation_api_schema::LidarConfiguration & lidar_configuration)
{
  if (configuration.standalone_mode) {
    return true;
  } else {
    simulation_api_schema::AttachLidarSensorRequest req;
    simulation_api_schema::AttachLidarSensorResponse res;
    *req.mutable_configuration() = lidar_configuration;
    attach_lidar_sensor_client_.call(req, res);
    return res.result().success();
  }
}

bool API::attachLidarSensor(const std::string & entity_name, const helper::LidarType lidar_type)
{
  return attachLidarSensor(helper::constructLidarConfiguration(
    lidar_type, entity_name, getParameter<std::string>("architecture_type", "tier4/proposal")));
}

bool API::updateSensorFrame()
{
  if (configuration.standalone_mode) {
    return true;
  } else {
    simulation_api_schema::UpdateSensorFrameRequest req;
    req.set_current_time(clock_.getCurrentSimulationTime());
    simulation_interface::toProto(
      clock_.getCurrentRosTimeAsMsg().clock, *req.mutable_current_ros_time());
    simulation_api_schema::UpdateSensorFrameResponse res;
    update_sensor_frame_client_.call(req, res);
    return res.result().success();
  }
}

bool API::updateTrafficLightsInSim()
{
  simulation_api_schema::UpdateTrafficLightsRequest req;
  simulation_api_schema::UpdateTrafficLightsResponse res;
  if (entity_manager_ptr_->trafficLightsChanged()) {
    auto ids = entity_manager_ptr_->getTrafficLightIds();
    for (auto id : ids) {
      simulation_api_schema::TrafficLightState state;
      auto traffic_light = entity_manager_ptr_->getTrafficLightInstance(id);
      simulation_interface::toProto(
#ifndef SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO
        static_cast<autoware_auto_perception_msgs::msg::TrafficSignal>(traffic_light),
#else
        static_cast<autoware_perception_msgs::msg::TrafficLightState>(traffic_light),
#endif
        state);
      *req.add_states() = state;
    }
    update_traffic_lights_client_.call(req, res);
  }
  // TODO handle response
  return res.result().success();
}

bool API::updateEntityStatusInSim()
{
  simulation_api_schema::UpdateEntityStatusRequest req;
  if (entity_manager_ptr_->getNumberOfEgo() != 0) {
    simulation_interface::toProto(
      entity_manager_ptr_->getVehicleCommand(entity_manager_ptr_->getEgoName()),
      *req.mutable_vehicle_command());
    const auto ego_status_before_update =
      entity_manager_ptr_->getEntityStatusBeforeUpdate(entity_manager_ptr_->getEgoName());
    if (ego_status_before_update) {
      req.set_ego_entity_status_before_update_is_empty(false);
      simulation_interface::toProto(
        ego_status_before_update.get(), *req.mutable_ego_entity_status_before_update());
    } else {
      req.set_ego_entity_status_before_update_is_empty(true);
    }
  }
  const auto names = entity_manager_ptr_->getEntityNames();
  for (const auto name : names) {
    auto status = entity_manager_ptr_->getEntityStatus(name);
    if (status) {
      traffic_simulator_msgs::EntityStatus proto;
      status.get().name = name;
      simulation_interface::toProto(status.get(), proto);
      *req.add_status() = proto;
    }
  }
  simulation_api_schema::UpdateEntityStatusResponse res;
  update_entity_status_client_.call(req, res);
  for (const auto status : res.status()) {
    auto entity_status = entity_manager_ptr_->getEntityStatus(status.name());
    if (!entity_status) {
      continue;
    }
    traffic_simulator_msgs::msg::EntityStatus status_msg;
    status_msg = entity_status.get();
    geometry_msgs::msg::Pose pose;
    simulation_interface::toMsg(status.pose(), pose);
    status_msg.pose = pose;
    const auto lanelet_pose = entity_manager_ptr_->toLaneletPose(
      pose, entity_manager_ptr_->getBoundingBox(status.name()), false);
    if (lanelet_pose) {
      status_msg.lanelet_pose_valid = true;
      status_msg.lanelet_pose = lanelet_pose.get();
    } else {
      status_msg.lanelet_pose_valid = false;
      status_msg.lanelet_pose = traffic_simulator_msgs::msg::LaneletPose();
    }
    simulation_interface::toMsg(status.action_status().twist(), status_msg.action_status.twist);
    simulation_interface::toMsg(status.action_status().accel(), status_msg.action_status.accel);
    entity_manager_ptr_->setEntityStatus(status.name(), status_msg);
  }
  return res.result().success();
}

bool API::updateFrame()
{
  boost::optional<traffic_simulator_msgs::msg::EntityStatus> ego_status_before_update = boost::none;
  entity_manager_ptr_->update(clock_.getCurrentSimulationTime(), clock_.getStepTime());
  traffic_controller_ptr_->execute();

  if (not configuration.standalone_mode) {
    simulation_api_schema::UpdateFrameRequest req;
    req.set_current_time(clock_.getCurrentSimulationTime());
    simulation_interface::toProto(
      clock_.getCurrentRosTimeAsMsg().clock, *req.mutable_current_ros_time());
    simulation_api_schema::UpdateFrameResponse res;
    update_frame_client_.call(req, res);
    if (!res.result().success()) {
      return false;
    }
    entity_manager_ptr_->broadcastEntityTransform();
    clock_.update();
    clock_pub_->publish(clock_.getCurrentRosTimeAsMsg());
    debug_marker_pub_->publish(entity_manager_ptr_->makeDebugMarker());
    metrics_manager_.calculate();
    if (!updateEntityStatusInSim()) {
      return false;
    }
    updateTrafficLightsInSim();
    return updateSensorFrame();
  } else {
    entity_manager_ptr_->broadcastEntityTransform();
    clock_.update();
    clock_pub_->publish(clock_.getCurrentRosTimeAsMsg());
    debug_marker_pub_->publish(entity_manager_ptr_->makeDebugMarker());
    metrics_manager_.calculate();
    return true;
  }
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
