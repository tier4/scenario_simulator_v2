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

#ifndef SIMULATION_API__API__API_HPP_
#define SIMULATION_API__API__API_HPP_

#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <awapi_accessor/accessor.hpp>
#include <boost/filesystem.hpp>
#include <openscenario_msgs/msg/driver_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <simulation_api/entity/entity_manager.hpp>
#include <simulation_api/helper/helper.hpp>
#include <simulation_api/metrics/metrics_manager.hpp>
#include <simulation_api/traffic_lights/traffic_light.hpp>
#include <simulation_api_schema.pb.h>
#include <simulation_interface/zmq_client.hpp>

#include <cassert>
#include <memory>
#include <string>
#include <utility>

namespace scenario_simulator
{
class ExecutionFailedError : public std::runtime_error
{
public:
  template<typename ... Ts>
  explicit ExecutionFailedError(Ts && ... xs)
  : runtime_error(std::forward<decltype(xs)>(xs)...)
  {}

  virtual ~ExecutionFailedError() = default;
};

class API
{
  using EntityManager = simulation_api::entity::EntityManager;

public:
  const std::string lanelet2_map_osm;

  template<
    class NodeT,
    class AllocatorT = std::allocator<void>>
  explicit API(
    NodeT && node,
    const boost::filesystem::path,
    const std::string & lanelet2_map_osm,
    const bool verbose = false,
    const bool standalone_mode = false,
    const std::string & metrics_logfile_path = "/tmp/metrics.json")
  : lanelet2_map_osm(lanelet2_map_osm),
    standalone_mode(standalone_mode),
    entity_manager_ptr_(std::make_shared<EntityManager>(node, lanelet2_map_osm)),
    metrics_manager_(verbose, metrics_logfile_path),
    initialize_client_(
      simulation_interface::protocol,
      simulation_interface::HostName::LOCLHOST,
      simulation_interface::ports::initialize),
    update_frame_client_(
      simulation_interface::protocol,
      simulation_interface::HostName::LOCLHOST,
      simulation_interface::ports::update_frame),
    update_sensor_frame_client_(
      simulation_interface::protocol,
      simulation_interface::HostName::LOCLHOST,
      simulation_interface::ports::update_sensor_frame),
    spawn_vehicle_entity_client_(
      simulation_interface::protocol,
      simulation_interface::HostName::LOCLHOST,
      simulation_interface::ports::spawn_vehicle_entity),
    spawn_pedestrian_entity_client_(
      simulation_interface::protocol,
      simulation_interface::HostName::LOCLHOST,
      simulation_interface::ports::spawn_pedestrian_entity),
    despawn_entity_client_(
      simulation_interface::protocol,
      simulation_interface::HostName::LOCLHOST,
      simulation_interface::ports::despawn_entity),
    update_entity_status_client_(
      simulation_interface::protocol,
      simulation_interface::HostName::LOCLHOST,
      simulation_interface::ports::update_entity_status),
    attach_lidar_sensor_client_(
      simulation_interface::protocol,
      simulation_interface::HostName::LOCLHOST,
      simulation_interface::ports::attach_lidar_sensor),
    attach_detection_sensor_client_(
      simulation_interface::protocol,
      simulation_interface::HostName::LOCLHOST,
      simulation_interface::ports::attach_detection_sensor)
  {
    static const std::string address = "127.0.0.1";

    metrics_manager_.setEntityManager(entity_manager_ptr_);

    setVerbose(verbose);
  }

  template<typename T, typename ... Ts>
  void addMetric(const std::string & name, Ts && ... xs)
  {
    metrics_manager_.addMetric<T>(name, std::forward<Ts>(xs)...);
  }

  void setVerbose(const bool verbose);

  [[deprecated("catalog_xml will be removed in the near future")]]
  bool spawn(
    const bool is_ego,
    const std::string & name,
    const std::string & catalog_xml);

  bool spawn(
    const bool is_ego,
    const std::string & name,
    const openscenario_msgs::msg::VehicleParameters & params);

  bool spawn(
    const bool is_ego,
    const std::string & name,
    const openscenario_msgs::msg::PedestrianParameters & params);

  template<typename ... Ts>
  decltype(auto) spawn(
    const bool is_ego,
    const std::string & name,
    const std::string & catalog_xml,
    Ts && ... xs)
  {
    return
      spawn(is_ego, name, catalog_xml) &&
      setEntityStatus(name, std::forward<decltype(xs)>(xs)...);
  }

  template<
    typename Parameters,  // Maybe, VehicleParameters or PedestrianParameters
    typename ... Ts  // Arguments for setEntityStatus
  >
  decltype(auto) spawn(
    const bool is_ego,
    const std::string & name,
    const Parameters & params, Ts && ... xs)
  {
    return
      spawn(is_ego, name, params) &&
      setEntityStatus(name, std::forward<decltype(xs)>(xs)...);
  }

  bool despawn(const std::string & name);

  openscenario_msgs::msg::EntityStatus getEntityStatus(
    const std::string & name);

  bool setEntityStatus(
    const std::string & name,
    const openscenario_msgs::msg::EntityStatus & status);
  bool setEntityStatus(
    const std::string & name,
    const geometry_msgs::msg::Pose & map_pose,
    const openscenario_msgs::msg::ActionStatus & action_status =
    simulation_api::helper::constructActionStatus());
  bool setEntityStatus(
    const std::string & name,
    const openscenario_msgs::msg::LaneletPose & lanelet_pose,
    const openscenario_msgs::msg::ActionStatus & action_status =
    simulation_api::helper::constructActionStatus());
  bool setEntityStatus(
    const std::string & name,
    const std::string & reference_entity_name,
    const geometry_msgs::msg::Pose & relative_pose,
    const openscenario_msgs::msg::ActionStatus & action_status =
    simulation_api::helper::constructActionStatus());
  bool setEntityStatus(
    const std::string & name,
    const std::string & reference_entity_name,
    const geometry_msgs::msg::Point & relative_position,
    const geometry_msgs::msg::Vector3 & relative_rpy,
    const openscenario_msgs::msg::ActionStatus & action_status =
    simulation_api::helper::constructActionStatus());

  boost::optional<double> getTimeHeadway(
    const std::string & from,
    const std::string & to);

  bool reachPosition(
    const std::string & name,
    const geometry_msgs::msg::Pose & target_pose,
    const double tolerance);
  bool reachPosition(
    const std::string & name,
    const openscenario_msgs::msg::LaneletPose & target_pose,
    const double tolerance);
  bool reachPosition(
    const std::string & name,
    const std::string & target_name,
    const double tolerance) const;

  bool attachLidarSensor(
    simulation_api_schema::LidarConfiguration configuration);
  bool attachDetectionSensor(
    simulation_api_schema::DetectionSensorConfiguration configuration);

  bool initialize(double realtime_factor, double step_time);

  bool updateFrame();
  bool updateSensorFrame();

  double getCurrentTime() const noexcept
  {
    return current_time_;
  }

  const bool standalone_mode;

  #define FORWARD_TO_ENTITY_MANAGER(NAME) \
  template<typename ... Ts> \
  decltype(auto) NAME(Ts && ... xs) \
  { \
    assert(entity_manager_ptr_); \
    return (*entity_manager_ptr_).NAME(std::forward<decltype(xs)>(xs)...); \
  } static_assert(true, "")

  FORWARD_TO_ENTITY_MANAGER(checkCollision);
  FORWARD_TO_ENTITY_MANAGER(entityExists);
  FORWARD_TO_ENTITY_MANAGER(getBoundingBoxDistance);
  FORWARD_TO_ENTITY_MANAGER(getLinearJerk);
  FORWARD_TO_ENTITY_MANAGER(getLongitudinalDistance);
  FORWARD_TO_ENTITY_MANAGER(getRelativePose);
  FORWARD_TO_ENTITY_MANAGER(getStandStillDuration);
  FORWARD_TO_ENTITY_MANAGER(getTrafficLightArrow);
  FORWARD_TO_ENTITY_MANAGER(getTrafficLightColor);
  FORWARD_TO_ENTITY_MANAGER(isInLanelet);
  FORWARD_TO_ENTITY_MANAGER(requestAcquirePosition);
  FORWARD_TO_ENTITY_MANAGER(requestAssignRoute);
  FORWARD_TO_ENTITY_MANAGER(requestLaneChange);
  FORWARD_TO_ENTITY_MANAGER(setDriverModel);
  FORWARD_TO_ENTITY_MANAGER(setTargetSpeed);
  FORWARD_TO_ENTITY_MANAGER(setTrafficLightArrow);
  FORWARD_TO_ENTITY_MANAGER(setTrafficLightArrowPhase);
  FORWARD_TO_ENTITY_MANAGER(setTrafficLightColor);
  FORWARD_TO_ENTITY_MANAGER(setTrafficLightColorPhase);
  FORWARD_TO_ENTITY_MANAGER(toLaneletPose);
  FORWARD_TO_ENTITY_MANAGER(toMapPose);

  #undef FORWARD_TO_ENTITY_MANAGER

private:
  bool updateEntityStatusInSim();
  bool spawn(
    const bool is_ego,
    const std::string & catalog_xml,
    const openscenario_msgs::msg::EntityStatus & status);

  template<
    typename Parameters  // Maybe, VehicleParameters or PedestrianParameters
  >
  bool spawn(
    const bool is_ego,
    const Parameters & parameters,
    const openscenario_msgs::msg::EntityStatus & status)
  {
    return spawn(is_ego, parameters.toXml(), status);
  }

  // openscenario_msgs::msg::EntityStatus toStatus(XmlRpc::XmlRpcValue param);
  // XmlRpc::XmlRpcValue toValue(openscenario_msgs::msg::EntityStatus status);

  // std::shared_ptr<XmlRpc::XmlRpcClient> client_ptr_;
  std::shared_ptr<simulation_api::entity::EntityManager> entity_manager_ptr_;
  double step_time_;
  double current_time_;

  metrics::MetricsManager metrics_manager_;

  zeromq::Client<simulation_api_schema::InitializeRequest,
    simulation_api_schema::InitializeResponse> initialize_client_;
  zeromq::Client<simulation_api_schema::UpdateFrameRequest,
    simulation_api_schema::UpdateFrameResponse> update_frame_client_;
  zeromq::Client<simulation_api_schema::UpdateSensorFrameRequest,
    simulation_api_schema::UpdateSensorFrameResponse> update_sensor_frame_client_;
  zeromq::Client<simulation_api_schema::SpawnVehicleEntityRequest,
    simulation_api_schema::SpawnVehicleEntityResponse> spawn_vehicle_entity_client_;
  zeromq::Client<simulation_api_schema::SpawnPedestrianEntityRequest,
    simulation_api_schema::SpawnPedestrianEntityResponse> spawn_pedestrian_entity_client_;
  zeromq::Client<simulation_api_schema::DespawnEntityRequest,
    simulation_api_schema::DespawnEntityResponse> despawn_entity_client_;
  zeromq::Client<simulation_api_schema::UpdateEntityStatusRequest,
    simulation_api_schema::UpdateEntityStatusResponse> update_entity_status_client_;
  zeromq::Client<simulation_api_schema::AttachLidarSensorRequest,
    simulation_api_schema::AttachLidarSensorResponse> attach_lidar_sensor_client_;
  zeromq::Client<simulation_api_schema::AttachDetectionSensorRequest,
    simulation_api_schema::AttachDetectionSensorResponse> attach_detection_sensor_client_;
};
}  // namespace scenario_simulator

#endif  // SIMULATION_API__API__API_HPP_
