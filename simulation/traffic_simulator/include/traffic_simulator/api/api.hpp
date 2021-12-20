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

#ifndef TRAFFIC_SIMULATOR__API__API_HPP_
#define TRAFFIC_SIMULATOR__API__API_HPP_

#include <simulation_api_schema.pb.h>

#include <autoware_auto_vehicle_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_command.hpp>
#include <boost/variant.hpp>
#include <cassert>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <simulation_interface/zmq_client.hpp>
#include <stdexcept>
#include <string>
#include <traffic_simulator/api/configuration.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/entity/entity_manager.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/metrics/metrics.hpp>
#include <traffic_simulator/metrics/metrics_manager.hpp>
#include <traffic_simulator/simulation_clock/simulation_clock.hpp>
#include <traffic_simulator/traffic/traffic_controller.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>
#include <traffic_simulator_msgs/msg/driver_model.hpp>
#include <utility>

namespace traffic_simulator
{
struct VehicleBehavior : public entity::VehicleEntity::BuiltinBehavior
{
  static auto autoware() noexcept -> const std::string &
  {
    static const std::string name = "Autoware";
    return name;
  }
};

struct PedestrianBehavior : public entity::PedestrianEntity::BuiltinBehavior
{
};

class API
{
  using EntityManager = traffic_simulator::entity::EntityManager;

public:
  template <class NodeT, class AllocatorT = std::allocator<void>>
  explicit API(NodeT && node, const Configuration & configuration = Configuration())
  : configuration(configuration),
    entity_manager_ptr_(std::make_shared<EntityManager>(node, configuration)),
    traffic_controller_ptr_(std::make_shared<traffic_simulator::traffic::TrafficController>(
      entity_manager_ptr_->getHdmapUtils(), [this]() { return API::getEntityNames(); },
      [this](const auto & name) { return API::getEntityPose(name); },
      [this](const auto & name) { return API::despawn(name); }, configuration.auto_sink)),
    metrics_manager_(configuration.metrics_log_path, configuration.verbose),
    clock_pub_(rclcpp::create_publisher<rosgraph_msgs::msg::Clock>(
      node, "/clock", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    debug_marker_pub_(rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      node, "debug_marker", rclcpp::QoS(100), rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    initialize_client_(
      simulation_interface::protocol, simulation_interface::HostName::LOCALHOST,
      simulation_interface::ports::initialize),
    update_frame_client_(
      simulation_interface::protocol, simulation_interface::HostName::LOCALHOST,
      simulation_interface::ports::update_frame),
    update_sensor_frame_client_(
      simulation_interface::protocol, simulation_interface::HostName::LOCALHOST,
      simulation_interface::ports::update_sensor_frame),
    spawn_vehicle_entity_client_(
      simulation_interface::protocol, simulation_interface::HostName::LOCALHOST,
      simulation_interface::ports::spawn_vehicle_entity),
    spawn_pedestrian_entity_client_(
      simulation_interface::protocol, simulation_interface::HostName::LOCALHOST,
      simulation_interface::ports::spawn_pedestrian_entity),
    spawn_misc_object_entity_client_(
      simulation_interface::protocol, simulation_interface::HostName::LOCALHOST,
      simulation_interface::ports::spawn_misc_object_entity),
    despawn_entity_client_(
      simulation_interface::protocol, simulation_interface::HostName::LOCALHOST,
      simulation_interface::ports::despawn_entity),
    update_entity_status_client_(
      simulation_interface::protocol, simulation_interface::HostName::LOCALHOST,
      simulation_interface::ports::update_entity_status),
    attach_lidar_sensor_client_(
      simulation_interface::protocol, simulation_interface::HostName::LOCALHOST,
      simulation_interface::ports::attach_lidar_sensor),
    attach_detection_sensor_client_(
      simulation_interface::protocol, simulation_interface::HostName::LOCALHOST,
      simulation_interface::ports::attach_detection_sensor),
    update_traffic_lights_client_(
      simulation_interface::protocol, simulation_interface::HostName::LOCALHOST,
      simulation_interface::ports::update_traffic_lights)
  {
    metrics_manager_.setEntityManager(entity_manager_ptr_);
    setVerbose(configuration.verbose);
  }

  template <typename T, typename... Ts>
  void addMetric(const std::string & name, Ts &&... xs)
  {
    metrics_manager_.addMetric<T>(name, std::forward<Ts>(xs)...);
  }

  metrics::MetricLifecycle getMetricLifecycle(const std::string & name);

  bool metricExists(const std::string & name);

  void setVerbose(const bool verbose);

  bool spawn(
    const std::string & name,                                //
    const traffic_simulator_msgs::msg::VehicleParameters &,  //
    const std::string & = VehicleBehavior::defaultBehavior());

  bool spawn(
    const std::string & name,                                   //
    const traffic_simulator_msgs::msg::PedestrianParameters &,  //
    const std::string & = PedestrianBehavior::defaultBehavior());

  bool spawn(const std::string & name, const traffic_simulator_msgs::msg::MiscObjectParameters &);

  bool despawn(const std::string & name);

  traffic_simulator_msgs::msg::EntityStatus getEntityStatus(const std::string & name);

  geometry_msgs::msg::Pose getEntityPose(const std::string & name);

  bool setEntityStatus(
    const std::string & name, const traffic_simulator_msgs::msg::EntityStatus & status);
  bool setEntityStatus(
    const std::string & name, const geometry_msgs::msg::Pose & map_pose,
    const traffic_simulator_msgs::msg::ActionStatus & action_status =
      traffic_simulator::helper::constructActionStatus());
  bool setEntityStatus(
    const std::string & name, const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose,
    const traffic_simulator_msgs::msg::ActionStatus & action_status =
      traffic_simulator::helper::constructActionStatus());
  bool setEntityStatus(
    const std::string & name, const std::string & reference_entity_name,
    const geometry_msgs::msg::Pose & relative_pose,
    const traffic_simulator_msgs::msg::ActionStatus & action_status =
      traffic_simulator::helper::constructActionStatus());
  bool setEntityStatus(
    const std::string & name, const std::string & reference_entity_name,
    const geometry_msgs::msg::Point & relative_position,
    const geometry_msgs::msg::Vector3 & relative_rpy,
    const traffic_simulator_msgs::msg::ActionStatus & action_status =
      traffic_simulator::helper::constructActionStatus());

  boost::optional<double> getTimeHeadway(const std::string & from, const std::string & to);

  bool reachPosition(
    const std::string & name, const geometry_msgs::msg::Pose & target_pose, const double tolerance);
  bool reachPosition(
    const std::string & name, const traffic_simulator_msgs::msg::LaneletPose & target_pose,
    const double tolerance);
  bool reachPosition(
    const std::string & name, const std::string & target_name, const double tolerance) const;

  bool attachLidarSensor(const simulation_api_schema::LidarConfiguration &);
  bool attachLidarSensor(
    const std::string &, const helper::LidarType = traffic_simulator::helper::LidarType::VLP16);

  bool attachDetectionSensor(const simulation_api_schema::DetectionSensorConfiguration &);
  bool attachDetectionSensor(const std::string &);

  bool initialize(double realtime_factor, double step_time);

  bool updateFrame();

  double getCurrentTime() const noexcept { return clock_.getCurrentSimulationTime(); }

  void requestLaneChange(const std::string & name, const std::int64_t & lanelet_id);

  void requestLaneChange(
    const std::string & name, const traffic_simulator::entity::Direction & direction);

#define FORWARD_TO_ENTITY_MANAGER(NAME)                                    \
  template <typename... Ts>                                                \
  decltype(auto) NAME(Ts &&... xs)                                         \
  {                                                                        \
    assert(entity_manager_ptr_);                                           \
    return (*entity_manager_ptr_).NAME(std::forward<decltype(xs)>(xs)...); \
  }                                                                        \
  static_assert(true, "")

  FORWARD_TO_ENTITY_MANAGER(cancelRequest);
  FORWARD_TO_ENTITY_MANAGER(checkCollision);
  FORWARD_TO_ENTITY_MANAGER(engage);
  FORWARD_TO_ENTITY_MANAGER(entityExists);
  FORWARD_TO_ENTITY_MANAGER(getBoundingBoxDistance);
  FORWARD_TO_ENTITY_MANAGER(getCurrentAction);
  FORWARD_TO_ENTITY_MANAGER(getDriverModel);
  FORWARD_TO_ENTITY_MANAGER(getEgoName);
  FORWARD_TO_ENTITY_MANAGER(getEntityNames);
  FORWARD_TO_ENTITY_MANAGER(getLinearJerk);
  FORWARD_TO_ENTITY_MANAGER(getLongitudinalDistance);
  FORWARD_TO_ENTITY_MANAGER(getRelativePose);
  FORWARD_TO_ENTITY_MANAGER(getStandStillDuration);
  FORWARD_TO_ENTITY_MANAGER(getTrafficLightArrow);
  FORWARD_TO_ENTITY_MANAGER(getTrafficLightColor);
  FORWARD_TO_ENTITY_MANAGER(getVehicleCommand);
  FORWARD_TO_ENTITY_MANAGER(isInLanelet);
  FORWARD_TO_ENTITY_MANAGER(ready);
  FORWARD_TO_ENTITY_MANAGER(requestAcquirePosition);
  FORWARD_TO_ENTITY_MANAGER(requestAssignRoute);
  FORWARD_TO_ENTITY_MANAGER(requestWalkStraight);
  FORWARD_TO_ENTITY_MANAGER(setAccelerationLimit);
  FORWARD_TO_ENTITY_MANAGER(setDecelerationLimit);
  FORWARD_TO_ENTITY_MANAGER(setDriverModel);
  FORWARD_TO_ENTITY_MANAGER(setTargetSpeed);
  FORWARD_TO_ENTITY_MANAGER(setTrafficLightArrow);
  FORWARD_TO_ENTITY_MANAGER(setTrafficLightArrowPhase);
  FORWARD_TO_ENTITY_MANAGER(setTrafficLightColor);
  FORWARD_TO_ENTITY_MANAGER(setTrafficLightColorPhase);
  FORWARD_TO_ENTITY_MANAGER(setUpperBoundSpeed);
  FORWARD_TO_ENTITY_MANAGER(toLaneletPose);
  FORWARD_TO_ENTITY_MANAGER(toMapPose);

#undef FORWARD_TO_ENTITY_MANAGER

private:
  bool updateSensorFrame();
  bool updateEntityStatusInSim();
  bool updateTrafficLightsInSim();

  template <typename Parameters>
  bool spawn(
    const bool is_ego, const Parameters & parameters,
    const traffic_simulator_msgs::msg::EntityStatus & status,
    const std::string & behavior = PedestrianBehavior::defaultBehavior())
  {
    return spawn(is_ego, parameters.toXml(), status, behavior);
  }

  const Configuration configuration;

  const std::shared_ptr<traffic_simulator::entity::EntityManager> entity_manager_ptr_;

  const std::shared_ptr<traffic_simulator::traffic::TrafficController> traffic_controller_ptr_;

  metrics::MetricsManager metrics_manager_;

  const rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;

  traffic_simulator::SimulationClock clock_;

  zeromq::Client<
    simulation_api_schema::InitializeRequest, simulation_api_schema::InitializeResponse>
    initialize_client_;
  zeromq::Client<
    simulation_api_schema::UpdateFrameRequest, simulation_api_schema::UpdateFrameResponse>
    update_frame_client_;
  zeromq::Client<
    simulation_api_schema::UpdateSensorFrameRequest,
    simulation_api_schema::UpdateSensorFrameResponse>
    update_sensor_frame_client_;
  zeromq::Client<
    simulation_api_schema::SpawnVehicleEntityRequest,
    simulation_api_schema::SpawnVehicleEntityResponse>
    spawn_vehicle_entity_client_;
  zeromq::Client<
    simulation_api_schema::SpawnPedestrianEntityRequest,
    simulation_api_schema::SpawnPedestrianEntityResponse>
    spawn_pedestrian_entity_client_;
  zeromq::Client<
    simulation_api_schema::SpawnMiscObjectEntityRequest,
    simulation_api_schema::SpawnMiscObjectEntityResponse>
    spawn_misc_object_entity_client_;
  zeromq::Client<
    simulation_api_schema::DespawnEntityRequest, simulation_api_schema::DespawnEntityResponse>
    despawn_entity_client_;
  zeromq::Client<
    simulation_api_schema::UpdateEntityStatusRequest,
    simulation_api_schema::UpdateEntityStatusResponse>
    update_entity_status_client_;
  zeromq::Client<
    simulation_api_schema::AttachLidarSensorRequest,
    simulation_api_schema::AttachLidarSensorResponse>
    attach_lidar_sensor_client_;
  zeromq::Client<
    simulation_api_schema::AttachDetectionSensorRequest,
    simulation_api_schema::AttachDetectionSensorResponse>
    attach_detection_sensor_client_;
  zeromq::Client<
    simulation_api_schema::UpdateTrafficLightsRequest,
    simulation_api_schema::UpdateTrafficLightsResponse>
    update_traffic_lights_client_;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__API__API_HPP_
