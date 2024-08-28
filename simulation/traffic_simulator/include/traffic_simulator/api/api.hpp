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

#ifndef TRAFFIC_SIMULATOR__API__API_HPP_
#define TRAFFIC_SIMULATOR__API__API_HPP_

#include <simulation_api_schema.pb.h>

#include <autoware_auto_vehicle_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_state_command.hpp>
#include <boost/variant.hpp>
#include <cassert>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <simulation_interface/conversions.hpp>
#include <simulation_interface/zmq_multi_client.hpp>
#include <std_msgs/msg/float64.hpp>
#include <stdexcept>
#include <string>
#include <traffic_simulator/api/configuration.hpp>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/data_type/lane_change.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/entity/entity_manager.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/simulation_clock/simulation_clock.hpp>
#include <traffic_simulator/traffic/traffic_controller.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
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
public:
  template <typename NodeT, typename AllocatorT = std::allocator<void>, typename... Ts>
  explicit API(NodeT && node, const Configuration & configuration, Ts &&... xs)
  : configuration(configuration),
    node_parameters_(
      rclcpp::node_interfaces::get_node_parameters_interface(std::forward<NodeT>(node))),
    entity_manager_ptr_(
      std::make_shared<entity::EntityManager>(node, configuration, node_parameters_)),
    traffic_controller_ptr_(std::make_shared<traffic::TrafficController>(
      entity_manager_ptr_->getHdmapUtils(), [this]() { return API::getEntityNames(); },
      [this](const auto & entity_name) {
        if (const auto entity = getEntity(entity_name)) {
          return entity->getMapPose();
        } else {
          THROW_SEMANTIC_ERROR("Entity ", std::quoted(entity_name), " does not exists.");
        }
      },
      [this](const auto & name) { return API::despawn(name); }, configuration.auto_sink)),
    clock_pub_(rclcpp::create_publisher<rosgraph_msgs::msg::Clock>(
      node, "/clock", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    debug_marker_pub_(rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      node, "debug_marker", rclcpp::QoS(100), rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    real_time_factor_subscriber(rclcpp::create_subscription<std_msgs::msg::Float64>(
      node, "/real_time_factor", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const std_msgs::msg::Float64 & message) {
        /**
         * @note Pausing the simulation by setting the realtime_factor_ value to 0 is not supported and causes the simulation crash.
         * For that reason, before performing the action, it needs to be ensured that the incoming request data is a positive number.
         */
        if (message.data >= 0.001) {
          clock_.realtime_factor = message.data;
          simulation_api_schema::UpdateStepTimeRequest request;
          request.set_simulation_step_time(clock_.getStepTime());
          zeromq_client_.call(request);
        }
      })),
    clock_(node->get_parameter("use_sim_time").as_bool(), std::forward<decltype(xs)>(xs)...),
    zeromq_client_(
      simulation_interface::protocol, configuration.simulator_host, getZMQSocketPort(*node))
  {
    setVerbose(configuration.verbose);

    if (not configuration.standalone_mode) {
      simulation_api_schema::InitializeRequest request;
      request.set_initialize_time(clock_.getCurrentSimulationTime());
      request.set_lanelet2_map_path(configuration.lanelet2_map_path().string());
      request.set_realtime_factor(clock_.realtime_factor);
      request.set_step_time(clock_.getStepTime());
      simulation_interface::toProto(
        clock_.getCurrentRosTime(), *request.mutable_initialize_ros_time());
      if (not zeromq_client_.call(request).result().success()) {
        throw common::SimulationError("Failed to initialize simulator by InitializeRequest");
      }
    }
  }

  template <typename ParameterT, typename... Ts>
  auto getROS2Parameter(Ts &&... xs) const -> decltype(auto)
  {
    return getParameter<ParameterT>(node_parameters_, std::forward<Ts>(xs)...);
  }

  template <typename Node>
  int getZMQSocketPort(Node & node)
  {
    if (!node.has_parameter("port")) node.declare_parameter("port", 5555);
    return node.get_parameter("port").as_int();
  }

  void closeZMQConnection() { zeromq_client_.closeConnection(); }

  void setVerbose(const bool verbose);

  template <typename Pose>
  auto spawn(
    const std::string & name, const Pose & pose,
    const traffic_simulator_msgs::msg::VehicleParameters & parameters,
    const std::string & behavior = VehicleBehavior::defaultBehavior(),
    const std::string & model3d = "")
  {
    auto register_to_entity_manager = [&]() {
      if (behavior == VehicleBehavior::autoware()) {
        return entity_manager_ptr_->entityExists(name) or
               entity_manager_ptr_->spawnEntity<entity::EgoEntity>(
                 name, pose, parameters, getCurrentTime(), configuration, node_parameters_);
      } else {
        return entity_manager_ptr_->spawnEntity<entity::VehicleEntity>(
          name, pose, parameters, getCurrentTime(), behavior);
      }
    };

    auto register_to_environment_simulator = [&]() {
      if (configuration.standalone_mode) {
        return true;
      } else if (const auto entity = entity_manager_ptr_->getEntity(name); not entity) {
        throw common::SemanticError(
          "Entity ", name, " can not be registered in simulator - it has not been spawned yet.");
      } else {
        simulation_api_schema::SpawnVehicleEntityRequest req;
        simulation_interface::toProto(parameters, *req.mutable_parameters());
        req.mutable_parameters()->set_name(name);
        req.set_asset_key(model3d);
        simulation_interface::toProto(entity->getMapPose(), *req.mutable_pose());
        req.set_is_ego(behavior == VehicleBehavior::autoware());
        /// @todo Should be filled from function API
        req.set_initial_speed(0.0);
        return zeromq_client_.call(req).result().success();
      }
    };

    return register_to_entity_manager() and register_to_environment_simulator();
  }

  template <typename Pose>
  auto spawn(
    const std::string & name, const Pose & pose,
    const traffic_simulator_msgs::msg::PedestrianParameters & parameters,
    const std::string & behavior = PedestrianBehavior::defaultBehavior(),
    const std::string & model3d = "")
  {
    auto register_to_entity_manager = [&]() {
      return entity_manager_ptr_->spawnEntity<entity::PedestrianEntity>(
        name, pose, parameters, getCurrentTime(), behavior);
    };

    auto register_to_environment_simulator = [&]() {
      if (configuration.standalone_mode) {
        return true;
      } else if (const auto entity = entity_manager_ptr_->getEntity(name); not entity) {
        throw common::SemanticError(
          "Entity ", name, " can not be registered in simulator - it has not been spawned yet.");
      } else {
        simulation_api_schema::SpawnPedestrianEntityRequest req;
        simulation_interface::toProto(parameters, *req.mutable_parameters());
        req.mutable_parameters()->set_name(name);
        req.set_asset_key(model3d);
        simulation_interface::toProto(entity->getMapPose(), *req.mutable_pose());
        return zeromq_client_.call(req).result().success();
      }
    };

    return register_to_entity_manager() and register_to_environment_simulator();
  }

  template <typename Pose>
  auto spawn(
    const std::string & name, const Pose & pose,
    const traffic_simulator_msgs::msg::MiscObjectParameters & parameters,
    const std::string & model3d = "")
  {
    auto register_to_entity_manager = [&]() {
      return entity_manager_ptr_->spawnEntity<entity::MiscObjectEntity>(
        name, pose, parameters, getCurrentTime());
    };

    auto register_to_environment_simulator = [&]() {
      if (configuration.standalone_mode) {
        return true;
      } else if (const auto entity = entity_manager_ptr_->getEntity(name); not entity) {
        throw common::SemanticError(
          "Entity ", name, " can not be registered in simulator - it has not been spawned yet.");
      } else {
        simulation_api_schema::SpawnMiscObjectEntityRequest req;
        simulation_interface::toProto(parameters, *req.mutable_parameters());
        req.mutable_parameters()->set_name(name);
        req.set_asset_key(model3d);
        simulation_interface::toProto(entity->getMapPose(), *req.mutable_pose());
        return zeromq_client_.call(req).result().success();
      }
    };

    return register_to_entity_manager() and register_to_environment_simulator();
  }

  bool despawn(const std::string & name);
  bool despawnEntities();

  auto setEntityStatus(const std::string & name, const EntityStatus & status) -> void;
  auto respawn(
    const std::string & name, const geometry_msgs::msg::PoseWithCovarianceStamped & new_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose) -> void;
  auto setEntityStatus(
    const std::string & name, const geometry_msgs::msg::Pose & map_pose,
    const traffic_simulator_msgs::msg::ActionStatus & action_status =
      helper::constructActionStatus()) -> void;
  auto setEntityStatus(
    const std::string & name, const LaneletPose & lanelet_pose,
    const traffic_simulator_msgs::msg::ActionStatus & action_status) -> void;
  auto setEntityStatus(
    const std::string & name, const CanonicalizedLaneletPose & canonicalized_lanelet_pose,
    const traffic_simulator_msgs::msg::ActionStatus & action_status =
      helper::constructActionStatus()) -> void;
  auto setEntityStatus(
    const std::string & name, const std::string & reference_entity_name,
    const geometry_msgs::msg::Pose & relative_pose,
    const traffic_simulator_msgs::msg::ActionStatus & action_status =
      helper::constructActionStatus()) -> void;
  auto setEntityStatus(
    const std::string & name, const std::string & reference_entity_name,
    const geometry_msgs::msg::Point & relative_position,
    const geometry_msgs::msg::Vector3 & relative_rpy,
    const traffic_simulator_msgs::msg::ActionStatus & action_status =
      helper::constructActionStatus()) -> void;

  std::optional<double> getTimeHeadway(const std::string & from, const std::string & to);

  auto attachImuSensor(
    const std::string &, const simulation_api_schema::ImuSensorConfiguration & configuration)
    -> bool;

  bool attachPseudoTrafficLightDetector(
    const simulation_api_schema::PseudoTrafficLightDetectorConfiguration &);

  bool attachLidarSensor(const simulation_api_schema::LidarConfiguration &);
  bool attachLidarSensor(
    const std::string &, const double lidar_sensor_delay,
    const helper::LidarType = helper::LidarType::VLP16);

  bool attachDetectionSensor(const simulation_api_schema::DetectionSensorConfiguration &);
  bool attachDetectionSensor(
    const std::string &, double detection_sensor_range, bool detect_all_objects_in_range,
    double pos_noise_stddev, int random_seed, double probability_of_lost,
    double object_recognition_delay);

  bool attachOccupancyGridSensor(const simulation_api_schema::OccupancyGridSensorConfiguration &);

  bool updateFrame();

  double getCurrentTime() const noexcept { return clock_.getCurrentScenarioTime(); }

  void startNpcLogic();

  void requestLaneChange(const std::string & name, const lanelet::Id & lanelet_id);

  void requestLaneChange(const std::string & name, const lane_change::Direction & direction);

  void requestLaneChange(const std::string & name, const lane_change::Parameter &);

  void requestLaneChange(
    const std::string & name, const lane_change::RelativeTarget & target,
    const lane_change::TrajectoryShape trajectory_shape,
    const lane_change::Constraint & constraint);

  void requestLaneChange(
    const std::string & name, const lane_change::AbsoluteTarget & target,
    const lane_change::TrajectoryShape trajectory_shape,
    const lane_change::Constraint & constraint);

  /**
   * @brief Add a traffic source to the simulation
   * @param radius The radius defining the area on which entities will be spawned
   * @param rate The rate at which entities will be spawned [Hz]
   * @param speed The speed of the spawned entities
   * @param position The center of the area on which entities will be spawned (includes orientation)
   * @param distribution The parameters of the spawned entities with their respective weights for random distribution
   *                     For each entity there are 4 parameters in a tuple:
   *                     - VehicleParameters or PedestrianParameters - parameters of entity
   *                     - std::string - name of behavior to be used when spawning
   *                     - std::string - name of 3D model to be used when spawning
   *                     - double - weight of entity for random distribution
   * @param allow_spawn_outside_lane Whether entities can be spawned outside the lane
   * @param require_footprint_fitting Whether entities are required to fit inside lanelet polygon when spawned
   *                                  (allow_spawn_outside_lane has higher priority)
   * @param random_orientation Whether entities should have their orientation randomized before lane matching
   * @param random_seed [Optional] The seed for the random number generator
   */
  auto addTrafficSource(
    const double radius, const double rate, const double speed,
    const geometry_msgs::msg::Pose & position,
    const traffic::TrafficSource::Distribution & distribution,
    const bool allow_spawn_outside_lane = false, const bool require_footprint_fitting = false,
    const bool random_orientation = false, std::optional<int> random_seed = std::nullopt) -> void;

  auto getEntity(const std::string & name) const -> std::shared_ptr<entity::EntityBase>;

  // clang-format off
#define FORWARD_TO_ENTITY_MANAGER(NAME)                                    \
  /*!                                                                      \
   @brief Forward to arguments to the EntityManager::NAME function.        \
   @return return value of the EntityManager::NAME function.               \
   @note This function was defined by FORWARD_TO_ENTITY_MANAGER macro.     \
   */                                                                      \
  template <typename... Ts>                                                \
  decltype(auto) NAME(Ts &&... xs)                                         \
  {                                                                        \
    assert(entity_manager_ptr_);                                           \
    return (*entity_manager_ptr_).NAME(std::forward<decltype(xs)>(xs)...); \
  }                                                                        \
  static_assert(true, "")
  // clang-format on

  FORWARD_TO_ENTITY_MANAGER(activateOutOfRangeJob);
  FORWARD_TO_ENTITY_MANAGER(asFieldOperatorApplication);
  FORWARD_TO_ENTITY_MANAGER(cancelRequest);
  FORWARD_TO_ENTITY_MANAGER(checkCollision);
  FORWARD_TO_ENTITY_MANAGER(entityExists);
  FORWARD_TO_ENTITY_MANAGER(getBehaviorParameter);
  FORWARD_TO_ENTITY_MANAGER(getBoundingBox);
  FORWARD_TO_ENTITY_MANAGER(getConventionalTrafficLight);
  FORWARD_TO_ENTITY_MANAGER(getConventionalTrafficLights);
  FORWARD_TO_ENTITY_MANAGER(getCurrentAccel);
  FORWARD_TO_ENTITY_MANAGER(getCurrentAction);
  FORWARD_TO_ENTITY_MANAGER(getCurrentTwist);
  FORWARD_TO_ENTITY_MANAGER(getEgoName);
  FORWARD_TO_ENTITY_MANAGER(getEntityNames);
  FORWARD_TO_ENTITY_MANAGER(getEntityStatus);
  FORWARD_TO_ENTITY_MANAGER(getCanonicalizedStatusBeforeUpdate);
  FORWARD_TO_ENTITY_MANAGER(getHdmapUtils);
  FORWARD_TO_ENTITY_MANAGER(getLinearJerk);
  FORWARD_TO_ENTITY_MANAGER(getStandStillDuration);
  FORWARD_TO_ENTITY_MANAGER(getTraveledDistance);
  FORWARD_TO_ENTITY_MANAGER(getV2ITrafficLight);
  FORWARD_TO_ENTITY_MANAGER(getV2ITrafficLights);
  FORWARD_TO_ENTITY_MANAGER(reachPosition);
  FORWARD_TO_ENTITY_MANAGER(isEgoSpawned);
  FORWARD_TO_ENTITY_MANAGER(isInLanelet);
  FORWARD_TO_ENTITY_MANAGER(isNpcLogicStarted);
  FORWARD_TO_ENTITY_MANAGER(laneMatchingSucceed);
  FORWARD_TO_ENTITY_MANAGER(requestAcquirePosition);
  FORWARD_TO_ENTITY_MANAGER(requestAssignRoute);
  FORWARD_TO_ENTITY_MANAGER(requestFollowTrajectory);
  FORWARD_TO_ENTITY_MANAGER(requestSpeedChange);
  FORWARD_TO_ENTITY_MANAGER(requestSynchronize);
  FORWARD_TO_ENTITY_MANAGER(requestWalkStraight);
  FORWARD_TO_ENTITY_MANAGER(requestClearRoute);
  FORWARD_TO_ENTITY_MANAGER(resetBehaviorPlugin);
  FORWARD_TO_ENTITY_MANAGER(resetConventionalTrafficLightPublishRate);
  FORWARD_TO_ENTITY_MANAGER(resetV2ITrafficLightPublishRate);
  FORWARD_TO_ENTITY_MANAGER(setAcceleration);
  FORWARD_TO_ENTITY_MANAGER(setAccelerationLimit);
  FORWARD_TO_ENTITY_MANAGER(setAccelerationRateLimit);
  FORWARD_TO_ENTITY_MANAGER(setBehaviorParameter);
  FORWARD_TO_ENTITY_MANAGER(setConventionalTrafficLightConfidence);
  FORWARD_TO_ENTITY_MANAGER(setDecelerationLimit);
  FORWARD_TO_ENTITY_MANAGER(setDecelerationRateLimit);
  FORWARD_TO_ENTITY_MANAGER(setLinearVelocity);
  FORWARD_TO_ENTITY_MANAGER(setMapPose);
  FORWARD_TO_ENTITY_MANAGER(setTwist);
  FORWARD_TO_ENTITY_MANAGER(setVelocityLimit);

private:
  FORWARD_TO_ENTITY_MANAGER(getDefaultMatchingDistanceForLaneletPoseCalculation);

public:
#undef FORWARD_TO_ENTITY_MANAGER

private:
  bool updateTimeInSim();

  bool updateEntitiesStatusInSim();

  bool updateTrafficLightsInSim();

  const Configuration configuration;

  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;

  const std::shared_ptr<entity::EntityManager> entity_manager_ptr_;

  const std::shared_ptr<traffic::TrafficController> traffic_controller_ptr_;

  const rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;

  const rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr real_time_factor_subscriber;

  SimulationClock clock_;

  zeromq::MultiClient zeromq_client_;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__API__API_HPP_
