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
#include <traffic_simulator/traffic/traffic_source.hpp>
#include <traffic_simulator/traffic_lights/traffic_light.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights.hpp>
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
  : configuration_(configuration),
    node_parameters_(
      rclcpp::node_interfaces::get_node_parameters_interface(std::forward<NodeT>(node))),
    clock_pub_(rclcpp::create_publisher<rosgraph_msgs::msg::Clock>(
      node, "/clock", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    debug_marker_pub_(rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      node, "debug_marker", rclcpp::QoS(100), rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    clock_(getROS2Parameter<bool>("use_sim_time", true), std::forward<decltype(xs)>(xs)...),
    zeromq_client_(
      simulation_interface::protocol, configuration.simulator_host,
      getROS2Parameter<int>("port", 5555)),
    entity_manager_ptr_(
      std::make_shared<entity::EntityManager>(node, configuration, node_parameters_)),
    traffic_controller_ptr_(std::make_shared<traffic::TrafficController>(
      [this](const std::string & name) { despawn(name); }, entity_manager_ptr_,
      configuration.auto_sink_entity_types)),
    traffic_lights_ptr_(std::make_shared<TrafficLights>(
      node, entity_manager_ptr_->getHdmapUtils(),
      getROS2Parameter<std::string>("architecture_type", "awf/universe/20240605"))),
    real_time_factor_subscriber_(rclcpp::create_subscription<std_msgs::msg::Float64>(
      node, "/real_time_factor", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](const std_msgs::msg::Float64 & message) {
        return setSimulationStepTime(message.data);
      }))
  {
    entity_manager_ptr_->setVerbose(configuration_.verbose);
    entity_manager_ptr_->setTrafficLights(traffic_lights_ptr_);
    if (not init()) {
        throw common::SimulationError("Failed to initialize simulator by InitializeRequest");
    }
  }

  // global
  template <typename ParameterT, typename... Ts>
  auto getROS2Parameter(Ts &&... xs) const -> decltype(auto)
  {
    return getParameter<ParameterT>(node_parameters_, std::forward<Ts>(xs)...);
  }

  auto init() -> bool;

  auto setVerbose(const bool verbose) -> void;

  auto setSimulationStepTime(const double step_time) -> bool;

  auto startNpcLogic() -> void;

  auto isNpcLogicStarted() const -> bool;

  auto getCurrentTime() const noexcept -> double;

  auto closeZMQConnection() -> void;

  // update
  auto updateFrame() -> bool;

  // entities, ego - spawn
  template <
    typename PoseType, typename ParamsType,
    typename = std::enable_if_t<std::disjunction_v<
      std::is_same<std::decay_t<ParamsType>, traffic_simulator_msgs::msg::VehicleParameters>,
      std::is_same<std::decay_t<ParamsType>, traffic_simulator_msgs::msg::PedestrianParameters>,
      std::is_same<std::decay_t<ParamsType>, traffic_simulator_msgs::msg::MiscObjectParameters>>>>
  auto spawn(
    const std::string & name, const PoseType & pose, const ParamsType & parameters,
    const std::string & behavior = "", const std::string & model3d = "") -> entity::EntityBase &
  {
    using VehicleParameters = traffic_simulator_msgs::msg::VehicleParameters;
    using PedestrianParameters = traffic_simulator_msgs::msg::PedestrianParameters;
    using MiscObjectParameters = traffic_simulator_msgs::msg::MiscObjectParameters;

    auto register_to_entity_manager = [&]() -> entity::EntityBase & {
      if constexpr (std::is_same_v<ParamsType, VehicleParameters>) {
      if (behavior == VehicleBehavior::autoware()) {
          return entity_manager_ptr_->spawnEntity<entity::EgoEntity>(
                 name, pose, parameters, getCurrentTime(), configuration_, node_parameters_);
      } else {
        return entity_manager_ptr_->spawnEntity<entity::VehicleEntity>(
            name, pose, parameters, getCurrentTime(),
            behavior.empty() ? VehicleBehavior::defaultBehavior() : behavior);
        }
      } else if constexpr (std::is_same_v<ParamsType, PedestrianParameters>) {
        return entity_manager_ptr_->spawnEntity<entity::PedestrianEntity>(
          name, pose, parameters, getCurrentTime(),
          behavior.empty() ? PedestrianBehavior::defaultBehavior() : behavior);
      } else if constexpr (std::is_same_v<ParamsType, MiscObjectParameters>) {
        return entity_manager_ptr_->spawnEntity<entity::MiscObjectEntity>(
          name, pose, parameters, getCurrentTime());
      }
    };

    auto prepare_and_send_request = [&](const auto & entity, auto & request) -> bool {
      simulation_interface::toProto(parameters, *request.mutable_parameters());
      request.mutable_parameters()->set_name(name);
      request.set_asset_key(model3d);
      simulation_interface::toProto(entity.getMapPose(), *request.mutable_pose());
      return zeromq_client_.call(request).result().success();
    };

    auto register_to_environment_simulator = [&](const auto & entity) -> bool {
      if (configuration_.standalone_mode) {
        return true;
      } else {
        if constexpr (std::is_same_v<ParamsType, VehicleParameters>) {
          simulation_api_schema::SpawnVehicleEntityRequest request;
          request.set_is_ego(behavior == VehicleBehavior::autoware());
          /// @todo Should be filled from function API
          request.set_initial_speed(0.0);
          return prepare_and_send_request(entity, request);
        } else if constexpr (std::is_same_v<ParamsType, PedestrianParameters>) {
          simulation_api_schema::SpawnPedestrianEntityRequest request;
          return prepare_and_send_request(entity, request);
        } else if constexpr (std::is_same_v<ParamsType, MiscObjectParameters>) {
          simulation_api_schema::SpawnMiscObjectEntityRequest request;
          return prepare_and_send_request(entity, request);
        } else {
          return false;
        }
      }
    };

    auto & entity = register_to_entity_manager();
    if (register_to_environment_simulator(entity)) {
      return entity;
      } else {
      THROW_SEMANTIC_ERROR("Spawn entity ", std::quoted(name), " resulted in failure.");
    }
  }

  // sensors - attach
  auto attachImuSensor(
    const std::string &, const simulation_api_schema::ImuSensorConfiguration & configuration)
    -> bool;

  auto attachPseudoTrafficLightDetector(
    const simulation_api_schema::PseudoTrafficLightDetectorConfiguration &) -> bool;

  auto attachLidarSensor(const simulation_api_schema::LidarConfiguration &) -> bool;

  auto attachLidarSensor(
    const std::string &, const double lidar_sensor_delay,
    const helper::LidarType = helper::LidarType::VLP16) -> bool;

  auto attachDetectionSensor(const simulation_api_schema::DetectionSensorConfiguration &) -> bool;

  auto attachDetectionSensor(
    const std::string &, double detection_sensor_range, bool detect_all_objects_in_range,
    double pos_noise_stddev, int random_seed, double probability_of_lost,
    double object_recognition_delay) -> bool;

  auto attachOccupancyGridSensor(const simulation_api_schema::OccupancyGridSensorConfiguration &)
    -> bool;

  // ego - checks, getters
  auto getFirstEgoName() const -> std::optional<std::string>;

  auto getEgoEntity(const std::string & name) -> entity::EgoEntity &;

  auto getEgoEntity(const std::string & name) const -> const entity::EgoEntity &;

  // entities - checks, getters
  auto isEntityExist(const std::string & name) const -> bool;

  auto getEntityNames() const -> std::vector<std::string>;

  auto getEntity(const std::string & name) -> entity::EntityBase &;

  auto getEntity(const std::string & name) const -> const entity::EntityBase &;

  auto getEntityPointer(const std::string & name) const -> std::shared_ptr<entity::EntityBase>;

  // entities - respawn, despawn, reset
  auto resetBehaviorPlugin(const std::string & name, const std::string & behavior_plugin_name)
    -> void;

  auto respawn(
    const std::string & name, const geometry_msgs::msg::PoseWithCovarianceStamped & new_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose) -> void;

  auto despawn(const std::string & name) -> bool;

  auto despawnEntities() -> bool;

  // entities - features
  auto checkCollision(
    const std::string & first_entity_name, const std::string & second_entity_name) const -> bool;

  // traffics, lanelet
  auto getHdmapUtils() const -> const std::shared_ptr<hdmap_utils::HdMapUtils> &;

  auto getV2ITrafficLights() const -> std::shared_ptr<V2ITrafficLights>;

  auto getConventionalTrafficLights() const -> std::shared_ptr<ConventionalTrafficLights>;
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

private:
  auto updateTimeInSim() -> bool;

  auto updateEntitiesStatusInSim() -> bool;

  auto updateTrafficLightsInSim() -> bool;

  const Configuration configuration_;

  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;

  const rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;

  SimulationClock clock_;

  zeromq::MultiClient zeromq_client_;

  const std::shared_ptr<entity::EntityManager> entity_manager_ptr_;

  const std::shared_ptr<traffic::TrafficController> traffic_controller_ptr_;

  const std::shared_ptr<TrafficLights> traffic_lights_ptr_;

  const rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr real_time_factor_subscriber_;
};
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__API__API_HPP_
