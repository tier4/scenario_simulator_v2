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

#ifndef TRAFFIC_SIMULATOR__ENTITY__ENTITY_MANAGER_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__ENTITY_MANAGER_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <optional>
#include <rclcpp/node_interfaces/get_node_topics_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <stdexcept>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <traffic_simulator/api/configuration.hpp>
#include <traffic_simulator/data_type/lane_change.hpp>
#include <traffic_simulator/data_type/speed_change.hpp>
#include <traffic_simulator/entity/ego_entity.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/entity/misc_object_entity.hpp>
#include <traffic_simulator/entity/pedestrian_entity.hpp>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic_lights/configurable_rate_updater.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_marker_publisher.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_publisher.hpp>
#include <traffic_simulator/traffic_lights/traffic_lights.hpp>
#include <traffic_simulator/utils/node_parameters.hpp>
#include <traffic_simulator/utils/pose.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/entity_status_with_trajectory_array.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace traffic_simulator
{
namespace entity
{
class LaneletMarkerQoS : public rclcpp::QoS
{
public:
  explicit LaneletMarkerQoS(std::size_t depth = 1) : rclcpp::QoS(depth) { transient_local(); }
};

class EntityMarkerQoS : public rclcpp::QoS
{
public:
  explicit EntityMarkerQoS(std::size_t depth = 100) : rclcpp::QoS(depth) {}
};

class EntityManager
{
  using EntityStatusWithTrajectoryArray =
    traffic_simulator_msgs::msg::EntityStatusWithTrajectoryArray;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

public:
  template <class NodeT, class AllocatorT = std::allocator<void>>
  explicit EntityManager(
    NodeT && node, const Configuration & configuration,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_parameters)
  : configuration_(configuration),
    node_topics_interface(rclcpp::node_interfaces::get_node_topics_interface(node)),
    node_parameters_(node_parameters),
    broadcaster_(node),
    base_link_broadcaster_(node),
    clock_ptr_(node->get_clock()),
    npc_logic_started_(false),
    entity_status_array_pub_ptr_(rclcpp::create_publisher<EntityStatusWithTrajectoryArray>(
      node, "entity/status", EntityMarkerQoS(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    lanelet_marker_pub_ptr_(rclcpp::create_publisher<MarkerArray>(
      node, "lanelet/marker", LaneletMarkerQoS(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    hdmap_utils_ptr_(std::make_shared<hdmap_utils::HdMapUtils>(
      configuration.lanelet2_map_path(), getOrigin(node_parameters))),
    markers_raw_(hdmap_utils_ptr_->generateMarker())
  {
    updateHdmapMarker();
  }

  ~EntityManager() = default;

  // global
  /**
     This function is necessary because the TrafficLights object is created after the EntityManager,
     so it can be assigned during the call of the EntityManager constructor.
     TrafficLights cannot be created before the EntityManager due to the dependency on HdMapUtils.
   */
  auto setTrafficLights(const std::shared_ptr<TrafficLights> & traffic_lights_ptr) -> void;

  auto setVerbose(const bool verbose) -> void;

  auto startNpcLogic(const double current_time) -> void;

  auto isNpcLogicStarted() const -> bool;

  auto makeDebugMarker() const -> visualization_msgs::msg::MarkerArray;

  // update
  auto update(const double current_time, const double step_time) -> void;

  auto updateNpcLogic(const std::string & name, const double current_time, const double step_time)
    -> const CanonicalizedEntityStatus &;

  auto updateHdmapMarker() -> void;

  auto broadcastEntityTransform() -> void;

  auto broadcastTransform(
    const geometry_msgs::msg::PoseStamped & pose, const bool static_transform = true) -> void;

  // entities, ego - spawn
  template <typename EntityType, typename PoseType, typename ParametersType, typename... Ts>
  auto spawnEntity(
    const std::string & name, const PoseType & pose, const ParametersType & parameters,
    const double current_time, Ts &&... xs)
  {
    static_assert(
      std::disjunction<
        std::is_same<PoseType, CanonicalizedLaneletPose>,
        std::is_same<PoseType, geometry_msgs::msg::Pose>>::value,
      "Pose must be of type CanonicalizedLaneletPose or geometry_msgs::msg::Pose");

    auto makeEntityStatus = [&]() -> CanonicalizedEntityStatus {
      EntityStatus entity_status;

      if constexpr (std::is_same_v<std::decay_t<EntityType>, EgoEntity>) {
        if (isAnyEgoSpawned()) {
          THROW_SEMANTIC_ERROR("multi ego simulation does not support yet");
        } else {
          entity_status.type.type = traffic_simulator_msgs::msg::EntityType::EGO;
        }
      } else if constexpr (std::is_same_v<std::decay_t<EntityType>, VehicleEntity>) {
        entity_status.type.type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
      } else if constexpr (std::is_same_v<std::decay_t<EntityType>, PedestrianEntity>) {
        entity_status.type.type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
      } else {
        entity_status.type.type = traffic_simulator_msgs::msg::EntityType::MISC_OBJECT;
      }

      entity_status.subtype = parameters.subtype;
      entity_status.time = current_time;
      entity_status.name = name;
      entity_status.bounding_box = parameters.bounding_box;
      entity_status.action_status = traffic_simulator_msgs::msg::ActionStatus();
      entity_status.action_status.current_action = "waiting for initialize";

      const auto include_crosswalk = [](const auto & entity_type) {
        return (traffic_simulator_msgs::msg::EntityType::PEDESTRIAN == entity_type.type) ||
               (traffic_simulator_msgs::msg::EntityType::MISC_OBJECT == entity_type.type);
      }(entity_status.type);

      const auto matching_distance = [](const auto & local_parameters) {
        if constexpr (std::is_same_v<
                        std::decay_t<ParametersType>,
                        traffic_simulator_msgs::msg::VehicleParameters>) {
          return std::max(
                   local_parameters.axles.front_axle.track_width,
                   local_parameters.axles.rear_axle.track_width) *
                   0.5 +
                 1.0;
        } else {
          return local_parameters.bounding_box.dimensions.y * 0.5 + 1.0;
        }
      }(parameters);

      if constexpr (std::is_same_v<std::decay_t<PoseType>, LaneletPose>) {
        THROW_SYNTAX_ERROR(
          "LaneletPose is not supported type as pose argument. Only CanonicalizedLaneletPose and "
          "msg::Pose are supported as pose argument of EntityManager::spawnEntity().");
      } else if constexpr (std::is_same_v<std::decay_t<PoseType>, CanonicalizedLaneletPose>) {
        entity_status.pose = pose::toMapPose(pose);
        return CanonicalizedEntityStatus(entity_status, pose);
      } else if constexpr (std::is_same_v<std::decay_t<PoseType>, geometry_msgs::msg::Pose>) {
        entity_status.pose = pose;
        const auto canonicalized_lanelet_pose = pose::toCanonicalizedLaneletPose(
          pose, parameters.bounding_box, include_crosswalk, matching_distance);
        return CanonicalizedEntityStatus(entity_status, canonicalized_lanelet_pose);
      }
    };

    if (const auto [iter, success] = entities_.emplace(
          name, std::make_unique<EntityType>(
                  name, makeEntityStatus(), hdmap_utils_ptr_, parameters,
                  std::forward<decltype(xs)>(xs)...));
        success) {
      // FIXME: this ignores V2I traffic lights
      iter->second->setTrafficLights(traffic_lights_ptr_->getConventionalTrafficLights());
      return success;
    } else {
      THROW_SEMANTIC_ERROR("Entity ", std::quoted(name), " is already exists.");
    }
  }

  // ego - checks, getters
  auto getNumberOfEgo() const -> std::size_t;

  auto isAnyEgoSpawned() const -> bool;

  auto getFirstEgoName() const -> std::optional<std::string>;

  auto getEgoEntity(const std::string & name) -> entity::EgoEntity &;

  auto getEgoEntity(const std::string & name) const -> const entity::EgoEntity &;

  // entities - checks, getters
  auto isEntityExist(const std::string & name) const -> bool;

  auto getEntityNames() const -> const std::vector<std::string>;

  auto getEntity(const std::string & name) -> entity::EntityBase &;

  auto getEntity(const std::string & name) const -> const entity::EntityBase &;

  auto getEntityPointer(const std::string & name) const -> std::shared_ptr<entity::EntityBase>;

  // entities - respawn, despawn, reset
  /**
   * @brief Reset behavior plugin of the target entity.
   * The internal behavior is to take over the various parameters and save them, then respawn the Entity and set the parameters.
   * @param name The name of the target entity.
   * @param behavior_plugin_name The name of the behavior plugin you want to set.
   * @sa entity::PedestrianEntity::BuiltinBehavior
   * @sa entity::VehicleEntity::BuiltinBehavior
   */
  auto resetBehaviorPlugin(const std::string & name, const std::string & behavior_plugin_name)
    -> void;

  auto despawnEntity(const std::string & name) -> bool;

  // traffics, lanelet
  auto getHdmapUtils() -> const std::shared_ptr<hdmap_utils::HdMapUtils> &;

  auto getObstacle(const std::string & name)
    -> std::optional<traffic_simulator_msgs::msg::Obstacle>;

  auto getPedestrianParameters(const std::string & name) const
    -> const traffic_simulator_msgs::msg::PedestrianParameters &;

  auto getVehicleParameters(const std::string & name) const
    -> const traffic_simulator_msgs::msg::VehicleParameters &;

  auto getWaypoints(const std::string & name) -> traffic_simulator_msgs::msg::WaypointsArray;

  template <typename T>
  auto getGoalPoses(const std::string & name) -> std::vector<T>
  {
    if constexpr (std::is_same_v<std::decay_t<T>, CanonicalizedLaneletPose>) {
      if (not npc_logic_started_) {
        return {};
      } else {
        return entities_.at(name)->getGoalPoses();
      }
    } else {
      if (not npc_logic_started_) {
        return {};
      } else {
        std::vector<geometry_msgs::msg::Pose> poses;
        for (const auto & lanelet_pose : getGoalPoses<CanonicalizedLaneletPose>(name)) {
          poses.push_back(pose::toMapPose(lanelet_pose));
        }
        return poses;
      }
    }
  }

  template <typename MessageT, typename... Args>
  auto createPublisher(Args &&... args)
  {
    return rclcpp::create_publisher<MessageT>(node_topics_interface, std::forward<Args>(args)...);
  }

  template <typename MessageT, typename... Args>
  auto createSubscription(Args &&... args)
  {
    return rclcpp::create_subscription<MessageT>(
      node_topics_interface, std::forward<Args>(args)...);
  }

private:
  Configuration configuration_;

  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface;
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;

  tf2_ros::StaticTransformBroadcaster broadcaster_;
  tf2_ros::TransformBroadcaster base_link_broadcaster_;

  const rclcpp::Clock::SharedPtr clock_ptr_;

  std::unordered_map<std::string, std::shared_ptr<entity::EntityBase>> entities_;

  bool npc_logic_started_;

  const rclcpp::Publisher<EntityStatusWithTrajectoryArray>::SharedPtr entity_status_array_pub_ptr_;

  const rclcpp::Publisher<MarkerArray>::SharedPtr lanelet_marker_pub_ptr_;

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;

  MarkerArray markers_raw_;

  std::shared_ptr<TrafficLights> traffic_lights_ptr_;
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__ENTITY_MANAGER_HPP_
