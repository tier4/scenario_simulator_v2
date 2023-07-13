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

#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <memory>
#include <optional>
#include <rclcpp/node_interfaces/get_node_topics_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <stdexcept>
#include <string>
#include <traffic_simulator/api/configuration.hpp>
#include <traffic_simulator/data_type/lane_change.hpp>
#include <traffic_simulator/data_type/speed_change.hpp>
#include <traffic_simulator/entity/ego_entity.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/entity/misc_object_entity.hpp>
#include <traffic_simulator/entity/pedestrian_entity.hpp>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic/traffic_sink.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_marker_publisher.hpp>
#include <traffic_simulator/traffic_lights/v2i_traffic_light_publisher.hpp>
#include <traffic_simulator_msgs/msg/behavior_parameter.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/entity_status_with_trajectory_array.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

/// @todo find some shared space for this function
template <typename T>
static auto getParameter(const std::string & name, T value = {})
{
  rclcpp::Node node{"get_parameter", "simulation"};

  node.declare_parameter<T>(name, value);
  node.get_parameter<T>(name, value);

  return value;
}

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
  Configuration configuration;

  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface;

  tf2_ros::StaticTransformBroadcaster broadcaster_;
  tf2_ros::TransformBroadcaster base_link_broadcaster_;

  const rclcpp::Clock::SharedPtr clock_ptr_;

  std::unordered_map<std::string, std::unique_ptr<traffic_simulator::entity::EntityBase>> entities_;

  double step_time_;

  double current_time_;

  bool npc_logic_started_;

  using EntityStatusWithTrajectoryArray =
    traffic_simulator_msgs::msg::EntityStatusWithTrajectoryArray;
  const rclcpp::Publisher<EntityStatusWithTrajectoryArray>::SharedPtr entity_status_array_pub_ptr_;

  using MarkerArray = visualization_msgs::msg::MarkerArray;
  const rclcpp::Publisher<MarkerArray>::SharedPtr lanelet_marker_pub_ptr_;

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;

  MarkerArray markers_raw_;

  const std::shared_ptr<TrafficLightManager> conventional_traffic_light_manager_ptr_;
  const std::shared_ptr<TrafficLightMarkerPublisher>
    conventional_traffic_light_marker_publisher_ptr_;

  const std::shared_ptr<TrafficLightManager> v2i_traffic_light_manager_ptr_;
  const std::shared_ptr<TrafficLightMarkerPublisher> v2i_traffic_light_marker_publisher_ptr_;
  const std::shared_ptr<
    V2ITrafficLightPublisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>>
    v2i_traffic_light_publisher_ptr_;

  using LaneletPose = traffic_simulator_msgs::msg::LaneletPose;

public:
  template <typename Node>
  auto getOrigin(Node & node) const
  {
    geographic_msgs::msg::GeoPoint origin;
    {
      if (!node.has_parameter("origin_latitude")) {
        node.declare_parameter("origin_latitude", 0.0);
      }
      if (!node.has_parameter("origin_longitude")) {
        node.declare_parameter("origin_longitude", 0.0);
      }
      node.get_parameter("origin_latitude", origin.latitude);
      node.get_parameter("origin_longitude", origin.longitude);
    }

    return origin;
  }

  template <typename... Ts>
  auto makeConventionalTrafficLightManager(Ts &&... xs) -> std::shared_ptr<TrafficLightManager>
  {
    if (const auto architecture_type =
          getParameter<std::string>("architecture_type", "awf/universe");
        architecture_type == "awf/universe") {
      return std::make_shared<TrafficLightManager>(std::forward<decltype(xs)>(xs)...);
    } else {
      throw common::SemanticError(
        "Unexpected architecture_type ", std::quoted(architecture_type),
        " given for conventional traffic lights simulation.");
    }
  }

  template <typename... Ts>
  auto makeV2ITrafficLightManager(Ts &&... xs) -> std::shared_ptr<TrafficLightManager>
  {
    if (const auto architecture_type =
          getParameter<std::string>("architecture_type", "awf/universe");
        architecture_type == "awf/universe") {
      return std::make_shared<TrafficLightManager>(std::forward<decltype(xs)>(xs)...);
    } else {
      throw common::SemanticError(
        "Unexpected architecture_type ", std::quoted(architecture_type),
        " given for V2I traffic lights simulation.");
    }
  }

  template <class NodeT, class AllocatorT = std::allocator<void>>
  explicit EntityManager(NodeT && node, const Configuration & configuration)
  : configuration(configuration),
    node_topics_interface(rclcpp::node_interfaces::get_node_topics_interface(node)),
    broadcaster_(node),
    base_link_broadcaster_(node),
    clock_ptr_(node->get_clock()),
    current_time_(std::numeric_limits<double>::quiet_NaN()),
    npc_logic_started_(false),
    entity_status_array_pub_ptr_(rclcpp::create_publisher<EntityStatusWithTrajectoryArray>(
      node, "entity/status", EntityMarkerQoS(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    lanelet_marker_pub_ptr_(rclcpp::create_publisher<MarkerArray>(
      node, "lanelet/marker", LaneletMarkerQoS(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    hdmap_utils_ptr_(std::make_shared<hdmap_utils::HdMapUtils>(
      configuration.lanelet2_map_path(), getOrigin(*node))),
    markers_raw_(hdmap_utils_ptr_->generateMarker()),
    conventional_traffic_light_manager_ptr_(makeConventionalTrafficLightManager(hdmap_utils_ptr_)),
    conventional_traffic_light_marker_publisher_ptr_(
      std::make_shared<TrafficLightMarkerPublisher>(conventional_traffic_light_manager_ptr_, node)),
    v2i_traffic_light_manager_ptr_(makeV2ITrafficLightManager(hdmap_utils_ptr_)),
    v2i_traffic_light_marker_publisher_ptr_(
      std::make_shared<TrafficLightMarkerPublisher>(v2i_traffic_light_manager_ptr_, node)),
    v2i_traffic_light_publisher_ptr_(
      std::make_shared<
        V2ITrafficLightPublisher<autoware_auto_perception_msgs::msg::TrafficSignalArray>>(
        v2i_traffic_light_manager_ptr_, "/v2x/traffic_signals", node))
  {
    updateHdmapMarker();
  }

  ~EntityManager() = default;

public:
#define FORWARD_GETTER_TO_TRAFFIC_LIGHT_MANAGER(NAME)                 \
  template <typename... Ts>                                           \
  decltype(auto) getConventional##NAME(Ts &&... xs) const             \
  {                                                                   \
    return conventional_traffic_light_manager_ptr_->get##NAME(xs...); \
  }                                                                   \
  static_assert(true, "");                                            \
  template <typename... Ts>                                           \
  decltype(auto) getV2I##NAME(Ts &&... xs) const                      \
  {                                                                   \
    return v2i_traffic_light_manager_ptr_->get##NAME(xs...);          \
  }                                                                   \
  static_assert(true, "")

  FORWARD_GETTER_TO_TRAFFIC_LIGHT_MANAGER(TrafficLights);
  FORWARD_GETTER_TO_TRAFFIC_LIGHT_MANAGER(TrafficLight);

#undef FORWARD_GETTER_TO_TRAFFIC_LIGHT_MANAGER

  auto resetConventionalTrafficLightPublishRate(double rate) -> void
  {
    conventional_traffic_light_marker_publisher_ptr_->resetPublishRate(rate);
  }

  auto resetV2ITrafficLightPublishRate(double rate) -> void
  {
    v2i_traffic_light_marker_publisher_ptr_->resetPublishRate(rate);
    v2i_traffic_light_publisher_ptr_->resetPublishRate(rate);
  }

#define FORWARD_TO_HDMAP_UTILS(NAME)                                  \
  template <typename... Ts>                                           \
  decltype(auto) NAME(Ts &&... xs) const                              \
  {                                                                   \
    return hdmap_utils_ptr_->NAME(std::forward<decltype(xs)>(xs)...); \
  }                                                                   \
  static_assert(true, "")

  FORWARD_TO_HDMAP_UTILS(toLaneletPose);
  // FORWARD_TO_HDMAP_UTILS(toMapPose);

#undef FORWARD_TO_HDMAP_UTILS

#define FORWARD_TO_ENTITY(IDENTIFIER, ...)                                     \
  template <typename... Ts>                                                    \
  decltype(auto) IDENTIFIER(const std::string & name, Ts &&... xs) __VA_ARGS__ \
  try {                                                                        \
    return entities_.at(name)->IDENTIFIER(std::forward<decltype(xs)>(xs)...);  \
  } catch (const std::out_of_range &) {                                        \
    THROW_SEMANTIC_ERROR("entity : ", name, "does not exist");                 \
  }                                                                            \
  static_assert(true, "")

  FORWARD_TO_ENTITY(asFieldOperatorApplication, const);
  FORWARD_TO_ENTITY(cancelRequest, );
  FORWARD_TO_ENTITY(get2DPolygon, const);
  FORWARD_TO_ENTITY(getBehaviorParameter, const);
  FORWARD_TO_ENTITY(getCurrentAccel, const);
  FORWARD_TO_ENTITY(getCurrentAction, const);
  FORWARD_TO_ENTITY(getCurrentTwist, const);
  FORWARD_TO_ENTITY(getDistanceToLaneBound, );
  FORWARD_TO_ENTITY(getDistanceToLaneBound, const);
  FORWARD_TO_ENTITY(getDistanceToLeftLaneBound, );
  FORWARD_TO_ENTITY(getDistanceToLeftLaneBound, const);
  FORWARD_TO_ENTITY(getDistanceToRightLaneBound, );
  FORWARD_TO_ENTITY(getDistanceToRightLaneBound, const);
  FORWARD_TO_ENTITY(getEntityStatusBeforeUpdate, const);
  FORWARD_TO_ENTITY(fillLaneletPose, const);
  FORWARD_TO_ENTITY(getLaneletPose, const);
  FORWARD_TO_ENTITY(getLinearJerk, const);
  FORWARD_TO_ENTITY(getMapPose, const);
  FORWARD_TO_ENTITY(getRouteLanelets, );
  FORWARD_TO_ENTITY(getStandStillDuration, const);
  FORWARD_TO_ENTITY(getTraveledDistance, const);
  FORWARD_TO_ENTITY(requestAcquirePosition, );
  FORWARD_TO_ENTITY(requestAssignRoute, );
  FORWARD_TO_ENTITY(requestFollowTrajectory, );
  FORWARD_TO_ENTITY(requestLaneChange, );
  FORWARD_TO_ENTITY(requestWalkStraight, );
  FORWARD_TO_ENTITY(activateOutOfRangeJob, );
  FORWARD_TO_ENTITY(setAccelerationLimit, );
  FORWARD_TO_ENTITY(setAccelerationRateLimit, );
  FORWARD_TO_ENTITY(setBehaviorParameter, );
  FORWARD_TO_ENTITY(setDecelerationLimit, );
  FORWARD_TO_ENTITY(setDecelerationRateLimit, );
  FORWARD_TO_ENTITY(setLinearVelocity, );
  FORWARD_TO_ENTITY(setVelocityLimit, );

#undef FORWARD_TO_ENTITY

  visualization_msgs::msg::MarkerArray makeDebugMarker() const;

  bool trafficLightsChanged();

  void requestSpeedChange(const std::string & name, double target_speed, bool continuous);

  void requestSpeedChange(
    const std::string & name, const double target_speed, const speed_change::Transition transition,
    const speed_change::Constraint constraint, const bool continuous);

  void requestSpeedChange(
    const std::string & name, const speed_change::RelativeTargetSpeed & target_speed,
    bool continuous);

  void requestSpeedChange(
    const std::string & name, const speed_change::RelativeTargetSpeed & target_speed,
    const speed_change::Transition transition, const speed_change::Constraint constraint,
    const bool continuous);

  traffic_simulator_msgs::msg::EntityStatus updateNpcLogic(
    const std::string & name,
    const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType> & type_list);

  void broadcastEntityTransform();

  void broadcastTransform(
    const geometry_msgs::msg::PoseStamped & pose, const bool static_transform = true);

  bool checkCollision(const std::string & name0, const std::string & name1);

  bool despawnEntity(const std::string & name);

  bool entityExists(const std::string & name);

  bool laneMatchingSucceed(const std::string & name) const;

  auto getBoundingBoxDistance(const std::string & from, const std::string & to)
    -> std::optional<double>;

  auto getCurrentTime() const noexcept -> double;

  auto getDistanceToCrosswalk(const std::string & name, const std::int64_t target_crosswalk_id)
    -> std::optional<double>;

  auto getDistanceToStopLine(const std::string & name, const std::int64_t target_stop_line_id)
    -> std::optional<double>;

  auto getEntityNames() const -> const std::vector<std::string>;

  auto getEntityStatus(const std::string & name) const -> traffic_simulator_msgs::msg::EntityStatus;

  auto getEntityTypeList() const
    -> const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType>;

  auto getHdmapUtils() -> const std::shared_ptr<hdmap_utils::HdMapUtils> &;

  // clang-format off
  auto getLateralDistance(const LaneletPose &, const LaneletPose &) const -> std::optional<double>;
  auto getLateralDistance(const LaneletPose &, const std::string &) const -> std::optional<double>;
  auto getLateralDistance(const std::string &, const LaneletPose &) const -> std::optional<double>;
  auto getLateralDistance(const std::string &, const std::string &) const -> std::optional<double>;
  auto getLateralDistance(const LaneletPose &, const LaneletPose &, double matching_distance) const -> std::optional<double>;
  auto getLateralDistance(const LaneletPose &, const std::string &, double matching_distance) const -> std::optional<double>;
  auto getLateralDistance(const std::string &, const LaneletPose &, double matching_distance) const -> std::optional<double>;
  auto getLateralDistance(const std::string &, const std::string &, double matching_distance) const -> std::optional<double>;
  auto getLongitudinalDistance(const LaneletPose &, const LaneletPose &) const -> std::optional<double>;
  auto getLongitudinalDistance(const LaneletPose &, const std::string &) const -> std::optional<double>;
  auto getLongitudinalDistance(const std::string &, const LaneletPose &) const -> std::optional<double>;
  auto getLongitudinalDistance(const std::string &, const std::string &) const -> std::optional<double>;
  // clang-format on

  auto getNumberOfEgo() const -> std::size_t;

  auto getObstacle(const std::string & name)
    -> std::optional<traffic_simulator_msgs::msg::Obstacle>;

  // clang-format off
  auto getRelativePose(const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to) const -> geometry_msgs::msg::Pose;
  auto getRelativePose(const geometry_msgs::msg::Pose & from, const std::string              & to) const -> geometry_msgs::msg::Pose;
  auto getRelativePose(const std::string              & from, const geometry_msgs::msg::Pose & to) const -> geometry_msgs::msg::Pose;
  auto getRelativePose(const std::string              & from, const std::string              & to) const -> geometry_msgs::msg::Pose;
  auto getRelativePose(const geometry_msgs::msg::Pose & from, const LaneletPose              & to) const -> geometry_msgs::msg::Pose;
  auto getRelativePose(const LaneletPose              & from, const geometry_msgs::msg::Pose & to) const -> geometry_msgs::msg::Pose;
  auto getRelativePose(const std::string              & from, const LaneletPose              & to) const -> geometry_msgs::msg::Pose;
  auto getRelativePose(const LaneletPose              & from, const std::string              & to) const -> geometry_msgs::msg::Pose;
  // clang-format on

  auto getStepTime() const noexcept -> double;

  auto getWaypoints(const std::string & name) -> traffic_simulator_msgs::msg::WaypointsArray;

  template <typename T>
  auto getGoalPoses(const std::string & name) -> std::vector<T>
  {
    if constexpr (std::is_same_v<std::decay_t<T>, traffic_simulator_msgs::msg::LaneletPose>) {
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
        for (const auto & lanelet_pose :
             getGoalPoses<traffic_simulator_msgs::msg::LaneletPose>(name)) {
          poses.push_back(toMapPose(lanelet_pose));
        }
        return poses;
      }
    }
  }

  bool isEgo(const std::string & name) const;

  bool isEgoSpawned() const;

  const std::string getEgoName() const;

  bool isInLanelet(const std::string & name, const std::int64_t lanelet_id, const double tolerance);

  bool isStopping(const std::string & name) const;

  bool reachPosition(
    const std::string & name, const geometry_msgs::msg::Pose & target_pose,
    const double tolerance) const;
  bool reachPosition(
    const std::string & name, const std::int64_t lanelet_id, const double s, const double offset,
    const double tolerance) const;
  bool reachPosition(
    const std::string & name, const std::string & target_name, const double tolerance) const;

  void requestLaneChange(
    const std::string & name, const traffic_simulator::lane_change::Direction & direction);

  auto setEntityStatus(const std::string & name, const traffic_simulator_msgs::msg::EntityStatus &)
    -> void;

  auto setEntityStatusExternally(
    const std::string & name, const traffic_simulator_msgs::msg::EntityStatus &) -> void;

  void setVerbose(const bool verbose);

  template <typename Entity, typename Pose, typename Parameters, typename... Ts>
  auto spawnEntity(
    const std::string & name, const Pose & pose, const Parameters & parameters, Ts &&... xs)
  {
    auto makeEntityStatus = [&]() {
      traffic_simulator_msgs::msg::EntityStatus entity_status;

      if constexpr (std::is_same_v<std::decay_t<Entity>, EgoEntity>) {
        if (auto iter = std::find_if(
              std::begin(entities_), std::end(entities_),
              [this](auto && each) { return isEgo(each.first); });
            iter != std::end(entities_)) {
          THROW_SEMANTIC_ERROR("multi ego simulation does not support yet");
        } else {
          entity_status.type.type = traffic_simulator_msgs::msg::EntityType::EGO;
        }
      } else if constexpr (std::is_same_v<std::decay_t<Entity>, VehicleEntity>) {
        entity_status.type.type = traffic_simulator_msgs::msg::EntityType::VEHICLE;
      } else if constexpr (std::is_same_v<std::decay_t<Entity>, PedestrianEntity>) {
        entity_status.type.type = traffic_simulator_msgs::msg::EntityType::PEDESTRIAN;
      } else {
        entity_status.type.type = traffic_simulator_msgs::msg::EntityType::MISC_OBJECT;
      }

      entity_status.subtype = parameters.subtype;

      entity_status.time = getCurrentTime();

      entity_status.name = name;

      entity_status.bounding_box = parameters.bounding_box;

      entity_status.action_status = traffic_simulator_msgs::msg::ActionStatus();

      if constexpr (std::is_same_v<std::decay_t<Pose>, traffic_simulator_msgs::msg::LaneletPose>) {
        entity_status.pose = toMapPose(pose);
        entity_status.lanelet_pose = pose;
        entity_status.lanelet_pose_valid = true;
      } else {
        entity_status.pose = pose;

        if (const auto lanelet_pose = toLaneletPose(pose, parameters.bounding_box, false);
            lanelet_pose) {
          entity_status.lanelet_pose = *lanelet_pose;
          entity_status.lanelet_pose_valid = true;
        } else {
          entity_status.lanelet_pose_valid = false;
        }
      }

      return entity_status;
    };

    if (const auto [iter, success] = entities_.emplace(
          name, std::make_unique<Entity>(
                  name, makeEntityStatus(), parameters, std::forward<decltype(xs)>(xs)...));
        success) {
      iter->second->setHdMapUtils(hdmap_utils_ptr_);
      // FIXME: this ignores V2I traffic lights
      iter->second->setTrafficLightManager(conventional_traffic_light_manager_ptr_);
      if (npc_logic_started_ && not isEgo(name)) {
        iter->second->startNpcLogic();
      }
      return success;
    } else {
      THROW_SEMANTIC_ERROR("Entity ", std::quoted(name), " is already exists.");
    }
  }

  auto toMapPose(const traffic_simulator_msgs::msg::LaneletPose &) const
    -> const geometry_msgs::msg::Pose;

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

  void update(const double current_time, const double step_time);

  void updateHdmapMarker();

  void startNpcLogic();

  auto isNpcLogicStarted() const { return npc_logic_started_; }
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__ENTITY_MANAGER_HPP_
