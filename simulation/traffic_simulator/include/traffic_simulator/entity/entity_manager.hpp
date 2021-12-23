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

#ifndef TRAFFIC_SIMULATOR__ENTITY__ENTITY_MANAGER_HPP_
#define TRAFFIC_SIMULATOR__ENTITY__ENTITY_MANAGER_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <boost/optional.hpp>
#include <memory>
#include <rclcpp/node_interfaces/get_node_topics_interface.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <stdexcept>
#include <string>
#include <traffic_simulator/api/configuration.hpp>
#include <traffic_simulator/data_type/data_types.hpp>
#include <traffic_simulator/entity/ego_entity.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/entity/misc_object_entity.hpp>
#include <traffic_simulator/entity/pedestrian_entity.hpp>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic/traffic_sink.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/driver_model.hpp>
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
  Configuration configuration;

  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface;

  tf2_ros::StaticTransformBroadcaster broadcaster_;
  tf2_ros::TransformBroadcaster base_link_broadcaster_;

  const rclcpp::Clock::SharedPtr clock_ptr_;

  std::unordered_map<std::string, std::unique_ptr<traffic_simulator::entity::EntityBase>> entities_;

  double step_time_;

  double current_time_;

  using EntityStatusWithTrajectoryArray =
    traffic_simulator_msgs::msg::EntityStatusWithTrajectoryArray;
  const rclcpp::Publisher<EntityStatusWithTrajectoryArray>::SharedPtr entity_status_array_pub_ptr_;

  using MarkerArray = visualization_msgs::msg::MarkerArray;
  const rclcpp::Publisher<MarkerArray>::SharedPtr lanelet_marker_pub_ptr_;

  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;

  MarkerArray markers_raw_;

  const std::shared_ptr<TrafficLightManagerBase> traffic_light_manager_ptr_;

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
  auto makeTrafficLightManager(Ts &&... xs) -> std::shared_ptr<TrafficLightManagerBase>
  {
    const auto architecture_type = getParameter<std::string>("architecture_type", "tier4/proposal");

#ifndef SCENARIO_SIMULATOR_V2_BACKWARD_COMPATIBLE_TO_AWF_AUTO
    if (architecture_type == "awf/universe") {
      return std::make_shared<
        TrafficLightManager<autoware_auto_perception_msgs::msg::TrafficSignalArray>>(
        std::forward<decltype(xs)>(xs)...);
    } else
#endif
      // NOTE: This broken indent is due to ament_clang_format.
      if (architecture_type == "tier4/proposal" or architecture_type == "awf/auto") {
      return std::make_shared<
        TrafficLightManager<autoware_perception_msgs::msg::TrafficLightStateArray>>(
        std::forward<decltype(xs)>(xs)...);
    } else {
      std::stringstream what;
      what << "Unexpected architecture_type " << std::quoted(architecture_type) << " given.";
      throw std::invalid_argument(what.str());
    }
  }

  template <class NodeT, class AllocatorT = std::allocator<void>>
  explicit EntityManager(NodeT && node, const Configuration & configuration)
  : configuration(configuration),
    node_topics_interface(rclcpp::node_interfaces::get_node_topics_interface(node)),
    broadcaster_(node),
    base_link_broadcaster_(node),
    clock_ptr_(node->get_clock()),
    current_time_(0),
    entity_status_array_pub_ptr_(rclcpp::create_publisher<EntityStatusWithTrajectoryArray>(
      node, "entity/status", EntityMarkerQoS(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    lanelet_marker_pub_ptr_(rclcpp::create_publisher<MarkerArray>(
      node, "lanelet/marker", LaneletMarkerQoS(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    hdmap_utils_ptr_(std::make_shared<hdmap_utils::HdMapUtils>(
      configuration.lanelet2_map_path(), getOrigin(*node))),
    markers_raw_(hdmap_utils_ptr_->generateMarker()),
    traffic_light_manager_ptr_(makeTrafficLightManager(hdmap_utils_ptr_, node))
  {
    updateHdmapMarker();
  }

  ~EntityManager() = default;

public:
#define DEFINE_SET_TRAFFIC_LIGHT(NAME)                                               \
  template <typename... Ts>                                                          \
  decltype(auto) setTrafficLight##NAME(Ts &&... xs)                                  \
  {                                                                                  \
    return traffic_light_manager_ptr_->set##NAME(std::forward<decltype(xs)>(xs)...); \
  }                                                                                  \
  static_assert(true, "")

  DEFINE_SET_TRAFFIC_LIGHT(Arrow);
  DEFINE_SET_TRAFFIC_LIGHT(ArrowPhase);
  DEFINE_SET_TRAFFIC_LIGHT(Color);
  DEFINE_SET_TRAFFIC_LIGHT(ColorPhase);

#undef DEFINE_SET_TRAFFIC_LIGHT

#define DEFINE_GET_TRAFFIC_LIGHT(NAME)                                               \
  template <typename... Ts>                                                          \
  decltype(auto) getTrafficLight##NAME(Ts &&... xs)                                  \
  {                                                                                  \
    return traffic_light_manager_ptr_->get##NAME(std::forward<decltype(xs)>(xs)...); \
  }                                                                                  \
  static_assert(true, "")

  DEFINE_GET_TRAFFIC_LIGHT(Color);
  DEFINE_GET_TRAFFIC_LIGHT(Arrow);
  DEFINE_GET_TRAFFIC_LIGHT(Ids);
  DEFINE_GET_TRAFFIC_LIGHT(Instance);

#undef DEFINE_GET_TRAFFIC_LIGHT

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

  FORWARD_TO_ENTITY(cancelRequest, );
  FORWARD_TO_ENTITY(engage, );
  FORWARD_TO_ENTITY(getBoundingBox, const);
  FORWARD_TO_ENTITY(getCurrentAction, const);
  FORWARD_TO_ENTITY(getDriverModel, const);
  FORWARD_TO_ENTITY(getEntityStatusBeforeUpdate, const);
  FORWARD_TO_ENTITY(getEntityType, const);
  FORWARD_TO_ENTITY(getLinearJerk, const);
  FORWARD_TO_ENTITY(getRouteLanelets, );
  FORWARD_TO_ENTITY(getStandStillDuration, const);
  FORWARD_TO_ENTITY(getVehicleCommand, const);
  FORWARD_TO_ENTITY(getVehicleParameters, const);
  FORWARD_TO_ENTITY(ready, const);
  FORWARD_TO_ENTITY(requestAcquirePosition, );
  FORWARD_TO_ENTITY(requestAssignRoute, );
  FORWARD_TO_ENTITY(requestLaneChange, );
  FORWARD_TO_ENTITY(requestWalkStraight, );
  FORWARD_TO_ENTITY(setAccelerationLimit, );
  FORWARD_TO_ENTITY(setDecelerationLimit, );
  FORWARD_TO_ENTITY(setDriverModel, );
  FORWARD_TO_ENTITY(setUpperBoundSpeed, );

#undef FORWARD_TO_SPECIFIED_ENTITY

  visualization_msgs::msg::MarkerArray makeDebugMarker() const;

  bool trafficLightsChanged();

  void setTargetSpeed(const std::string & name, double target_speed, bool continuous);

  void requestSpeedChange(
    const std::string & name, const double target_speed, const SpeedChangeTransition transition,
    const SpeedChangeConstraint constraint, const bool continuous);

  traffic_simulator_msgs::msg::EntityStatus updateNpcLogic(
    const std::string & name,
    const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType> & type_list);

  void broadcastEntityTransform();

  void broadcastTransform(
    const geometry_msgs::msg::PoseStamped & pose, const bool static_transform = true);

  bool checkCollision(const std::string & name0, const std::string & name1);

  bool despawnEntity(const std::string & name);

  bool entityExists(const std::string & name);

  bool laneMatchingSucceed(const std::string & name);

  // TODO (yamacir-kit) Rename to 'hasEntityStatus'
  bool entityStatusSet(const std::string & name) const;

  auto getBoundingBoxDistance(const std::string & from, const std::string & to)
    -> boost::optional<double>;

  auto getCurrentTime() const noexcept -> double;

  auto getDistanceToCrosswalk(const std::string & name, const std::int64_t target_crosswalk_id)
    -> boost::optional<double>;

  auto getDistanceToStopLine(const std::string & name, const std::int64_t target_stop_line_id)
    -> boost::optional<double>;

  auto getEntityNames() const -> const std::vector<std::string>;

  auto getEntityStatus(const std::string & name) const
    -> const boost::optional<traffic_simulator_msgs::msg::EntityStatus>;

  auto getEntityTypeList() const
    -> const std::unordered_map<std::string, traffic_simulator_msgs::msg::EntityType>;

  auto getHdmapUtils() -> const std::shared_ptr<hdmap_utils::HdMapUtils> &;

  auto getLaneletPose(const std::string & name)
    -> boost::optional<traffic_simulator_msgs::msg::LaneletPose>;

  // clang-format off
  auto getLongitudinalDistance(const LaneletPose &, const LaneletPose &, const double = 100) -> boost::optional<double>;
  auto getLongitudinalDistance(const LaneletPose &, const std::string &, const double = 100) -> boost::optional<double>;
  auto getLongitudinalDistance(const std::string &, const LaneletPose &, const double = 100) -> boost::optional<double>;
  auto getLongitudinalDistance(const std::string &, const std::string &, const double = 100) -> boost::optional<double>;
  // clang-format on

  auto getMapPose(const std::string & entity_name) -> geometry_msgs::msg::Pose;
  auto getMapPose(
    const std::string & reference_entity_name, const geometry_msgs::msg::Pose & relative_pose)
    -> geometry_msgs::msg::Pose;

  auto getNumberOfEgo() const -> std::size_t;

  auto getObstacle(const std::string & name)
    -> boost::optional<traffic_simulator_msgs::msg::Obstacle>;

  // clang-format off
  auto getRelativePose(const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to) const -> geometry_msgs::msg::Pose;
  auto getRelativePose(const geometry_msgs::msg::Pose & from, const std::string              & to)       -> geometry_msgs::msg::Pose;
  auto getRelativePose(const std::string              & from, const geometry_msgs::msg::Pose & to)       -> geometry_msgs::msg::Pose;
  auto getRelativePose(const std::string              & from, const std::string              & to)       -> geometry_msgs::msg::Pose;
  // clang-format on

  auto getStepTime() const noexcept -> double;

  auto getWaypoints(const std::string & name) -> traffic_simulator_msgs::msg::WaypointsArray;

  void getGoalPoses(
    const std::string & name, std::vector<traffic_simulator_msgs::msg::LaneletPose> & goals);

  void getGoalPoses(const std::string & name, std::vector<geometry_msgs::msg::Pose> & goals);

  bool isEgo(const std::string & name) const;

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

  void requestLaneChange(const std::string & name, const Direction & direction);

  bool setEntityStatus(const std::string & name, traffic_simulator_msgs::msg::EntityStatus status);

  void setVerbose(const bool verbose);

  template <typename Entity, typename... Ts>
  auto spawnEntity(const std::string & name, Ts &&... xs)
  {
    const auto result =
      entities_.emplace(name, std::make_unique<Entity>(name, std::forward<decltype(xs)>(xs)...));
    if (result.second) {
      result.first->second->setHdMapUtils(hdmap_utils_ptr_);
      result.first->second->setTrafficLightManager(traffic_light_manager_ptr_);
      return result.second;
    } else {
      THROW_SEMANTIC_ERROR("entity : ", name, " is already exists.");
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
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__ENTITY_MANAGER_HPP_
