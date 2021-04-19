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

#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <boost/optional.hpp>
#include <map>
#include <memory>
#include <openscenario_msgs/msg/bounding_box.hpp>
#include <openscenario_msgs/msg/driver_model.hpp>
#include <openscenario_msgs/msg/entity_status_with_trajectory_array.hpp>
#include <openscenario_msgs/msg/vehicle_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>  // TODO(yamacir-kit): Remove this!
#include <string>
#include <traffic_simulator/entity/ego_entity.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/entity/exception.hpp>
#include <traffic_simulator/entity/pedestrian_entity.hpp>
#include <traffic_simulator/entity/vehicle_entity.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator/traffic/traffic_sink.hpp>
#include <traffic_simulator/traffic_lights/traffic_light_manager.hpp>
#include <type_traits>
#include <typeinfo>
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
private:
  bool verbose_;

  tf2_ros::StaticTransformBroadcaster broadcaster_;
  tf2_ros::TransformBroadcaster base_link_broadcaster_;

  rclcpp::Clock::SharedPtr clock_ptr_;

  // std::unordered_map<std::string, boost::any> entities_;
  std::unordered_map<std::string, std::unique_ptr<traffic_simulator::entity::EntityBase>> entities_;

  // rclcpp::TimerBase::SharedPtr hdmap_marker_timer_;

  boost::optional<autoware_auto_msgs::msg::VehicleControlCommand> control_cmd_;
  boost::optional<autoware_auto_msgs::msg::VehicleStateCommand> state_cmd_;

  double step_time_;
  double current_time_;

  using EntityStatusWithTrajectoryArray = openscenario_msgs::msg::EntityStatusWithTrajectoryArray;
  rclcpp::Publisher<EntityStatusWithTrajectoryArray>::SharedPtr entity_status_array_pub_ptr_;

  using MarkerArray = visualization_msgs::msg::MarkerArray;
  rclcpp::Publisher<MarkerArray>::SharedPtr lanelet_marker_pub_ptr_;
  MarkerArray markers_raw_;

  using VehicleKinematicState = autoware_auto_msgs::msg::VehicleKinematicState;
  rclcpp::Publisher<VehicleKinematicState>::SharedPtr kinematic_state_pub_ptr_;

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;

  std::shared_ptr<TrafficLightManager> traffic_light_manager_ptr_;

  std::size_t getNumberOfEgo() const;

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

public:
  template <class NodeT, class AllocatorT = std::allocator<void>>
  explicit EntityManager(NodeT && node, const std::string & map_path)
  : verbose_(false),
    broadcaster_(node),
    base_link_broadcaster_(node),
    clock_ptr_(node->get_clock()),
    entity_status_array_pub_ptr_(rclcpp::create_publisher<EntityStatusWithTrajectoryArray>(
      node, "entity/status", EntityMarkerQoS(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    lanelet_marker_pub_ptr_(rclcpp::create_publisher<MarkerArray>(
      node, "lanelet/marker", LaneletMarkerQoS(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())),
    kinematic_state_pub_ptr_(rclcpp::create_publisher<VehicleKinematicState>(
      node, "output/kinematic_state", LaneletMarkerQoS(),
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>()))
  {
    geographic_msgs::msg::GeoPoint origin;
    {
      node->declare_parameter("origin_latitude", 0.0);
      node->declare_parameter("origin_longitude", 0.0);
      // node->declare_parameter("origin_altitude", 0.0);
      node->get_parameter("origin_latitude", origin.latitude);
      node->get_parameter("origin_longitude", origin.longitude);
      // node->get_parameter("origin_altitude", origin.altitude);
      node->undeclare_parameter("origin_latitude");
      node->undeclare_parameter("origin_longitude");
      // node->undeclare_parameter("origin_altitude");
    }

    hdmap_utils_ptr_ = std::make_shared<hdmap_utils::HdMapUtils>(map_path, origin);

    markers_raw_ = hdmap_utils_ptr_->generateMarker();

    updateHdmapMarker();

    traffic_light_manager_ptr_ = std::make_shared<TrafficLightManager>(
      hdmap_utils_ptr_,
      rclcpp::create_publisher<MarkerArray>(node, "traffic_light/marker", LaneletMarkerQoS()),
      rclcpp::create_publisher<autoware_perception_msgs::msg::TrafficLightStateArray>(
        node, "/awapi/traffic_light/put/traffic_light_status", rclcpp::QoS(10).transient_local()),
      clock_ptr_);
  }

  ~EntityManager() = default;

  void updateHdmapMarker()
  {
    MarkerArray markers;
    const auto stamp = clock_ptr_->now();
    for (const auto & marker_raw : markers_raw_.markers) {
      visualization_msgs::msg::Marker marker = marker_raw;
      marker.header.stamp = stamp;
      markers.markers.emplace_back(marker);
    }
    lanelet_marker_pub_ptr_->publish(markers);
  }

  const std::shared_ptr<hdmap_utils::HdMapUtils> getHdmapUtils();

  boost::optional<double> getLinearJerk(const std::string & name) const;

  double getStepTime() const noexcept;

  double getCurrentTime() const noexcept;

  void setDriverModel(const std::string & name, const openscenario_msgs::msg::DriverModel & model);

  const openscenario_msgs::msg::BoundingBox getBoundingBox(const std::string & name) const;

  const geometry_msgs::msg::Pose toMapPose(
    const openscenario_msgs::msg::LaneletPose & lanelet_pose) const;

  bool checkCollision(const std::string & name0, const std::string & name1);

  void setVerbose(bool verbose);

  void requestAcquirePosition(
    const std::string & name, const openscenario_msgs::msg::LaneletPose & lanelet_pose);

  void requestAssignRoute(
    const std::string & name, const std::vector<openscenario_msgs::msg::LaneletPose> & waypoints);

  void requestLaneChange(const std::string & name, std::int64_t to_lanelet_id);
  void requestLaneChange(const std::string & name, const Direction & direction);

  void requestWalkStraight(const std::string & name);

  std::vector<std::int64_t> getConflictingEntityOnRouteLanelets(
    const std::string & name, const double horizon);

  std::vector<std::int64_t> getRouteLanelets(const std::string & name, const double horizon);

  openscenario_msgs::msg::WaypointsArray getWaypoints(const std::string & name);

  boost::optional<openscenario_msgs::msg::Obstacle> getObstacle(const std::string & name);

  boost::optional<double> getLongitudinalDistance(
    const std::string & from, const std::string & to, const double max_distance = 100);

  boost::optional<double> getBoundingBoxDistance(const std::string & from, const std::string & to);

  geometry_msgs::msg::Pose getRelativePose(const std::string & from, const std::string & to);
  geometry_msgs::msg::Pose getRelativePose(
    const std::string & from, const geometry_msgs::msg::Pose & to);
  geometry_msgs::msg::Pose getRelativePose(
    const geometry_msgs::msg::Pose & from, const std::string & to);
  geometry_msgs::msg::Pose getRelativePose(
    const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to) const;

  geometry_msgs::msg::Pose getMapPose(const std::string & entity_name);
  geometry_msgs::msg::Pose getMapPose(
    const std::string & reference_entity_name, const geometry_msgs::msg::Pose & relative_pose);

  const boost::optional<openscenario_msgs::msg::VehicleParameters> getVehicleParameters(
    const std::string & name) const;

  const std::vector<std::string> getEntityNames() const;

  bool setEntityStatus(const std::string & name, openscenario_msgs::msg::EntityStatus status);

  const boost::optional<openscenario_msgs::msg::EntityStatus> getEntityStatus(
    const std::string & name) const;

  boost::optional<double> getSValueInRoute(
    const std::string & name, const std::vector<std::int64_t> & route);

  bool isInLanelet(const std::string & name, const std::int64_t lanelet_id, const double tolerance);

  bool entityStatusSet(const std::string & name) const;

  void setTargetSpeed(const std::string & name, const double target_speed, const bool continuous);

  void update(const double current_time, const double step_time);

  void broadcastTransform(
    const geometry_msgs::msg::PoseStamped & pose, const bool static_transform = true);

  boost::optional<double> getDistanceToStopLine(
    const std::string & name, const std::int64_t target_stop_line_id);

  boost::optional<double> getDistanceToCrosswalk(
    const std::string & name, const std::int64_t target_crosswalk_id);

  bool reachPosition(
    const std::string & name, const geometry_msgs::msg::Pose & target_pose,
    const double tolerance) const;
  bool reachPosition(
    const std::string & name, const std::int64_t lanelet_id, const double s, const double offset,
    const double tolerance) const;
  bool reachPosition(
    const std::string & name, const std::string & target_name, const double tolerance) const;

  void broadcastEntityTransform();

  void broadcastBaseLinkTransform();

  const boost::optional<double> getStandStillDuration(const std::string & name) const;

  bool isStopping(const std::string & name) const;

  const std::unordered_map<std::string, openscenario_msgs::msg::EntityType> getEntityTypeList()
    const;

  bool isEgo(const std::string & name) const;

  openscenario_msgs::msg::EntityType getEntityType(const std::string & name) const;

  const std::string getCurrentAction(const std::string & name) const;

  boost::optional<openscenario_msgs::msg::LaneletPose> getLaneletPose(const std::string & name);

  template <
    typename Entity, typename = typename std::enable_if<
                       std::is_base_of<EntityBase, typename std::decay<Entity>::type>::value>::type>
  bool spawnEntity(Entity && entity)
  {
    if (entities_.count(entity.name) != 0) {
      throw traffic_simulator::SimulationRuntimeError("entity " + entity.name + " already exist.");
    } else {
      entity.setHdMapUtils(hdmap_utils_ptr_);
      entity.setTrafficLightManager(traffic_light_manager_ptr_);
      entities_.emplace(
        entity.name, std::make_unique<typename std::decay<Entity>::type>(
                       std::forward<decltype(entity)>(entity)));
      return true;
    }
  }

  bool despawnEntity(const std::string & name)
  {
    return entityExists(name) && entities_.erase(name);
  }

  bool entityExists(const std::string & name)
  {
    return entities_.find(name) != std::end(entities_);
  }

  decltype(auto) reference(const std::string & name)
  {
    const auto iter = entities_.find(name);

    if (iter != std::end(entities_)) {
      return std::get<1>(*iter);
    } else {
      std::stringstream ss{};
      ss << "Unknown entity '" << name << "' has been referenced.";
      ss << "Check the scenario.";
      throw SimulationRuntimeError(ss.str());
    }
  }
};
}  // namespace entity
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR__ENTITY__ENTITY_MANAGER_HPP_
