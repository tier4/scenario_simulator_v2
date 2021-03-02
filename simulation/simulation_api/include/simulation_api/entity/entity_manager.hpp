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

#ifndef  SIMULATION_API__ENTITY__ENTITY_MANAGER_HPP_
#define  SIMULATION_API__ENTITY__ENTITY_MANAGER_HPP_

#include <simulation_api/entity/ego_entity.hpp>
#include <simulation_api/entity/vehicle_entity.hpp>
#include <simulation_api/entity/pedestrian_entity.hpp>
#include <simulation_api/entity/exception.hpp>
#include <simulation_api/hdmap_utils/hdmap_utils.hpp>
#include <simulation_api/traffic_lights/traffic_light_manager.hpp>

#include <openscenario_msgs/msg/vehicle_parameters.hpp>
#include <openscenario_msgs/msg/entity_status_with_trajectory_array.hpp>
#include <openscenario_msgs/msg/bounding_box.hpp>
#include <openscenario_msgs/msg/driver_model.hpp>

#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp/rclcpp.hpp>

#include <boost/any.hpp>
#include <boost/optional.hpp>
#include <type_traits>
#include <typeinfo>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

namespace simulation_api
{
namespace entity
{

class LaneletMarkerQoS : public rclcpp::QoS
{
public:
  explicit LaneletMarkerQoS(std::size_t depth = 1)
  : rclcpp::QoS(depth)
  {
    transient_local();
  }
};

class EntityMarkerQoS : public rclcpp::QoS
{
public:
  explicit EntityMarkerQoS(std::size_t depth = 100)
  : rclcpp::QoS(depth) {}
};

class EntityManager
{
private:
  bool verbose_;

  std::unordered_map<std::string, boost::any> entities_;

  rclcpp::Clock::SharedPtr clock_ptr_;

  visualization_msgs::msg::MarkerArray markers_raw_;
  rclcpp::TimerBase::SharedPtr hdmap_marker_timer_;

  std::size_t getNumberOfEgo() const;

  boost::optional<autoware_auto_msgs::msg::VehicleControlCommand> control_cmd_;
  boost::optional<autoware_auto_msgs::msg::VehicleStateCommand> state_cmd_;

  double step_time_;
  double current_time_;

  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
  std::shared_ptr<TrafficLightManager> traffic_light_manager_ptr_;

public:
  template<typename ... Ts>
  decltype(auto) setTrafficLightColorPhase(Ts && ... xs)
  {
    traffic_light_manager_ptr_->setColorPhase(std::forward<Ts>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) setTrafficLightArrowPhase(Ts && ... xs)
  {
    traffic_light_manager_ptr_->setArrowPhase(std::forward<Ts>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) setTrafficLightColor(Ts && ... xs)
  {
    traffic_light_manager_ptr_->setColor(std::forward<Ts>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) setTrafficLightArrow(Ts && ... xs)
  {
    traffic_light_manager_ptr_->setArrow(std::forward<Ts>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) getTrafficLightColor(Ts && ... xs)
  {
    return traffic_light_manager_ptr_->getColor(std::forward<Ts>(xs)...);
  }

  template<typename ... Ts>
  decltype(auto) getTrafficLightArrow(Ts && ... xs)
  {
    return traffic_light_manager_ptr_->getArrow(std::forward<Ts>(xs)...);
  }

#define FORWARD_TO_HDMAP_UTILS(NAME) \
  template<typename ... Ts> \
  decltype(auto) NAME(Ts && ... xs) const \
  { \
    return hdmap_utils_ptr_->NAME(std::forward<decltype(xs)>(xs)...); \
  } static_assert(true, "")

  FORWARD_TO_HDMAP_UTILS(toLaneletPose);
  // FORWARD_TO_HDMAP_UTILS(toMapPose);

public:
  template<class NodeT, class AllocatorT = std::allocator<void>>
  explicit EntityManager(NodeT && node, const std::string & map_path)
  : broadcaster_(node),
    base_link_broadcaster_(node),
    clock_ptr_(node->get_clock())
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

    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> options =
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>();

    lanelet_marker_pub_ptr_ =
      rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      node, "lanelet/marker", LaneletMarkerQoS(), options);

    kinematic_state_pub_ptr_ =
      rclcpp::create_publisher<autoware_auto_msgs::msg::VehicleKinematicState>(
      node, "output/kinematic_state", LaneletMarkerQoS(), options);

    entity_status_array_pub_ptr_ =
      rclcpp::create_publisher<openscenario_msgs::msg::EntityStatusWithTrajectoryArray>(
      node, "entity/status", EntityMarkerQoS(), options);

    markers_raw_ = hdmap_utils_ptr_->generateMarker();

    updateHdmapMarker();

    const auto traffic_light_marker_pub =
      rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      node, "traffic_light/marker", LaneletMarkerQoS(), options);

    traffic_light_manager_ptr_ = std::make_shared<TrafficLightManager>(
      hdmap_utils_ptr_, traffic_light_marker_pub, clock_ptr_);
  }

  ~EntityManager() = default;

  void updateHdmapMarker()
  {
    visualization_msgs::msg::MarkerArray markers;
    auto stamp = clock_ptr_->now();
    for (const auto & marker_raw : markers_raw_.markers) {
      visualization_msgs::msg::Marker marker = marker_raw;
      marker.header.stamp = stamp;
      markers.markers.emplace_back(marker);
    }
    lanelet_marker_pub_ptr_->publish(markers);
  }

  boost::optional<double> getLinearJerk(std::string name);

  double getStepTime() const noexcept;
  double getCurrentTime() const noexcept;

  void setDriverModel(
    const std::string & name,
    const openscenario_msgs::msg::DriverModel & model);

  const openscenario_msgs::msg::BoundingBox getBoundingBox(std::string name) const;

  // const boost::optional<openscenario_msgs::msg::LaneletPose> toLaneletPose(
  //   geometry_msgs::msg::Pose pose) const;
  const geometry_msgs::msg::Pose toMapPose(
    const openscenario_msgs::msg::LaneletPose lanelet_pose) const;

  bool checkCollision(std::string name0, std::string name1);
  void setVerbose(bool verbose);
  void requestAcquirePosition(std::string name, openscenario_msgs::msg::LaneletPose lanelet_pose);
  void requestLaneChange(std::string name, std::int64_t to_lanelet_id);
  void requestLaneChange(std::string name, Direction direction);
  std::vector<std::int64_t> getConflictingEntityOnRouteLanelets(std::string name, double horizon);
  std::vector<std::int64_t> getRouteLanelets(std::string name, double horizon);
  openscenario_msgs::msg::WaypointsArray getWaypoints(std::string name);
  boost::optional<openscenario_msgs::msg::Obstacle> getObstacle(std::string name);
  boost::optional<double> getLongitudinalDistance(
    std::string from, std::string to,
    double max_distance = 100);
  geometry_msgs::msg::Pose getRelativePose(std::string from, std::string to);
  geometry_msgs::msg::Pose getRelativePose(std::string from, geometry_msgs::msg::Pose to);
  geometry_msgs::msg::Pose getRelativePose(geometry_msgs::msg::Pose from, std::string to);
  geometry_msgs::msg::Pose getRelativePose(
    geometry_msgs::msg::Pose from,
    geometry_msgs::msg::Pose to) const;
  geometry_msgs::msg::Pose getMapPose(
    std::string reference_entity_name,
    geometry_msgs::msg::Pose relative_pose);
  const boost::optional<openscenario_msgs::msg::VehicleParameters> getVehicleParameters(
    std::string name) const;
  const std::vector<std::string> getEntityNames() const;
  bool setEntityStatus(std::string name, openscenario_msgs::msg::EntityStatus status);
  const boost::optional<openscenario_msgs::msg::EntityStatus> getEntityStatus(
    std::string name) const;
  boost::optional<double> getSValueInRoute(std::string name, std::vector<std::int64_t> route);
  bool isInLanelet(std::string name, std::int64_t lanelet_id, double tolerance);
  bool entityStatusSetted(std::string name) const;
  void setTargetSpeed(std::string name, double target_speed, bool continuous);
  void update(double current_time, double step_time);
  void broadcastTransform(geometry_msgs::msg::PoseStamped pose, bool static_transform = true);

  boost::optional<double> getDistanceToStopLine(
    std::string name, std::int64_t target_stop_line_id);
  boost::optional<double> getDistanceToCrosswalk(
    std::string name,
    std::int64_t target_crosswalk_id);

  bool reachPosition(
    std::string name, geometry_msgs::msg::Pose target_pose, double tolerance) const;
  bool reachPosition(
    std::string name, std::int64_t lanelet_id, double s, double offset, double tolerance) const;
  bool reachPosition(
    std::string name, std::string target_name, double tolerance) const;

  void broadcastEntityTransform();
  void broadcastBaseLinkTransform();
  const boost::optional<double> getStandStillDuration(std::string name) const;
  bool isStopping(std::string name) const;
  const std::unordered_map<std::string,
    openscenario_msgs::msg::EntityType> getEntityTypeList() const;
  bool isEgo(std::string name) const;
  openscenario_msgs::msg::EntityType getEntityType(std::string name) const;
  const std::string getCurrentAction(std::string name) const;
  tf2_ros::StaticTransformBroadcaster broadcaster_;
  tf2_ros::TransformBroadcaster base_link_broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lanelet_marker_pub_ptr_;
  rclcpp::Publisher<openscenario_msgs::msg::EntityStatusWithTrajectoryArray>::SharedPtr
    entity_status_array_pub_ptr_;
  rclcpp::Publisher<autoware_auto_msgs::msg::VehicleKinematicState>::SharedPtr
    kinematic_state_pub_ptr_;
  boost::optional<openscenario_msgs::msg::LaneletPose> getLaneletPose(std::string name);

  template<
    typename Entity,
    typename = typename std::enable_if<
      std::is_base_of<EntityBase, typename std::decay<Entity>::type>::value
    >::type>
  bool spawnEntity(Entity && entity)
  {
    if (entities_.count(entity.name) != 0) {
      throw simulation_api::SimulationRuntimeError("entity " + entity.name + " already exist.");
    } else {
      entity.setHdMapUtils(hdmap_utils_ptr_);
      entities_.emplace(entity.name, std::forward<decltype(entity)>(entity));
      return true;
    }
  }

  bool despawnEntity(const std::string & name)
  {
    return entityExists(name) && entities_.erase(name);
  }

  bool entityExists(const std::string & name)
  {
    return entities_.count(name) != 0;
  }
};
}  // namespace entity
}  // namespace simulation_api

#endif   // SIMULATION_API__ENTITY__ENTITY_MANAGER_HPP_
