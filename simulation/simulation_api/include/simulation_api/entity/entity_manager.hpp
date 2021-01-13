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

#include <openscenario_msgs/msg/entity_status_with_trajectory_array.hpp>
#include <openscenario_msgs/msg/bounding_box.hpp>

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

class LaneletMarkerQos : public rclcpp::QoS
{
public:
  explicit LaneletMarkerQos(size_t depth = 1)
  : rclcpp::QoS(depth)
  {
    transient_local();
  }
};

class EntityMarkerQos : public rclcpp::QoS
{
public:
  explicit EntityMarkerQos(size_t depth = 100)
  : rclcpp::QoS(depth) {}
};

class EntityManager
{
private:
  bool verbose_;
  std::map<std::string, boost::any> entities_;
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
  rclcpp::Clock::SharedPtr clock_ptr_;
  visualization_msgs::msg::MarkerArray markers_raw_;
  rclcpp::TimerBase::SharedPtr hdmap_marker_timer_;
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
  int getNumberOfEgo() const;
  boost::optional<autoware_auto_msgs::msg::VehicleControlCommand> control_cmd_;
  boost::optional<autoware_auto_msgs::msg::VehicleStateCommand> state_cmd_;

public:
  template<class NodeT, class AllocatorT = std::allocator<void>>
  explicit EntityManager(NodeT && node, const std::string & map_path = "")
  : broadcaster_(node), base_link_broadcaster_(node)
  {
    clock_ptr_ = node->get_clock();
    geographic_msgs::msg::GeoPoint origin;
    node->declare_parameter("origin_latitude", 0.0);
    node->declare_parameter("origin_longitude", 0.0);
    // node->declare_parameter("origin_altitude", 0.0);
    node->get_parameter("origin_latitude", origin.latitude);
    node->get_parameter("origin_longitude", origin.longitude);
    // node->get_parameter("origin_altitude", origin.altitude);
    node->undeclare_parameter("origin_latitude");
    node->undeclare_parameter("origin_longitude");
    // node->undeclare_parameter("origin_altitude");
    hdmap_utils_ptr_ = std::make_shared<hdmap_utils::HdMapUtils>(map_path, origin);
    const rclcpp::QoS & qos = LaneletMarkerQos();
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options =
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>();
    lanelet_marker_pub_ptr_ = rclcpp::create_publisher
      <visualization_msgs::msg::MarkerArray>(
      node,
      "lanelet/marker", qos,
      options);
    kinematic_state_pub_ptr_ = rclcpp::create_publisher
      <autoware_auto_msgs::msg::VehicleKinematicState>(
      node,
      "output/kinematic_state", qos,
      options);
    const rclcpp::QoS & entity_marker_qos = EntityMarkerQos();
    entity_status_array_pub_ptr_ =
      rclcpp::create_publisher<openscenario_msgs::msg::EntityStatusWithTrajectoryArray>(
      node,
      "entity/status", entity_marker_qos,
      options);
    visualization_msgs::msg::MarkerArray markers;
    markers_raw_ = hdmap_utils_ptr_->generateMarker();

    hdmap_marker_timer_ =
      node->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&EntityManager::updateHdmapMarker, this));
  }
  ~EntityManager() {}
  const openscenario_msgs::msg::BoundingBox getBoundingBox(std::string name) const;
  const boost::optional<openscenario_msgs::msg::LaneletPose> toLaneletPose(
    geometry_msgs::msg::Pose pose) const;
  const geometry_msgs::msg::Pose toMapPose(const openscenario_msgs::msg::LaneletPose lanelet_pose)
  const;
  bool checkCollision(std::string name0, std::string name1);
  void setVehicleCommands(
    boost::optional<autoware_auto_msgs::msg::VehicleControlCommand> control_cmd,
    boost::optional<autoware_auto_msgs::msg::VehicleStateCommand> state_cmd);
  void setVerbose(bool verbose);
  void requestAcquirePosition(std::string name, openscenario_msgs::msg::LaneletPose lanelet_pose);
  void requestLaneChange(std::string name, std::int64_t to_lanelet_id);
  void requestLaneChange(std::string name, Direction direction);
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
  const boost::optional<VehicleParameters> getVehicleParameters(std::string name) const;
  const std::vector<std::string> getEntityNames() const;
  bool setEntityStatus(std::string name, openscenario_msgs::msg::EntityStatus status);
  const boost::optional<openscenario_msgs::msg::EntityStatus> getEntityStatus(
    std::string name) const;
  bool isInLanelet(std::string name, std::int64_t lanelet_id, double tolerance);
  bool entityStatusSetted(std::string name) const;
  void setTargetSpeed(std::string name, double target_speed, bool continuous);
  void update(double current_time, double step_time);
  void broadcastTransform(geometry_msgs::msg::PoseStamped pose, bool static_transform = true);
  bool reachPosition(
    std::string name, geometry_msgs::msg::Pose target_pose,
    double tolerance) const;
  bool reachPosition(
    std::string name, std::int64_t lanelet_id, double s, double offset,
    double tolerance) const;
  void broadcastEntityTransform();
  void broadcastBaseLinkTransform();
  const boost::optional<double> getStandStillDuration(std::string name) const;
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
  template<typename T>
  bool spawnEntity(T & entity)
  {
    if (entities_.count(entity.name) != 0) {
      throw simulation_api::SimulationRuntimeError(
              "entity " + entity.name + " already exist.");
    }
    if (std::is_base_of<EntityBase, T>::value == false) {
      return false;
    }
    entity.setHdMapUtils(hdmap_utils_ptr_);
    entities_.insert(std::make_pair(entity.name, entity));
    return true;
  }
  bool despawnEntity(std::string name)
  {
    if (entities_.count(name) == 0) {
      return false;
    }
    entities_.erase(name);
    return true;
  }
  bool entityExists(std::string name)
  {
    if (entities_.count(name) == 0) {
      return false;
    }
    return true;
  }
};
}  // namespace entity
}  // namespace simulation_api

#endif   // SIMULATION_API__ENTITY__ENTITY_MANAGER_HPP_
