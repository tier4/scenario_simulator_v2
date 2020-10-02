// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef  SIMULATION_CONTROLLER__ENTITY__ENTITY_MANAGER_HPP_
#define  SIMULATION_CONTROLLER__ENTITY__ENTITY_MANAGER_HPP_

#include <simulation_controller/entity/ego_entity.hpp>
#include <simulation_controller/entity/vehicle_entity.hpp>
#include <simulation_controller/entity/pedestrian_entity.hpp>
#include <simulation_controller/entity/exception.hpp>

#include <simulation_controller/hdmap_utils/hdmap_utils.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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

namespace simulation_controller
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
  std::map<std::string, boost::any> entities_;
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;
  rclcpp::Clock::SharedPtr clock_ptr_;

public:
  template<class NodeT, class AllocatorT = std::allocator<void>>
  explicit EntityManager(NodeT && node)
  : broadcaster_(node)
  {
    clock_ptr_ = node->get_clock();
    node->declare_parameter("map_path", "");
    std::string map_path;
    node->get_parameter("map_path", map_path);
    geographic_msgs::msg::GeoPoint origin;
    node->declare_parameter("origin_latitude", 0.0);
    node->declare_parameter("origin_longitude", 0.0);
    // node->declare_parameter("origin_altitude", 0.0);
    node->get_parameter("origin_latitude", origin.latitude);
    node->get_parameter("origin_longitude", origin.longitude);
    // node->get_parameter("origin_altitude", origin.altitude);
    hdmap_utils_ptr_ = std::make_shared<hdmap_utils::HdMapUtils>(map_path, origin);
    const rclcpp::QoS & qos = LaneletMarkerQos();
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options =
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>();
    lanelet_marker_pub_ptr_ = rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(node,
        "lanelet/marker", qos,
        options);
    const rclcpp::QoS & entity_marker_qos = EntityMarkerQos();
    entity_marker_pub_ptr_ = rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(node,
        "entity/marker", entity_marker_qos,
        options);
    visualization_msgs::msg::MarkerArray markers;
    auto markers_raw = hdmap_utils_ptr_->generateMarker();

    auto stamp = clock_ptr_->now();
    for (const auto & marker_raw : markers_raw.markers) {
      visualization_msgs::msg::Marker marker = marker_raw;
      marker.header.stamp = stamp;
      markers.markers.emplace_back(marker);
    }
    lanelet_marker_pub_ptr_->publish(markers);
  }
  void setVerbose(bool verbose);
  void requestAcquirePosition(std::string name, int lanelet_id, double s, double offset);
  void requestLaneChange(std::string name, int to_lanelet_id);
  void requestLaneChange(std::string name, Direction direction);
  boost::optional<double> getLongitudinalDistance(
    std::string from, std::string to,
    double max_distance = 100);
  geometry_msgs::msg::Pose getRelativePose(std::string from, std::string to);
  geometry_msgs::msg::Pose getRelativePose(std::string from, geometry_msgs::msg::Pose to);
  geometry_msgs::msg::Pose getRelativePose(geometry_msgs::msg::Pose from, std::string to);
  geometry_msgs::msg::Pose getRelativePose(
    geometry_msgs::msg::Pose from,
    geometry_msgs::msg::Pose to) const;
  const boost::optional<VehicleParameters> getVehicleParameters(std::string name) const;
  const std::vector<std::string> getEntityNames() const;
  const visualization_msgs::msg::MarkerArray generateMarker() const;
  bool setEntityStatus(std::string name, EntityStatus status);
  const CoordinateFrameTypes & getEntityStatusCoordinate(std::string name) const;
  const boost::optional<EntityStatus> getEntityStatus(
    std::string name,
    CoordinateFrameTypes coordinate =
    CoordinateFrameTypes::WORLD) const;
  bool entityStatusSetted(std::string name) const;
  void setTargetSpeed(std::string name, double target_speed, bool continuous);
  void update(double current_time, double step_time);
  void broadcastTransform(geometry_msgs::msg::PoseStamped pose);
  bool reachPosition(
    std::string name, geometry_msgs::msg::Pose target_pose,
    double tolerance) const;
  bool reachPosition(
    std::string name, int lanelet_id, double s, double offset,
    double tolerance) const;
  void broadcastEntityTransform();
  const boost::optional<double> getStandStillDuration(std::string name) const;
  const std::unordered_map<std::string, EntityType> getEntityTypeList() const;
  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lanelet_marker_pub_ptr_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr entity_marker_pub_ptr_;
  template<typename T>
  bool spawnEntity(T & entity)
  {
    if (entities_.count(entity.name) != 0) {
      throw simulation_controller::SimulationRuntimeError(
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

};
}  // namespace entity
}  // namespace simulation_controller

#endif   // SIMULATION_CONTROLLER__ENTITY__ENTITY_MANAGER_HPP_
