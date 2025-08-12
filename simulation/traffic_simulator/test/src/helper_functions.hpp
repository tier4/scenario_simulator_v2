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

#ifndef TRAFFIC_SIMULATOR__TEST__HELPER_FUNCTIONS_HPP_
#define TRAFFIC_SIMULATOR__TEST__HELPER_FUNCTIONS_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <traffic_simulator/utils/lanelet_map.hpp>
#include <traffic_simulator_msgs/msg/entity_subtype.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>

#include "catalogs.hpp"
#include "expect_eq_macros.hpp"

constexpr auto convertDegToRad(const double deg) -> double { return deg / 180.0 * M_PI; }
constexpr auto convertRadToDeg(const double rad) -> double { return rad * 180.0 / M_PI; }

auto makePoint(const double x, const double y, const double z = 0.0) -> geometry_msgs::msg::Point
{
  return geometry_msgs::build<geometry_msgs::msg::Point>().x(x).y(y).z(z);
}

auto makeBoundingBox(const double center_y = 0.0) -> traffic_simulator_msgs::msg::BoundingBox
{
  return traffic_simulator_msgs::build<traffic_simulator_msgs::msg::BoundingBox>()
    .center(makePoint(1.0, center_y))
    .dimensions(geometry_msgs::build<geometry_msgs::msg::Vector3>().x(4.0).y(2.0).z(1.5));
}

auto makeCustom2DBoundingBox(
  const double x, const double y, const double x_offset = 0.0, const double y_offset = 0.0)
  -> traffic_simulator_msgs::msg::BoundingBox
{
  return traffic_simulator_msgs::build<traffic_simulator_msgs::msg::BoundingBox>()
    .center(geometry_msgs::build<geometry_msgs::msg::Point>().x(x_offset).y(y_offset).z(0.0))
    .dimensions(geometry_msgs::build<geometry_msgs::msg::Vector3>().x(x).y(y).z(0.0));
}

auto makeSmallBoundingBox(const double center_y = 0.0) -> traffic_simulator_msgs::msg::BoundingBox
{
  return traffic_simulator_msgs::build<traffic_simulator_msgs::msg::BoundingBox>()
    .center(makePoint(0.0, center_y))
    .dimensions(geometry_msgs::build<geometry_msgs::msg::Vector3>().x(1.0).y(1.0).z(1.0));
}

auto makeQuaternionFromYaw(const double yaw) -> geometry_msgs::msg::Quaternion
{
  return math::geometry::convertEulerAngleToQuaternion(
    geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(yaw));
}

auto makePose(const double x, const double y, const double z, const double yaw_deg)
  -> geometry_msgs::msg::Pose
{
  /**
   * @note +x axis is  0 degrees; +y axis is 90 degrees
   */
  return geometry_msgs::build<geometry_msgs::msg::Pose>()
    .position(geometry_msgs::build<geometry_msgs::msg::Point>().x(x).y(y).z(z))
    .orientation(
      math::geometry::convertEulerAngleToQuaternion(
        geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(
          convertDegToRad(yaw_deg))));
}

auto makePose(
  geometry_msgs::msg::Point position,
  geometry_msgs::msg::Quaternion orientation = geometry_msgs::msg::Quaternion())
  -> geometry_msgs::msg::Pose
{
  return geometry_msgs::build<geometry_msgs::msg::Pose>().position(position).orientation(
    orientation);
}

auto makeHdMapUtilsSharedPointer() -> std::shared_ptr<hdmap_utils::HdMapUtils>
{
  return std::make_shared<hdmap_utils::HdMapUtils>(
    ament_index_cpp::get_package_share_directory("traffic_simulator") +
      "/map/standard_map/lanelet2_map.osm",
    geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
      .latitude(35.9037067912303)
      .longitude(139.9337945139059)
      .altitude(0.0));
}

auto activateLaneletWrapper(const std::string map_name) -> void
{
  const auto lanelet_path = ament_index_cpp::get_package_share_directory("traffic_simulator") +
                            "/map/" + map_name + "/lanelet2_map.osm";
  traffic_simulator::lanelet_map::activate(lanelet_path);
}

auto makeCanonicalizedLaneletPose(
  const lanelet::Id id = 120659, const double s = 0.0, const double offset = 0.0)
  -> traffic_simulator::lanelet_pose::CanonicalizedLaneletPose
{
  return traffic_simulator::lanelet_pose::CanonicalizedLaneletPose(
    traffic_simulator::helper::constructLaneletPose(id, s, offset));
}

auto makeEntityStatus(
  traffic_simulator::lanelet_pose::CanonicalizedLaneletPose pose,
  traffic_simulator_msgs::msg::BoundingBox bbox, const double speed = 0.0,
  const std::string name = "default_entity_name",
  const uint8_t type = traffic_simulator_msgs::msg::EntityType::VEHICLE)
  -> traffic_simulator::EntityStatus
{
  return traffic_simulator_msgs::build<traffic_simulator::EntityStatus>()
    .type(traffic_simulator_msgs::build<traffic_simulator_msgs::msg::EntityType>().type(type))
    .subtype(
      traffic_simulator_msgs::build<traffic_simulator_msgs::msg::EntitySubtype>().value(
        traffic_simulator_msgs::msg::EntitySubtype::UNKNOWN))
    .time(0.0)
    .name(name)
    .bounding_box(bbox)
    .action_status(traffic_simulator::helper::constructActionStatus(speed, 0.0, 0.0, 0.0))
    .pose(traffic_simulator::pose::toMapPose(pose))
    .lanelet_poses(
      std::vector<traffic_simulator_msgs::msg::LaneletPose>{
        static_cast<traffic_simulator::LaneletPose>(pose)});
}

auto makeEntityStatus(
  geometry_msgs::msg::Pose pose, traffic_simulator_msgs::msg::BoundingBox bbox,
  const double speed = 0.0, const std::string name = "default_entity_name",
  const uint8_t type = traffic_simulator_msgs::msg::EntityType::VEHICLE)
  -> traffic_simulator::EntityStatus
{
  return traffic_simulator_msgs::build<traffic_simulator::EntityStatus>()
    .type(traffic_simulator_msgs::build<traffic_simulator_msgs::msg::EntityType>().type(type))
    .subtype(
      traffic_simulator_msgs::build<traffic_simulator_msgs::msg::EntitySubtype>().value(
        traffic_simulator_msgs::msg::EntitySubtype::UNKNOWN))
    .time(0.0)
    .name(name)
    .bounding_box(bbox)
    .action_status(traffic_simulator::helper::constructActionStatus(speed, 0.0, 0.0, 0.0))
    .pose(pose)
    .lanelet_poses(std::vector<traffic_simulator_msgs::msg::LaneletPose>{});
}

auto makeCanonicalizedEntityStatus(
  traffic_simulator::lanelet_pose::CanonicalizedLaneletPose canonicalized_lanelet_pose,
  traffic_simulator_msgs::msg::BoundingBox bbox, const double speed = 0.0,
  const std::string name = "default_entity_name",
  const uint8_t type = traffic_simulator_msgs::msg::EntityType::VEHICLE)
  -> traffic_simulator::entity_status::CanonicalizedEntityStatus
{
  return traffic_simulator::entity_status::CanonicalizedEntityStatus(
    makeEntityStatus(canonicalized_lanelet_pose, bbox, speed, name, type),
    canonicalized_lanelet_pose);
}

auto makeCanonicalizedEntityStatus(
  geometry_msgs::msg::Pose pose, traffic_simulator_msgs::msg::BoundingBox bbox,
  const double matching_distance = 1.0, const double speed = 0.0,
  const std::string name = "default_entity_name",
  const uint8_t type = traffic_simulator_msgs::msg::EntityType::VEHICLE)
  -> traffic_simulator::entity_status::CanonicalizedEntityStatus
{
  const auto include_crosswalk =
    (traffic_simulator_msgs::msg::EntityType::PEDESTRIAN == type ||
     traffic_simulator_msgs::msg::EntityType::MISC_OBJECT == type);
  const auto canonicalized_lanelet_poses = traffic_simulator::pose::toCanonicalizedLaneletPoses(
    pose, bbox, include_crosswalk, matching_distance);
  return traffic_simulator::entity_status::CanonicalizedEntityStatus(
    // WIP just use the first canonicalized lanelet pose for now
    makeEntityStatus(pose, bbox, speed, name, type), canonicalized_lanelet_poses.front());
}

#endif  // TRAFFIC_SIMULATOR__TEST__ENTITY_HELPER_FUNCTIONS_HPP_
