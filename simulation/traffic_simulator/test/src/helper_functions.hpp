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
#include <traffic_simulator_msgs/msg/entity_subtype.hpp>
#include <traffic_simulator_msgs/msg/entity_type.hpp>

#include "catalogs.hpp"
#include "expect_eq_macros.hpp"

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
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm",
    geographic_msgs::build<geographic_msgs::msg::GeoPoint>()
      .latitude(35.9037067912303)
      .longitude(139.9337945139059)
      .altitude(0.0));
}

auto makeCanonicalizedLaneletPose(
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils, const lanelet::Id id = 120659,
  const double s = 0.0, const double offset = 0.0)
  -> traffic_simulator::lanelet_pose::CanonicalizedLaneletPose
{
  return traffic_simulator::lanelet_pose::CanonicalizedLaneletPose(
    traffic_simulator::helper::constructLaneletPose(id, s, offset), hdmap_utils);
}

auto makeEntityStatus(
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils,
  traffic_simulator::lanelet_pose::CanonicalizedLaneletPose pose,
  traffic_simulator_msgs::msg::BoundingBox bbox, const double speed = 0.0,
  const std::string name = "default_entity_name",
  const uint8_t type = traffic_simulator_msgs::msg::EntityType::VEHICLE)
  -> traffic_simulator::EntityStatus
{
  return traffic_simulator_msgs::build<traffic_simulator::EntityStatus>()
    .type(traffic_simulator_msgs::build<traffic_simulator_msgs::msg::EntityType>().type(type))
    .subtype(traffic_simulator_msgs::build<traffic_simulator_msgs::msg::EntitySubtype>().value(
      traffic_simulator_msgs::msg::EntitySubtype::UNKNOWN))
    .time(0.0)
    .name(name)
    .bounding_box(bbox)
    .action_status(traffic_simulator::helper::constructActionStatus(speed, 0.0, 0.0, 0.0))
    .pose(hdmap_utils->toMapPose(static_cast<traffic_simulator::LaneletPose>(pose)).pose)
    .lanelet_pose(static_cast<traffic_simulator::LaneletPose>(pose))
    .lanelet_pose_valid(true);
}

auto makeEntityStatus(
  std::shared_ptr<hdmap_utils::HdMapUtils> /* hdmap_utils */, geometry_msgs::msg::Pose pose,
  traffic_simulator_msgs::msg::BoundingBox bbox, const double speed = 0.0,
  const std::string name = "default_entity_name",
  const uint8_t type = traffic_simulator_msgs::msg::EntityType::VEHICLE)
  -> traffic_simulator::EntityStatus
{
  return traffic_simulator_msgs::build<traffic_simulator::EntityStatus>()
    .type(traffic_simulator_msgs::build<traffic_simulator_msgs::msg::EntityType>().type(type))
    .subtype(traffic_simulator_msgs::build<traffic_simulator_msgs::msg::EntitySubtype>().value(
      traffic_simulator_msgs::msg::EntitySubtype::UNKNOWN))
    .time(0.0)
    .name(name)
    .bounding_box(bbox)
    .action_status(traffic_simulator::helper::constructActionStatus(speed, 0.0, 0.0, 0.0))
    .pose(pose)
    .lanelet_pose(traffic_simulator_msgs::msg::LaneletPose{})
    .lanelet_pose_valid(false);
}

auto makeCanonicalizedEntityStatus(
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils,
  traffic_simulator::lanelet_pose::CanonicalizedLaneletPose canonicalized_lanelet_pose,
  traffic_simulator_msgs::msg::BoundingBox bbox, const double speed = 0.0,
  const std::string name = "default_entity_name",
  const uint8_t type = traffic_simulator_msgs::msg::EntityType::VEHICLE)
  -> traffic_simulator::entity_status::CanonicalizedEntityStatus
{
  return traffic_simulator::entity_status::CanonicalizedEntityStatus(
    makeEntityStatus(hdmap_utils, canonicalized_lanelet_pose, bbox, speed, name, type),
    canonicalized_lanelet_pose);
}

auto makeCanonicalizedEntityStatus(
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils, geometry_msgs::msg::Pose pose,
  traffic_simulator_msgs::msg::BoundingBox bbox, const double matching_distance = 1.0,
  const double speed = 0.0, const std::string name = "default_entity_name",
  const uint8_t type = traffic_simulator_msgs::msg::EntityType::VEHICLE)
  -> traffic_simulator::entity_status::CanonicalizedEntityStatus
{
  const auto include_crosswalk =
    (traffic_simulator_msgs::msg::EntityType::PEDESTRIAN == type ||
     traffic_simulator_msgs::msg::EntityType::MISC_OBJECT == type);
  const auto canonicalized_lanelet_pose = traffic_simulator::pose::toCanonicalizedLaneletPose(
    pose, bbox, include_crosswalk, matching_distance, hdmap_utils);
  return traffic_simulator::entity_status::CanonicalizedEntityStatus(
    makeEntityStatus(hdmap_utils, pose, bbox, speed, name, type), canonicalized_lanelet_pose);
}

#endif  // TRAFFIC_SIMULATOR__TEST__ENTITY_HELPER_FUNCTIONS_HPP_
