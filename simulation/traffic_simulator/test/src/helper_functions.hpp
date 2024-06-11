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

#ifndef TRAFFIC_SIMULATOR__TEST__ENTITY_HELPER_FUNCTIONS_HPP_
#define TRAFFIC_SIMULATOR__TEST__ENTITY_HELPER_FUNCTIONS_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/entity/entity_base.hpp>
#include <traffic_simulator/helper/helper.hpp>

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
  return quaternion_operation::convertEulerAngleToQuaternion(
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
  std::string path =
    ament_index_cpp::get_package_share_directory("traffic_simulator") + "/map/lanelet2_map.osm";
  geographic_msgs::msg::GeoPoint origin;
  origin.latitude = 35.9037067912303;
  origin.longitude = 139.9337945139059;
  return std::make_shared<hdmap_utils::HdMapUtils>(path, origin);
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
  const std::string name = "dummy_entity",
  const uint8_t type = traffic_simulator_msgs::msg::EntityType::VEHICLE)
  -> traffic_simulator::EntityStatus
{
  traffic_simulator::EntityStatus entity_status;
  entity_status.type.type = type;
  entity_status.subtype.value = traffic_simulator_msgs::msg::EntitySubtype::CAR;
  entity_status.time = 0.0;
  entity_status.name = name;
  entity_status.bounding_box = bbox;
  geometry_msgs::msg::Twist twist;
  entity_status.action_status =
    traffic_simulator::helper::constructActionStatus(speed, 0.0, 0.0, 0.0);
  entity_status.lanelet_pose_valid = true;
  entity_status.lanelet_pose = static_cast<traffic_simulator::LaneletPose>(pose);
  entity_status.pose = hdmap_utils->toMapPose(entity_status.lanelet_pose).pose;
  return entity_status;
}

auto makeEntityStatus(
  std::shared_ptr<hdmap_utils::HdMapUtils> /* hdmap_utils */, geometry_msgs::msg::Pose pose,
  traffic_simulator_msgs::msg::BoundingBox bbox, const double speed = 0.0,
  const std::string name = "dummy_entity",
  const uint8_t type = traffic_simulator_msgs::msg::EntityType::VEHICLE)
  -> traffic_simulator::EntityStatus
{
  traffic_simulator::EntityStatus entity_status;
  entity_status.type.type = type;
  entity_status.subtype.value = traffic_simulator_msgs::msg::EntitySubtype::CAR;
  entity_status.time = 0.0;
  entity_status.name = name;
  entity_status.bounding_box = bbox;
  geometry_msgs::msg::Twist twist;
  entity_status.action_status =
    traffic_simulator::helper::constructActionStatus(speed, 0.0, 0.0, 0.0);
  entity_status.lanelet_pose_valid = false;
  entity_status.pose = pose;
  return entity_status;
}

auto makeCanonicalizedEntityStatus(
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils,
  traffic_simulator::lanelet_pose::CanonicalizedLaneletPose pose,
  traffic_simulator_msgs::msg::BoundingBox bbox, const double speed = 0.0,
  const std::string name = "dummy_entity",
  const uint8_t type = traffic_simulator_msgs::msg::EntityType::VEHICLE)
  -> traffic_simulator::entity_status::CanonicalizedEntityStatus
{
  return traffic_simulator::entity_status::CanonicalizedEntityStatus(
    makeEntityStatus(hdmap_utils, pose, bbox, speed, name, type), hdmap_utils);
}

auto makeCanonicalizedEntityStatus(
  std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils, geometry_msgs::msg::Pose pose,
  traffic_simulator_msgs::msg::BoundingBox bbox, const double speed = 0.0,
  const std::string name = "dummy_entity",
  const uint8_t type = traffic_simulator_msgs::msg::EntityType::VEHICLE)
  -> traffic_simulator::entity_status::CanonicalizedEntityStatus
{
  return traffic_simulator::entity_status::CanonicalizedEntityStatus(
    makeEntityStatus(hdmap_utils, pose, bbox, speed, name, type), hdmap_utils);
}

#endif  // TRAFFIC_SIMULATOR__TEST__ENTITY_HELPER_FUNCTIONS_HPP_