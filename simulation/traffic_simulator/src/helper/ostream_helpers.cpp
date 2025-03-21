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

#include <traffic_simulator/helper/ostream_helpers.hpp>

namespace traffic_simulator
{
// basic ros types
std::ostream & operator<<(std::ostream & os, const geometry_msgs::msg::Point & point)
{
  os << "Point(x: " << point.x << ", y: " << point.y << ", z: " << point.z << ")";
  return os;
}

std::ostream & operator<<(std::ostream & os, const geometry_msgs::msg::Vector3 & vector)
{
  os << "Vector3(x: " << vector.x << ", y: " << vector.y << ", z: " << vector.z << ")";
  return os;
}

std::ostream & operator<<(std::ostream & os, const geometry_msgs::msg::Quaternion & quat)
{
  os << "Quaternion(x: " << quat.x << ", y: " << quat.y << ", z: " << quat.z << ", w: " << quat.w
     << ")";
  return os;
}

std::ostream & operator<<(std::ostream & os, const geometry_msgs::msg::Pose & pose)
{
  os << "Pose(\n";
  os << "  position: " << pose.position << "\n";
  os << "  orientation: " << pose.orientation << "\n";
  os << ")";
  return os;
}

// traffic_simulator_msgs
std::ostream & operator<<(
  std::ostream & os, const traffic_simulator_msgs::msg::LaneletPose & lanelet_pose)
{
  os << "LaneletPose(lanelet_id: " << lanelet_pose.lanelet_id << ", s: " << lanelet_pose.s
     << ", offset: " << lanelet_pose.offset << ", rpy: " << lanelet_pose.rpy << ")";
  return os;
}

std::ostream & operator<<(
  std::ostream & os, const traffic_simulator_msgs::msg::EntitySubtype & subtype)
{
  using EntitySubtype = traffic_simulator_msgs::msg::EntitySubtype;
  static const std::unordered_map<uint8_t, std::string> entity_names = {
    {EntitySubtype::UNKNOWN, "UNKNOWN"}, {EntitySubtype::CAR, "CAR"},
    {EntitySubtype::TRUCK, "TRUCK"},     {EntitySubtype::BUS, "BUS"},
    {EntitySubtype::TRAILER, "TRAILER"}, {EntitySubtype::MOTORCYCLE, "MOTORCYCLE"},
    {EntitySubtype::BICYCLE, "BICYCLE"}, {EntitySubtype::PEDESTRIAN, "PEDESTRIAN"}};

  os << "EntitySubtype(";
  if (const auto & it = entity_names.find(subtype.value); it != entity_names.end()) {
    os << it->second;
  } else {
    os << "UNKNOWN(" << static_cast<int>(subtype.value) << ")";
  }
  os << ")";
  return os;
}

std::ostream & operator<<(std::ostream & os, const traffic_simulator_msgs::msg::Axle & axle)
{
  os << "Axle(\n";
  os << "  max_steering: " << axle.max_steering << "\n";
  os << "  wheel_diameter: " << axle.wheel_diameter << "\n";
  os << "  track_width: " << axle.track_width << "\n";
  os << "  position_x: " << axle.position_x << "\n";
  os << "  position_z: " << axle.position_z << "\n";
  os << ")";
  return os;
}

std::ostream & operator<<(std::ostream & os, const traffic_simulator_msgs::msg::Axles & axles)
{
  os << "Axles(\n";
  os << "  front_axle: " << axles.front_axle << "\n";
  os << "  rear_axle: " << axles.rear_axle << "\n";
  os << ")";
  return os;
}

std::ostream & operator<<(
  std::ostream & os, const traffic_simulator_msgs::msg::BoundingBox & bounding_box)
{
  os << "BoundingBox(\n";
  os << "  center: " << bounding_box.center << "\n";
  os << "  dimensions: " << bounding_box.dimensions << "\n";
  os << ")";
  return os;
}

std::ostream & operator<<(
  std::ostream & os, const traffic_simulator_msgs::msg::Performance & performance)
{
  os << "Performance(\n";
  os << "  max_acceleration: " << performance.max_acceleration << "\n";
  os << "  max_acceleration_rate: " << performance.max_acceleration_rate << "\n";
  os << "  max_deceleration: " << performance.max_deceleration << "\n";
  os << "  max_deceleration_rate: " << performance.max_deceleration_rate << "\n";
  os << "  max_speed: " << performance.max_speed << "\n";
  os << ")";
  return os;
}

std::ostream & operator<<(
  std::ostream & os, const traffic_simulator_msgs::msg::VehicleParameters & params)
{
  os << "VehicleParameters(\n";
  os << "  name: " << params.name << "\n";
  os << "  subtype: " << params.subtype << "\n";
  os << "  bounding_box: " << params.bounding_box << "\n";
  os << "  performance: " << params.performance << "\n";
  os << "  axles: " << params.axles << "\n";
  os << ")";
  return os;
}
}  // namespace traffic_simulator
