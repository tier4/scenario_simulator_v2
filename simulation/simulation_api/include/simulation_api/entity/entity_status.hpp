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

#ifndef SIMULATION_API__ENTITY__ENTITY_STATUS_HPP_
#define SIMULATION_API__ENTITY__ENTITY_STATUS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <openscenario_msgs/msg/entity_status.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>

namespace simulation_api
{
namespace entity
{
enum CoordinateFrameTypes
{
  WORLD = 0,
  LANE = 1,
};

class EntityStatus
{
public:
  EntityStatus() = default;
  EntityStatus(
    double t,
    geometry_msgs::msg::Pose pose,
    geometry_msgs::msg::Twist twist,
    geometry_msgs::msg::Accel accel);
  EntityStatus(
    double t,
    std::int64_t lanelet_id, double s, double offset,
    geometry_msgs::msg::Vector3 rpy,
    geometry_msgs::msg::Twist twist,
    geometry_msgs::msg::Accel accel);
  EntityStatus & operator=(const EntityStatus & obj)
  {
    this->time = obj.time;
    this->coordinate = obj.coordinate;
    this->twist = obj.twist;
    this->accel = obj.accel;
    this->lanelet_id = obj.lanelet_id;
    this->offset = obj.offset;
    this->s = obj.s;
    this->rpy = obj.rpy;
    this->pose = obj.pose;
    return *this;
  }
  double time;
  CoordinateFrameTypes coordinate;
  geometry_msgs::msg::Twist twist;
  geometry_msgs::msg::Accel accel;
  /* Field for Lene Pose */
  std::int64_t lanelet_id;
  double offset;
  double s;
  geometry_msgs::msg::Vector3 rpy;
  /* Field for world pose */
  geometry_msgs::msg::Pose pose;
  openscenario_msgs::msg::EntityStatus toRosMsg() const;
};
}  // namespace entity
}  // namespace simulation_api

#endif  // SIMULATION_API__ENTITY__ENTITY_STATUS_HPP_
