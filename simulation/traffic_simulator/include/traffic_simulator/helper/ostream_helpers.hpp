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

#ifndef TRAFFIC_SIMULATOR_HELPER__OSTREAM_HELPERS_HPP_
#define TRAFFIC_SIMULATOR_HELPER__OSTREAM_HELPERS_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <traffic_simulator_msgs/msg/axle.hpp>
#include <traffic_simulator_msgs/msg/axles.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>
#include <traffic_simulator_msgs/msg/entity_subtype.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>
#include <traffic_simulator_msgs/msg/performance.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>

namespace traffic_simulator
{
std::ostream & operator<<(std::ostream & os, const geometry_msgs::msg::Point & point);

std::ostream & operator<<(std::ostream & os, const geometry_msgs::msg::Vector3 & vector);

std::ostream & operator<<(std::ostream & os, const geometry_msgs::msg::Quaternion & quat);

std::ostream & operator<<(std::ostream & os, const geometry_msgs::msg::Pose & pose);

std::ostream & operator<<(
  std::ostream & os, const traffic_simulator_msgs::msg::LaneletPose & ll_pose);

std::ostream & operator<<(
  std::ostream & os, const traffic_simulator_msgs::msg::EntitySubtype & subtype);

std::ostream & operator<<(std::ostream & os, const traffic_simulator_msgs::msg::Axle & axle);

std::ostream & operator<<(std::ostream & os, const traffic_simulator_msgs::msg::Axles & axles);

std::ostream & operator<<(
  std::ostream & os, const traffic_simulator_msgs::msg::BoundingBox & bounding_box);

std::ostream & operator<<(
  std::ostream & os, const traffic_simulator_msgs::msg::Performance & performance);

std::ostream & operator<<(
  std::ostream & os, const traffic_simulator_msgs::msg::VehicleParameters & params);
}  // namespace traffic_simulator

#endif  // TRAFFIC_SIMULATOR_HELPER__OSTREAM_HELPERS_HPP_
