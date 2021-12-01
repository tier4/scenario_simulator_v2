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

#ifndef TRAFFIC_SIMULATOR__HELPER__HELPER_HPP_
#define TRAFFIC_SIMULATOR__HELPER__HELPER_HPP_

#include <simulation_api_schema.pb.h>

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <iostream>
#include <string>
#include <traffic_simulator_msgs/msg/action_status.hpp>
#include <traffic_simulator_msgs/msg/lanelet_pose.hpp>

namespace traffic_simulator
{
namespace helper
{
/**
 * @brief helper function for constructing action status
 *
 * @param linear_vel linear velocity
 * @param angular_vel angular velocity
 * @param linear_accel linear acceleration
 * @param angular_accel angular acceleration
 * @return traffic_simulator_msgs::msg::ActionStatus
 */
traffic_simulator_msgs::msg::ActionStatus constructActionStatus(
  double linear_vel = 0, double angular_vel = 0, double linear_accel = 0, double angular_accel = 0);

/**
 * @brief helper function for constructing lanelet pose
 *
 * @param lanelet_id lanelet id
 * @param s s value in lane coordinate
 * @param offset offset value in lane coordinate
 * @param roll roll value in the lane coordinate
 * @param pitch pitch value in the lane coordinate
 * @param yaw yaw value in the lane coordinate
 * @return traffic_simulator_msgs::msg::LaneletPose
 */
traffic_simulator_msgs::msg::LaneletPose constructLaneletPose(
  std::int64_t lanelet_id, double s, double offset = 0, double roll = 0, double pitch = 0,
  double yaw = 0);

/**
 * @brief helper function for constructing rpy
 *
 * @param roll roll value of the orientation
 * @param pitch pitch value of the orientation
 * @param yaw yaw value of the orientation
 * @return geometry_msgs::msg::Vector3 RPY values
 */
geometry_msgs::msg::Vector3 constructRPY(double roll = 0, double pitch = 0, double yaw = 0);

/**
 * @brief helper function for constructing rpy
 *
 * @param quaternion quaternion class
 * @return geometry_msgs::msg::Vector3 RPY value
 */
geometry_msgs::msg::Vector3 constructRPYfromQuaternion(geometry_msgs::msg::Quaternion quaternion);

/**
 * @brief helper function for constructing pose
 *
 * @param x x value in position
 * @param y y value in position
 * @param z z value in position
 * @param roll roll value in orientation
 * @param pitch pitch value in orientation
 * @param yaw yaw value in orientation
 * @return geometry_msgs::msg::Pose
 */
geometry_msgs::msg::Pose constructPose(
  double x, double y, double z, double roll, double pitch, double yaw);

enum class LidarType { VLP16, VLP32 };

const simulation_api_schema::LidarConfiguration constructLidarConfiguration(
  const LidarType type, const std::string & entity, const std::string & architecture_type,
  const double horizontal_resolution = 1.0 / 180.0 * M_PI);

const simulation_api_schema::DetectionSensorConfiguration constructDetectionSensorConfiguration(
  const std::string & entity, const std::string & architecture_type, const double update_duration);
}  // namespace helper
}  // namespace traffic_simulator

std::ostream & operator<<(
  std::ostream & os, const traffic_simulator_msgs::msg::LaneletPose & ll_pose);

#endif  // TRAFFIC_SIMULATOR__HELPER__HELPER_HPP_
