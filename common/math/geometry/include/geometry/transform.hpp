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

#ifndef GEOMETRY__TRANSFORM_HPP_
#define GEOMETRY__TRANSFORM_HPP_

#include <geometry_msgs/msg/pose.hpp>

namespace math
{
namespace geometry
{
/**
 * @brief Get the relative pose between two poses.
 * @param from origin pose
 * @param to target pose
 * @return const geometry_msgs::msg::Pose relative pose from origin to target
 */
const geometry_msgs::msg::Pose getRelativePose(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to);
/**
 * @brief Get transformed pose in world frame.
 * @param pose pose world frame
 * @param point point in local frame
 * @return const geometry_msgs::msg::Point transformed point
 */
const geometry_msgs::msg::Point transformPoint(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Point & point);
/**
 * @brief Get transformed pose in sensor frame.
 * @param pose pose in world frame
 * @param sensor_pose sensor pose in world frame
 * @param point point in local frame
 * @return const geometry_msgs::msg::Point transformed point
 */
const geometry_msgs::msg::Point transformPoint(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Pose & sensor_pose,
  const geometry_msgs::msg::Point & point);
/**
 * @brief Get transformed points in world frame.
 * @param pose pose in world frame
 * @param points points in local frame
 * @return std::vector<geometry_msgs::msg::Point> transformed points
 */
std::vector<geometry_msgs::msg::Point> transformPoints(
  const geometry_msgs::msg::Pose & pose, const std::vector<geometry_msgs::msg::Point> & points);
/**
 * @brief Get transformed points in sensor frame.
 * @param pose pose in world frame
 * @param points points in local frame
 * @return std::vector<geometry_msgs::msg::Point> transformed points
 */
std::vector<geometry_msgs::msg::Point> transformPoints(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Pose & sensor_pose,
  const std::vector<geometry_msgs::msg::Point> & points);
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__TRANSFORM_HPP_
