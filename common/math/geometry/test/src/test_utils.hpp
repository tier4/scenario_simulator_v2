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

#ifndef GEOMETRY__TEST__TEST_UTILS_HPP_
#define GEOMETRY__TEST__TEST_UTILS_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <traffic_simulator_msgs/msg/bounding_box.hpp>

/**
 * Creates a `geometry_msgs::msg::Point` object with the specified coordinates.
 *
 * @param x The x-coordinate of the point.
 * @param y The y-coordinate of the point.
 * @param z The z-coordinate of the point. Defaults to 0 if not provided.
 *
 * @return The created `geometry_msgs::msg::Point` object with the specified coordinates.
 */
inline geometry_msgs::msg::Point makePoint(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

/**
 * Creates a `geometry_msgs::msg::Vector3` object with the specified values for `x`, `y`, and `z` coordinates.
 *
 * @param x The value for the `x` coordinate.
 * @param y The value for the `y` coordinate.
 * @param z The value for the `z` coordinate. Default value is 0.
 *
 * @return A `geometry_msgs::msg::Vector3` object with the specified values for `x`, `y`, and `z` coordinates.
 */
inline geometry_msgs::msg::Vector3 makeVector(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

/**
 * Generates a `Pose` message with the given coordinates and orientation.
 *
 * @param x The x-coordinate of the position.
 * @param y The y-coordinate of the position.
 * @param z The z-coordinate of the position (default: 0).
 * @param q The orientation of the pose (default: identity quaternion).
 *
 * @return A `Pose` message with the specified coordinates and orientation.
 */
inline geometry_msgs::msg::Pose makePose(
  double x, double y, double z = 0.0,
  geometry_msgs::msg::Quaternion q = geometry_msgs::msg::Quaternion())
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation = q;
  return p;
}

/**
 * Generates a bounding box with the given dimensions and center coordinates.
 *
 * @param dim_x The dimension of the bounding box along the x-axis.
 * @param dim_y The dimension of the bounding box along the y-axis.
 * @param dim_z The dimension of the bounding box along the z-axis. (default = 0.0)
 * @param center_x The x-coordinate of the center of the bounding box. (default = 0.0)
 * @param center_y The y-coordinate of the center of the bounding box. (default = 0.0)
 * @param center_z The z-coordinate of the center of the bounding box. (default = 0.0)
 *
 * @return The generated bounding box.
 */
inline traffic_simulator_msgs::msg::BoundingBox makeBbox(
  double dim_x, double dim_y, double dim_z = 0.0, double center_x = 0.0, double center_y = 0.0,
  double center_z = 0.0)
{
  traffic_simulator_msgs::msg::BoundingBox bbox;
  bbox.dimensions.x = dim_x;
  bbox.dimensions.y = dim_y;
  bbox.dimensions.z = dim_z;
  bbox.center.x = center_x;
  bbox.center.y = center_y;
  bbox.center.z = center_z;
  return bbox;
}

#endif  // GEOMETRY__TEST__TEST_UTILS_HPP_
