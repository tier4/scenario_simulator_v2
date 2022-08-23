/**
 * @file distance.hpp
 * @author Masaya Kataoka (masaya.kataoka@tier4.jp)
 * @brief functions for calculating distance
 * @version 0.1
 * @date 2021-04-01
 *
 * @copyright Copyright(c) TIER IV.Inc {2015}
 *
 */

// Copyright 2015 TIER IV.inc. All rights reserved.
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

#ifndef GEOMETRY__DISTANCE_HPP_
#define GEOMETRY__DISTANCE_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <vector>

namespace math
{
namespace geometry
{
double getDistance(const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1);
double getDistance(const geometry_msgs::msg::Pose & p0, const geometry_msgs::msg::Point & p1);
double getDistance(const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Pose & p1);
double getDistance(const geometry_msgs::msg::Pose & p0, const geometry_msgs::msg::Pose & p1);
double getDistance2D(
  const std::vector<geometry_msgs::msg::Point> & polygon0,
  const std::vector<geometry_msgs::msg::Point> & polygon1);
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__DISTANCE_HPP_
