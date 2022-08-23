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

#ifndef GEOMETRY__POLYGON__POLYGON_HPP_
#define GEOMETRY__POLYGON__POLYGON_HPP_

#include <geometry_msgs/msg/point.hpp>

namespace math
{
namespace geometry
{
enum class Axis { X = 0, Y = 1, Z = 2 };
double getMaxValue(const std::vector<geometry_msgs::msg::Point> & points, const Axis & axis);
double getMinValue(const std::vector<geometry_msgs::msg::Point> & points, const Axis & axis);
std::vector<double> filterByAxis(
  const std::vector<geometry_msgs::msg::Point> & points, const Axis & axis);
std::vector<geometry_msgs::msg::Point> get2DConvexHull(
  const std::vector<geometry_msgs::msg::Point> & points);
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__POLYGON__POLYGON_HPP_
