
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

#ifndef GEOMETRY__AXIS__AXIS_HPP_
#define GEOMETRY__AXIS__AXIS_HPP_

#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>

namespace math
{
namespace geometry
{
enum class Axis { X = 0, Y = 1, Z = 2 };

auto axisToEigenAxis(Axis axis) -> Eigen::Vector3d;

auto getMaxValue(const std::vector<geometry_msgs::msg::Point> & points, const Axis & axis)
  -> double;

auto getMinValue(const std::vector<geometry_msgs::msg::Point> & points, const Axis & axis)
  -> double;

auto filterByAxis(const std::vector<geometry_msgs::msg::Point> & points, const Axis & axis)
  -> std::vector<double>;
}  // namespace geometry
}  // namespace math

#endif  // GEOMETRY__AXIS__AXIS_HPP_
