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

#include <boost/geometry.hpp>
// #include <boost/geometry/geometries/point_xy.hpp>
// #include <boost/geometry/geometries/polygon.hpp>
// #include <geometry/polygon/polygon.hpp>
// #include <rclcpp/rclcpp.hpp>
#include <geometry/axis/axis.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace math
{
namespace geometry
{
auto axisToEigenAxis(Axis axis) -> Eigen::Vector3d
{
  switch (axis) {
    case Axis::X:
      return Eigen::Vector3d::UnitX();
    case Axis::Y:
      return Eigen::Vector3d::UnitY();
    case Axis::Z:
      return Eigen::Vector3d::UnitZ();
    default:
      THROW_SIMULATION_ERROR("Invalid axis specified.");
  }
}

auto getMaxValue(const std::vector<geometry_msgs::msg::Point> & points, const Axis & axis) -> double
{
  if (const auto values = filterByAxis(points, axis); values.size() == 1) {
    return values.front();
  } else {
    return *std::max_element(values.begin(), values.end());
  }
}

auto getMinValue(const std::vector<geometry_msgs::msg::Point> & points, const Axis & axis) -> double
{
  if (const auto values = filterByAxis(points, axis); values.size() == 1) {
    return values.front();
  } else {
    return *std::min_element(values.begin(), values.end());
  }
}

auto filterByAxis(const std::vector<geometry_msgs::msg::Point> & points, const Axis & axis)
  -> std::vector<double>
{
  if (points.empty()) {
    THROW_SIMULATION_ERROR(
      "Invalid point list is specified, while trying to filter ",
      axis == Axis::X   ? "X"
      : axis == Axis::Y ? "Y"
                        : "Z",
      " axis. ",
      "The point list in filterByAxis() should have at least one point to filter. "
      "This message is not originally intended to be displayed, if you see it, please "
      "contact the developer of traffic_simulator.");
  } else {
    const auto axisExtractor =
      [](const Axis axis) -> std::function<double(const geometry_msgs::msg::Point &)> {
      switch (axis) {
        case Axis::X:
          return [](const geometry_msgs::msg::Point & p) { return p.x; };
        case Axis::Y:
          return [](const geometry_msgs::msg::Point & p) { return p.y; };
        case Axis::Z:
          return [](const geometry_msgs::msg::Point & p) { return p.z; };
        default:
          throw std::invalid_argument("Invalid axis");
      }
    };

    std::vector<double> single_axis;
    std::transform(
      points.begin(), points.end(), std::back_inserter(single_axis), axisExtractor(axis));
    return single_axis;
  }
}
}  // namespace geometry
}  // namespace math
