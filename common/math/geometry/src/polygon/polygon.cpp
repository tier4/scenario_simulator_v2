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
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <geometry/polygon/polygon.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>

namespace math
{
namespace geometry
{
std::vector<geometry_msgs::msg::Point> get2DConvexHull(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  typedef boost::geometry::model::d2::point_xy<double> boost_point;
  typedef boost::geometry::model::polygon<boost_point> boost_polygon;
  boost_polygon poly;
  for (const auto & p : points) {
    boost::geometry::exterior_ring(poly).push_back(boost_point(p.x, p.y));
  }
  boost_polygon hull;
  boost::geometry::convex_hull(poly, hull);
  std::vector<geometry_msgs::msg::Point> polygon;
  for (auto it = boost::begin(boost::geometry::exterior_ring(hull));
       it != boost::end(boost::geometry::exterior_ring(hull)); ++it) {
    double x = boost::geometry::get<0>(*it);
    double y = boost::geometry::get<1>(*it);
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    polygon.emplace_back(p);
  }
  return polygon;
}

double getMaxValue(const std::vector<geometry_msgs::msg::Point> & points, const Axis & axis)
{
  if (points.empty()) {
    THROW_SIMULATION_ERROR(
      "Invalid point list is specified, while getting max value on ",
      axis == Axis::X   ? "X"
      : axis == Axis::Y ? "Y"
                        : "Z",
      " axis. ",
      "The point list in getMaxValue should have at least one point to get the max value from. "
      "This message is not originally intended to be displayed, if you see it, please "
      "contact the developer of traffic_simulator.");
  }
  const auto values = filterByAxis(points, axis);
  if (values.size() == 1) {
    return values.front();
  }
  return *std::max_element(values.begin(), values.end());
}

double getMinValue(const std::vector<geometry_msgs::msg::Point> & points, const Axis & axis)
{
  if (points.empty()) {
    THROW_SIMULATION_ERROR(
      "Invalid point list is specified, while getting min value on ",
      axis == Axis::X   ? "X"
      : axis == Axis::Y ? "Y"
                        : "Z",
      " axis. ",
      "The point list in getMinValue should have at least one point to get the min value from. "
      "This message is not originally intended to be displayed, if you see it, please "
      "contact the developer of traffic_simulator.");
  }
  const auto values = filterByAxis(points, axis);
  if (values.size() == 1) {
    return values.front();
  }
  return *std::min_element(values.begin(), values.end());
}

std::vector<double> filterByAxis(
  const std::vector<geometry_msgs::msg::Point> & points, const Axis & axis)
{
  std::vector<double> ret;
  switch (axis) {
    case Axis::X: {
      std::transform(
        points.begin(), points.end(), std::back_inserter(ret),
        [](const geometry_msgs::msg::Point & point) { return point.x; });
      break;
    }
    case Axis::Y: {
      std::transform(
        points.begin(), points.end(), std::back_inserter(ret),
        [](const geometry_msgs::msg::Point & point) { return point.y; });
      break;
    }
    case Axis::Z: {
      std::transform(
        points.begin(), points.end(), std::back_inserter(ret),
        [](const geometry_msgs::msg::Point & point) { return point.z; });
      break;
    }
  }
  return ret;
}
}  // namespace geometry
}  // namespace math
