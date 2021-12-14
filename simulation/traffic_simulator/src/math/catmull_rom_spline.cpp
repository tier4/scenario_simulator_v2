// Copyright 2015-2020 TierIV.inc. All rights reserved.
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

#include <iostream>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <traffic_simulator/math/catmull_rom_spline.hpp>
#include <utility>
#include <vector>

namespace traffic_simulator
{
namespace math
{
const std::vector<geometry_msgs::msg::Point> CatmullRomSpline::getPolygon(
  double width, size_t num_points, double z_offset)
{
  std::vector<geometry_msgs::msg::Point> points;
  std::vector<geometry_msgs::msg::Point> left_bounds = getLeftBounds(width, num_points, z_offset);
  std::vector<geometry_msgs::msg::Point> right_bounds = getRightBounds(width, num_points, z_offset);
  size_t num_sections = static_cast<size_t>(left_bounds.size() - 1);
  for (size_t i = 0; i < num_sections; i++) {
    geometry_msgs::msg::Point pr_0 = right_bounds[i];
    geometry_msgs::msg::Point pl_0 = left_bounds[i];
    geometry_msgs::msg::Point pr_1 = right_bounds[i + 1];
    geometry_msgs::msg::Point pl_1 = left_bounds[i + 1];
    points.emplace_back(pr_0);
    points.emplace_back(pl_0);
    points.emplace_back(pr_1);
    points.emplace_back(pl_0);
    points.emplace_back(pl_1);
    points.emplace_back(pr_1);
  }
  return points;
}

const geometry_msgs::msg::Point CatmullRomSpline::getRightBoundsPoint(
  double width, double s, double z_offset) const
{
  geometry_msgs::msg::Vector3 vec = getNormalVector(s);
  double theta = std::atan2(vec.y, vec.x);
  geometry_msgs::msg::Point p = getPoint(s);
  geometry_msgs::msg::Point point;
  point.x = p.x + 0.5 * width * std::cos(theta);
  point.y = p.y + 0.5 * width * std::sin(theta);
  point.z = p.z + z_offset;
  return point;
}

const geometry_msgs::msg::Point CatmullRomSpline::getLeftBoundsPoint(
  double width, double s, double z_offset) const
{
  geometry_msgs::msg::Vector3 vec = getNormalVector(s);
  double theta = std::atan2(vec.y, vec.x);
  geometry_msgs::msg::Point p = getPoint(s);
  geometry_msgs::msg::Point point;
  point.x = p.x - 0.5 * width * std::cos(theta);
  point.y = p.y - 0.5 * width * std::sin(theta);
  point.z = p.z + z_offset;
  return point;
}

const std::vector<geometry_msgs::msg::Point> CatmullRomSpline::getRightBounds(
  double width, size_t num_points, double z_offset) const
{
  std::vector<geometry_msgs::msg::Point> points;
  double step_size = getLength() / static_cast<double>(num_points);
  for (size_t i = 0; i < static_cast<size_t>(num_points + 1); i++) {
    double s = step_size * static_cast<double>(i);
    points.emplace_back(getRightBoundsPoint(width, s, z_offset));
  }
  return points;
}

const std::vector<geometry_msgs::msg::Point> CatmullRomSpline::getLeftBounds(
  double width, size_t num_points, double z_offset) const
{
  std::vector<geometry_msgs::msg::Point> points;
  double step_size = getLength() / static_cast<double>(num_points);
  for (size_t i = 0; i < static_cast<size_t>(num_points + 1); i++) {
    double s = step_size * static_cast<double>(i);
    points.emplace_back(getLeftBoundsPoint(width, s, z_offset));
  }
  return points;
}

const std::vector<geometry_msgs::msg::Point> CatmullRomSpline::getTrajectory(
  double start_s, double end_s, double resolution) const
{
  if (start_s > end_s) {
    std::vector<geometry_msgs::msg::Point> ret;
    resolution = std::fabs(resolution);
    double s = start_s;
    while (s >= end_s) {
      auto p = getPoint(s);
      ret.emplace_back(p);
      s = s - resolution;
    }
    return ret;
  } else {
    std::vector<geometry_msgs::msg::Point> ret;
    resolution = std::fabs(resolution);
    double s = start_s;
    while (s <= end_s) {
      auto p = getPoint(s);
      ret.emplace_back(p);
      s = s + resolution;
    }
    return ret;
  }
}

CatmullRomSpline::CatmullRomSpline(const std::vector<geometry_msgs::msg::Point> & control_points)
: control_points(control_points)
{
  size_t n = control_points.size() - 1;
  if (control_points.size() <= 2) {
    THROW_SEMANTIC_ERROR(
      control_points.size(),
      " control points are only exists. At minimum, 2 control points are required");
  }
  for (size_t i = 0; i < n; i++) {
    if (i == 0) {
      double ax = 0;
      double bx = control_points[0].x - 2 * control_points[1].x + control_points[2].x;
      double cx = -3 * control_points[0].x + 4 * control_points[1].x - control_points[2].x;
      double dx = 2 * control_points[0].x;
      double ay = 0;
      double by = control_points[0].y - 2 * control_points[1].y + control_points[2].y;
      double cy = -3 * control_points[0].y + 4 * control_points[1].y - control_points[2].y;
      double dy = 2 * control_points[0].y;
      double az = 0;
      double bz = control_points[0].z - 2 * control_points[1].z + control_points[2].z;
      double cz = -3 * control_points[0].z + 4 * control_points[1].z - control_points[2].z;
      double dz = 2 * control_points[0].z;
      ax = ax * 0.5;
      bx = bx * 0.5;
      cx = cx * 0.5;
      dx = dx * 0.5;
      ay = ay * 0.5;
      by = by * 0.5;
      cy = cy * 0.5;
      dy = dy * 0.5;
      az = az * 0.5;
      bz = bz * 0.5;
      cz = cz * 0.5;
      dz = dz * 0.5;
      curves_.emplace_back(HermiteCurve(ax, bx, cx, dx, ay, by, cy, dy, az, bz, cz, dz));
    } else if (i == (n - 1)) {
      double ax = 0;
      double bx = control_points[i - 1].x - 2 * control_points[i].x + control_points[i + 1].x;
      double cx = -1 * control_points[i - 1].x + control_points[i + 1].x;
      double dx = 2 * control_points[i].x;
      double ay = 0;
      double by = control_points[i - 1].y - 2 * control_points[i].y + control_points[i + 1].y;
      double cy = -1 * control_points[i - 1].y + control_points[i + 1].y;
      double dy = 2 * control_points[i].y;
      double az = 0;
      double bz = control_points[i - 1].z - 2 * control_points[i].z + control_points[i + 1].z;
      double cz = -1 * control_points[i - 1].z + control_points[i + 1].z;
      double dz = 2 * control_points[i].z;
      ax = ax * 0.5;
      bx = bx * 0.5;
      cx = cx * 0.5;
      dx = dx * 0.5;
      ay = ay * 0.5;
      by = by * 0.5;
      cy = cy * 0.5;
      dy = dy * 0.5;
      az = az * 0.5;
      bz = bz * 0.5;
      cz = cz * 0.5;
      dz = dz * 0.5;
      curves_.emplace_back(HermiteCurve(ax, bx, cx, dx, ay, by, cy, dy, az, bz, cz, dz));
    } else {
      double ax = -1 * control_points[i - 1].x + 3 * control_points[i].x -
                  3 * control_points[i + 1].x + control_points[i + 2].x;
      double bx = 2 * control_points[i - 1].x - 5 * control_points[i].x +
                  4 * control_points[i + 1].x - control_points[i + 2].x;
      double cx = -control_points[i - 1].x + control_points[i + 1].x;
      double dx = 2 * control_points[i].x;
      double ay = -1 * control_points[i - 1].y + 3 * control_points[i].y -
                  3 * control_points[i + 1].y + control_points[i + 2].y;
      double by = 2 * control_points[i - 1].y - 5 * control_points[i].y +
                  4 * control_points[i + 1].y - control_points[i + 2].y;
      double cy = -control_points[i - 1].y + control_points[i + 1].y;
      double dy = 2 * control_points[i].y;
      double az = -1 * control_points[i - 1].z + 3 * control_points[i].z -
                  3 * control_points[i + 1].z + control_points[i + 2].z;
      double bz = 2 * control_points[i - 1].z - 5 * control_points[i].z +
                  4 * control_points[i + 1].z - control_points[i + 2].z;
      double cz = -control_points[i - 1].z + control_points[i + 1].z;
      double dz = 2 * control_points[i].z;
      ax = ax * 0.5;
      bx = bx * 0.5;
      cx = cx * 0.5;
      dx = dx * 0.5;
      ay = ay * 0.5;
      by = by * 0.5;
      cy = cy * 0.5;
      dy = dy * 0.5;
      az = az * 0.5;
      bz = bz * 0.5;
      cz = cz * 0.5;
      dz = dz * 0.5;
      curves_.emplace_back(HermiteCurve(ax, bx, cx, dx, ay, by, cy, dy, az, bz, cz, dz));
    }
  }
  for (const auto & curve : curves_) {
    length_list_.emplace_back(curve.getLength());
    maximum_2d_curvatures_.emplace_back(curve.getMaximum2DCurvature());
  }
  total_length_ = 0;
  for (const auto & length : length_list_) {
    total_length_ = total_length_ + length;
  }
  checkConnection();
}

std::pair<size_t, double> CatmullRomSpline::getCurveIndexAndS(double s) const
{
  if (s < 0) {
    return std::make_pair(0, s);
  }
  if (s >= total_length_) {
    return std::make_pair(
      curves_.size() - 1, s - (total_length_ - curves_[curves_.size() - 1].getLength()));
  }
  double current_s = 0;
  for (size_t i = 0; i < curves_.size(); i++) {
    double prev_s = current_s;
    current_s = current_s + length_list_[i];
    if (prev_s <= s && s < current_s) {
      return std::make_pair(i, s - prev_s);
    }
  }
  THROW_SIMULATION_ERROR("failed to calculate curve index");  // LCOV_EXCL_LINE
}

double CatmullRomSpline::getSInSplineCurve(size_t curve_index, double s) const
{
  size_t n = curves_.size();
  double ret = 0;
  for (size_t i = 0; i < n; i++) {
    if (i == curve_index) {
      return ret + s;
    } else {
      ret = ret + curves_[i].getLength();
    }
  }
  THROW_SEMANTIC_ERROR("curve index does not match");  // LCOV_EXCL_LINE
}

boost::optional<double> CatmullRomSpline::getCollisionPointIn2D(
  const std::vector<geometry_msgs::msg::Point> & polygon, bool search_backward,
  bool close_start_end) const
{
  size_t n = curves_.size();
  if (search_backward) {
    for (size_t i = 0; i < n; i++) {
      auto s = curves_[n - 1 - i].getCollisionPointIn2D(polygon, search_backward, close_start_end);
      if (s) {
        return getSInSplineCurve(n - 1 - i, s.get());
      }
    }
    return boost::none;
  } else {
    for (size_t i = 0; i < n; i++) {
      auto s = curves_[i].getCollisionPointIn2D(polygon, search_backward, close_start_end);
      if (s) {
        return getSInSplineCurve(i, s.get());
      }
    }
    return boost::none;
  }
  return boost::none;
}

boost::optional<double> CatmullRomSpline::getCollisionPointIn2D(
  const geometry_msgs::msg::Point & point0, const geometry_msgs::msg::Point & point1,
  bool search_backward) const
{
  size_t n = curves_.size();
  if (search_backward) {
    for (size_t i = 0; i < n; i++) {
      auto s = curves_[n - 1 - i].getCollisionPointIn2D(point0, point1, search_backward);
      if (s) {
        return getSInSplineCurve(n - 1 - i, s.get());
      }
    }
    return boost::none;
  } else {
    for (size_t i = 0; i < n; i++) {
      auto s = curves_[i].getCollisionPointIn2D(point0, point1, search_backward);
      if (s) {
        return getSInSplineCurve(i, s.get());
      }
    }
    return boost::none;
  }
  return boost::none;
}

boost::optional<double> CatmullRomSpline::getSValue(
  const geometry_msgs::msg::Pose & pose, double threshold_distance)
{
  double s = 0;
  for (size_t i = 0; i < curves_.size(); i++) {
    auto s_value = curves_[i].getSValue(pose, threshold_distance, true);
    if (s_value) {
      s = s + s_value.get();
      return s;
    }
    s = s + curves_[i].getLength();
  }
  return boost::none;
}

double CatmullRomSpline::getSquaredDistanceIn2D(geometry_msgs::msg::Point point, double s) const
{
  const auto index_and_s = getCurveIndexAndS(s);
  return curves_[index_and_s.first].getSquaredDistanceIn2D(point, index_and_s.second, true);
}

const geometry_msgs::msg::Point CatmullRomSpline::getPoint(double s) const
{
  const auto index_and_s = getCurveIndexAndS(s);
  return curves_[index_and_s.first].getPoint(index_and_s.second, true);
}

double CatmullRomSpline::getMaximum2DCurvature() const
{
  if (maximum_2d_curvatures_.empty()) {
    THROW_SIMULATION_ERROR("maximum 2D curvature vector size is 0.");  // LCOV_EXCL_LINE
  }
  return *std::max_element(maximum_2d_curvatures_.begin(), maximum_2d_curvatures_.end());
}

const geometry_msgs::msg::Vector3 CatmullRomSpline::getNormalVector(double s) const
{
  const auto index_and_s = getCurveIndexAndS(s);
  return curves_[index_and_s.first].getNormalVector(index_and_s.second, true);
}

const geometry_msgs::msg::Vector3 CatmullRomSpline::getTangentVector(double s) const
{
  const auto index_and_s = getCurveIndexAndS(s);
  return curves_[index_and_s.first].getTangentVector(index_and_s.second, true);
}

const geometry_msgs::msg::Pose CatmullRomSpline::getPose(double s) const
{
  const auto index_and_s = getCurveIndexAndS(s);
  return curves_[index_and_s.first].getPose(index_and_s.second, true);
}

bool CatmullRomSpline::checkConnection() const
{
  if (control_points.size() != (curves_.size() + 1)) {
    THROW_SIMULATION_ERROR(                                    // LCOV_EXCL_LINE
      "number of control points and curves does not match.");  // LCOV_EXCL_LINE
  }
  for (size_t i = 0; i < curves_.size(); i++) {
    const auto control_point0 = control_points[i];
    const auto control_point1 = control_points[i + 1];
    const auto p0 = curves_[i].getPoint(0, false);
    const auto p1 = curves_[i].getPoint(1, false);
    if (equals(control_point0, p0) && equals(control_point1, p1)) {
      continue;
    } else if (!equals(control_point0, p0)) {                       // LCOV_EXCL_LINE
      THROW_SIMULATION_ERROR(                                       // LCOV_EXCL_LINE
        "start point of the curve number ", i, " does not match");  // LCOV_EXCL_LINE
    } else if (!equals(control_point1, p1)) {                       // LCOV_EXCL_LINE
      THROW_SIMULATION_ERROR(                                       // LCOV_EXCL_LINE
        "end point of the curve number ", i, " does not match");    // LCOV_EXCL_LINE
    }
  }
  if (curves_.empty()) {
    THROW_SIMULATION_ERROR("curve size should not be zero");  // LCOV_EXCL_LINE
  }
  return true;
}

bool CatmullRomSpline::equals(geometry_msgs::msg::Point p0, geometry_msgs::msg::Point p1) const
{
  constexpr double e = std::numeric_limits<float>::epsilon();
  if (std::abs(p0.x - p1.x) > e) {
    return false;  // LCOV_EXCL_LINE
  }
  if (std::abs(p0.y - p1.y) > e) {
    return false;  // LCOV_EXCL_LINE
  }
  if (std::abs(p0.z - p1.z) > e) {
    return false;  // LCOV_EXCL_LINE
  }
  return true;
}
}  // namespace math
}  // namespace traffic_simulator
