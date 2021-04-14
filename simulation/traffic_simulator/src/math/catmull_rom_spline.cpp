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
#include <string>
#include <traffic_simulator/math/catmull_rom_spline.hpp>
#include <utility>
#include <vector>

namespace traffic_simulator
{
namespace math
{
CatmullRomSpline::CatmullRomSpline(
  const std::vector<openscenario_msgs::msg::HermiteCurve> & hermite_curves)
{
  for (const auto & curve : hermite_curves) {
    curves_.emplace_back(HermiteCurve(curve));
  }
}

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
  double start_s, double end_s, double num_points) const
{
  std::vector<geometry_msgs::msg::Point> traj;
  num_points = std::fabs(num_points);
  double s = start_s;
  if (start_s < end_s) {
    while (s < end_s) {
      auto p = getPoint(s);
      traj.emplace_back(p);
      s = s + num_points;
    }
  } else {
    while (s < end_s) {
      auto p = getPoint(s);
      traj.emplace_back(p);
      s = s - num_points;
    }
  }
  return traj;
}

CatmullRomSpline::CatmullRomSpline(const openscenario_msgs::msg::CatmullRomSpline & spline)
{
  for (const auto & curve : spline.curves) {
    curves_.emplace_back(HermiteCurve(curve));
  }
}

CatmullRomSpline::CatmullRomSpline(std::vector<geometry_msgs::msg::Point> control_points)
: control_points_(control_points)
{
  size_t n = control_points_.size() - 1;
  if (control_points_.size() <= 1) {
    throw SplineInterpolationError("numbers of control points are not enough.");
  }
  if (control_points_.size() == 2) {
    const auto p0 = control_points_[0];
    const auto p1 = control_points_[1];
    geometry_msgs::msg::Point p2;
    p2.x = (p0.x + p1.x) * 0.5;
    p2.y = (p0.y + p1.y) * 0.5;
    p2.z = (p0.z + p1.z) * 0.5;
    control_points_.clear();
    control_points_.emplace_back(p0);
    control_points_.emplace_back(p2);
    control_points_.emplace_back(p1);
  }
  for (size_t i = 0; i < n; i++) {
    if (i == 0) {
      double ax = 0;
      double bx = control_points_[0].x - 2 * control_points_[1].x + control_points_[2].x;
      double cx = -3 * control_points_[0].x + 4 * control_points_[1].x - control_points_[2].x;
      double dx = 2 * control_points_[0].x;
      double ay = 0;
      double by = control_points_[0].y - 2 * control_points_[1].y + control_points_[2].y;
      double cy = -3 * control_points_[0].y + 4 * control_points_[1].y - control_points_[2].y;
      double dy = 2 * control_points_[0].y;
      double az = 0;
      double bz = control_points_[0].z - 2 * control_points_[1].z + control_points_[2].z;
      double cz = -3 * control_points_[0].z + 4 * control_points_[1].z - control_points_[2].z;
      double dz = 2 * control_points_[0].z;
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
      double bx = control_points_[i - 1].x - 2 * control_points_[i].x + control_points_[i + 1].x;
      double cx = -1 * control_points_[i - 1].x + control_points_[i + 1].x;
      double dx = 2 * control_points_[i].x;
      double ay = 0;
      double by = control_points_[i - 1].y - 2 * control_points_[i].y + control_points_[i + 1].y;
      double cy = -1 * control_points_[i - 1].y + control_points_[i + 1].y;
      double dy = 2 * control_points_[i].y;
      double az = 0;
      double bz = control_points_[i - 1].z - 2 * control_points_[i].z + control_points_[i + 1].z;
      double cz = -1 * control_points_[i - 1].z + control_points_[i + 1].z;
      double dz = 2 * control_points_[i].z;
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
      double ax = -1 * control_points_[i - 1].x + 3 * control_points_[i].x -
                  3 * control_points_[i + 1].x + control_points_[i + 2].x;
      double bx = 2 * control_points_[i - 1].x - 5 * control_points_[i].x +
                  4 * control_points_[i + 1].x - control_points_[i + 2].x;
      double cx = -control_points_[i - 1].x + control_points_[i + 1].x;
      double dx = 2 * control_points_[i].x;
      double ay = -1 * control_points_[i - 1].y + 3 * control_points_[i].y -
                  3 * control_points_[i + 1].y + control_points_[i + 2].y;
      double by = 2 * control_points_[i - 1].y - 5 * control_points_[i].y +
                  4 * control_points_[i + 1].y - control_points_[i + 2].y;
      double cy = -control_points_[i - 1].y + control_points_[i + 1].y;
      double dy = 2 * control_points_[i].y;
      double az = -1 * control_points_[i - 1].z + 3 * control_points_[i].z -
                  3 * control_points_[i + 1].z + control_points_[i + 2].z;
      double bz = 2 * control_points_[i - 1].z - 5 * control_points_[i].z +
                  4 * control_points_[i + 1].z - control_points_[i + 2].z;
      double cz = -control_points_[i - 1].z + control_points_[i + 1].z;
      double dz = 2 * control_points_[i].z;
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
    maximum_2d_curvatures_.emplace_back(curve.getMaximu2DCurvature());
  }
  total_length_ = 0;
  for (const auto & length : length_list_) {
    total_length_ = total_length_ + length;
  }
  checkConnection();
}

const openscenario_msgs::msg::CatmullRomSpline CatmullRomSpline::toRosMsg() const
{
  openscenario_msgs::msg::CatmullRomSpline spline;
  for (const auto & curve : curves_) {
    spline.curves.emplace_back(curve.toRosMsg());
  }
  return spline;
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
  throw SplineInterpolationError("failed to calculate curve index");
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
  throw SplineInterpolationError("curve index does not match.");
}

boost::optional<double> CatmullRomSpline::getCollisionPointIn2D(
  std::vector<geometry_msgs::msg::Point> polygon, bool search_backward) const
{
  size_t n = curves_.size();
  if (search_backward) {
    for (size_t i = 0; i < n; i++) {
      auto s = curves_[n - 1 - i].getCollisionPointIn2D(polygon, search_backward);
      if (s) {
        return getSInSplineCurve(n - 1 - i, s.get());
      }
    }
    return boost::none;
  } else {
    for (size_t i = 0; i < n; i++) {
      auto s = curves_[i].getCollisionPointIn2D(polygon, search_backward);
      if (s) {
        return getSInSplineCurve(i, s.get());
      }
    }
    return boost::none;
  }
  return boost::none;
}

boost::optional<double> CatmullRomSpline::getCollisionPointIn2D(
  geometry_msgs::msg::Point point0, geometry_msgs::msg::Point point1, bool search_backward) const
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

const std::vector<geometry_msgs::msg::Point> CatmullRomSpline::getTrajectory(int num_points) const
{
  std::vector<geometry_msgs::msg::Point> ret;
  if (num_points <= 1) {
    throw SplineInterpolationError("trajectory points should be more than 2.");
  }
  double seg_size = total_length_ / (num_points - 1);
  for (int i = 0; i < num_points; i++) {
    double s = seg_size * static_cast<double>(i);
    ret.emplace_back(getPoint(s));
  }
  return ret;
}

boost::optional<double> CatmullRomSpline::getSValue(
  geometry_msgs::msg::Point position, double threadhold_distance, unsigned int initial_num_points,
  unsigned int max_iteration, double tolerance)
{
  std::vector<double> s_values;
  std::vector<double> error_values;
  std::vector<size_t> curve_index;
  for (size_t i = 0; i < curves_.size(); i++) {
    auto s_value = curves_[i].getSValue(
      position, threadhold_distance, initial_num_points, max_iteration, tolerance, true);
    if (s_value) {
      if (s_value.get() > 0 && s_value.get() < curves_[i].getLength()) {
        s_values.emplace_back(s_value.get());
        error_values.emplace_back(curves_[i].getSquaredDistanceIn2D(position, s_value.get(), true));
        curve_index.emplace_back(i);
      }
    }
  }
  if (s_values.size() != error_values.size()) {
    throw SplineInterpolationError("s values and error values size are does not match.");
  }
  if (s_values.size() != curve_index.size()) {
    throw SplineInterpolationError("s values and error values size are does not match.");
  }
  if (s_values.empty()) {
    return boost::none;
  }
  double s = 0;
  auto iter = std::max_element(error_values.begin(), error_values.end());
  size_t min_error_index = std::distance(error_values.begin(), iter);
  for (size_t i = 0; i <= curve_index[min_error_index]; i++) {
    if (i == curve_index[min_error_index]) {
      s = s + s_values[min_error_index];
      break;
    } else {
      s = s + curves_[i].getLength();
    }
  }
  return s;
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

double CatmullRomSpline::getMaximum2DCurventure() const
{
  if (maximum_2d_curvatures_.empty()) {
    throw SplineInterpolationError("maximum 2D curventure vector size is 0.");
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
  for (size_t i = 0; i < curves_.size(); i++) {
    const auto control_point0 = control_points_[i];
    const auto control_point1 = control_points_[i + 1];
    const auto p0 = curves_[i].getPoint(0, false);
    const auto p1 = curves_[i].getPoint(1, false);
    if (equals(control_point0, p0) && equals(control_point1, p1)) {
      continue;
    } else if (!equals(control_point0, p0)) {
      throw SplineInterpolationError(
        "start point of the curve number " + std::to_string(i) + " does not match.");
    } else if (!equals(control_point1, p1)) {
      throw SplineInterpolationError(
        "end point of the curve number " + std::to_string(i) + " does not match.");
    }
  }
  if (curves_.empty()) {
    throw SplineInterpolationError("curve size should not be zero.");
  }
  return true;
}

bool CatmullRomSpline::equals(geometry_msgs::msg::Point p0, geometry_msgs::msg::Point p1) const
{
  constexpr double e = std::numeric_limits<float>::epsilon();
  if (std::abs(p0.x - p1.x) > e) {
    return false;
  }
  if (std::abs(p0.y - p1.y) > e) {
    return false;
  }
  if (std::abs(p0.z - p1.z) > e) {
    return false;
  }
  return true;
}
}  // namespace math
}  // namespace traffic_simulator
