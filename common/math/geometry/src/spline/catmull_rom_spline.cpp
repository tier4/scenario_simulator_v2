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

#include <geometry/spline/catmull_rom_spline.hpp>
#include <iostream>
#include <limits>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <string>
#include <utility>
#include <vector>

namespace math
{
namespace geometry
{
auto CatmullRomSpline::getPolygon(
  const double width, const size_t num_points, const double z_offset)
  -> std::vector<geometry_msgs::msg::Point>
{
  if (num_points == 0) {
    return {};
  }
  std::vector<geometry_msgs::msg::Point> points;
  std::vector<geometry_msgs::msg::Point> left_bounds = getLeftBounds(width, num_points, z_offset);
  std::vector<geometry_msgs::msg::Point> right_bounds = getRightBounds(width, num_points, z_offset);
  for (size_t i = 0; i < static_cast<size_t>(left_bounds.size() - 1); i++) {
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

auto CatmullRomSpline::getRightBounds(
  const double width, const size_t num_points, const double z_offset) const
  -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> points;
  double step_size = getLength() / static_cast<double>(num_points);
  for (size_t i = 0; i < static_cast<size_t>(num_points + 1); i++) {
    double s = step_size * static_cast<double>(i);
    points.emplace_back(
      [this](
        const double width, const double s, const double z_offset) -> geometry_msgs::msg::Point {
        geometry_msgs::msg::Vector3 vec = getNormalVector(s);
        double theta = std::atan2(vec.y, vec.x);
        geometry_msgs::msg::Point p = getPoint(s);
        geometry_msgs::msg::Point point;
        point.x = p.x + 0.5 * width * std::cos(theta);
        point.y = p.y + 0.5 * width * std::sin(theta);
        point.z = p.z + z_offset;
        return point;
      }(width, s, z_offset));
  }
  return points;
}

auto CatmullRomSpline::getLeftBounds(
  const double width, const size_t num_points, const double z_offset) const
  -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> points;
  double step_size = getLength() / static_cast<double>(num_points);
  for (size_t i = 0; i < static_cast<size_t>(num_points + 1); i++) {
    double s = step_size * static_cast<double>(i);
    points.emplace_back(
      [this](
        const double width, const double s, const double z_offset) -> geometry_msgs::msg::Point {
        geometry_msgs::msg::Vector3 vec = getNormalVector(s);
        double theta = std::atan2(vec.y, vec.x);
        geometry_msgs::msg::Point p = getPoint(s);
        geometry_msgs::msg::Point point;
        point.x = p.x - 0.5 * width * std::cos(theta);
        point.y = p.y - 0.5 * width * std::sin(theta);
        point.z = p.z + z_offset;
        return point;
      }(width, s, z_offset));
  }
  return points;
}

auto CatmullRomSpline::getTrajectory(
  const double start_s, const double end_s, const double resolution, const double offset) const
  -> std::vector<geometry_msgs::msg::Point>
{
  if (start_s > end_s) {
    std::vector<geometry_msgs::msg::Point> ret;
    double s = start_s;
    while (s > end_s) {
      ret.emplace_back(getPoint(s, offset));
      s = s - std::fabs(resolution);
    }
    ret.emplace_back(getPoint(end_s, offset));
    return ret;
  } else {
    std::vector<geometry_msgs::msg::Point> ret;
    double s = start_s;
    while (s < end_s) {
      ret.emplace_back(getPoint(s, offset));
      s = s + std::fabs(resolution);
    }
    ret.emplace_back(getPoint(end_s, offset));
    return ret;
  }
}

CatmullRomSpline::CatmullRomSpline(const std::vector<geometry_msgs::msg::Point> & control_points)
: control_points(control_points), line_segments_(getLineSegments(control_points)), total_length_(0)
{
  switch (control_points.size()) {
    case 0:
      THROW_SEMANTIC_ERROR(
        "Control points are empty. We cannot determine the shape of the curve.",
        "This message is not originally intended to be displayed, if you see it, please contact "
        "the developer of traffic_simulator.");
      break;
    /// @note In this case, spline is interpreted as point.
    case 1:
      break;
    /// @note In this case, spline is interpreted as line segment.
    case 2:
      total_length_ = line_segments_[0].length;
      break;
    /// @note In this case, spline is interpreted as curve.
    default:
      [this](const auto & control_points) -> void {
        size_t n = control_points.size() - 1;
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
      }(control_points);
      break;
  }
}

auto CatmullRomSpline::getCurveIndexAndS(const double s) const -> std::pair<size_t, double>
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

auto CatmullRomSpline::getSInSplineCurve(const size_t curve_index, const double s) const -> double
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

auto CatmullRomSpline::getCollisionPointsIn2D(
  const std::vector<geometry_msgs::msg::Point> & polygon, const bool search_backward) const
  -> std::set<double>
{
  if (polygon.size() <= 1) {
    THROW_SIMULATION_ERROR(
      "Number of points in polygon are invalid, it requires more than 2 points but only ",
      static_cast<int>(polygon.size()), " exists.",
      " This message is not originally intended to be displayed, if you see it, please contact the "
      "developer of traffic_simulator.");
  }
  /// @note If the spline has three or more control points.
  const auto get_collision_point_2d_with_curve =
    [this](const auto & polygon, const auto search_backward) -> std::set<double> {
    std::set<double> s_value_candidates;
    for (size_t i = 0; i < curves_.size(); ++i) {
      /// @note The polygon is assumed to be closed
      const auto s = curves_[i].getCollisionPointsIn2D(polygon, search_backward, true, true);
      std::for_each(s.begin(), s.end(), [&s_value_candidates, i, this](const auto & s) {
        s_value_candidates.insert(getSInSplineCurve(i, s));
      });
    }
    return s_value_candidates;
  };
  /// @note If the spline has two control points. (Same as single line segment.)
  const auto get_collision_point_2d_with_line = [this](const auto & polygon) -> std::set<double> {
    std::set<double> s_value_candidates;
    for (const auto & line : getLineSegments(polygon)) {
      if (static_cast<int>(line_segments_.size()) != 1) {
        THROW_SIMULATION_ERROR(
          "Number of the line segments are invalid : ", static_cast<int>(line_segments_.size()),
          "Number of the line segments should be 1 when the spline have 2 control points.",
          "Something completely unexpected happened.",
          "This message is not originally intended to be displayed, if you see it, please "
          "contact the developer of traffic_simulator.");
      }
      if (const auto s = line_segments_[0].get2DIntersectionSValue(line, true)) {
        s_value_candidates.insert(s.value());
      }
    }
    return s_value_candidates;
  };
  /// @note If the spline has one control point. (Same as point.)
  const auto get_collision_point_2d_with_point = [this](const auto & polygon) -> std::set<double> {
    std::set<double> s_value_candidates;
    for (const auto & line : getLineSegments(polygon)) {
      if (line.isIntersect2D(control_points[0])) {
        s_value_candidates.insert(0.0);
      }
    }
    return s_value_candidates;
  };

  switch (control_points.size()) {
    case 0:
      THROW_SEMANTIC_ERROR(
        "Control points are empty. We cannot determine the shape of the curve.",
        "This message is not originally intended to be displayed, if you see it, please contact "
        "the developer of traffic_simulator.");
      break;
    /// @note In this case, spline is interpreted as point.
    case 1:
      return get_collision_point_2d_with_point(polygon);
    /// @note In this case, spline is interpreted as line segment.
    case 2:
      return get_collision_point_2d_with_line(polygon);
    /// @note In this case, spline is interpreted as curve.
    default:
      return get_collision_point_2d_with_curve(polygon, search_backward);
  }
}

/**
 * @brief Get collision point in 2D (x and y)
 * @param polygon points of polygons.
 * @param search_backward If true, return collision points with maximum s value. If false, return collision points with minimum s value.
 * @return std::optional<double> Denormalized s values along the frenet coordinate of the spline curve.
 */
auto CatmullRomSpline::getCollisionPointIn2D(
  const std::vector<geometry_msgs::msg::Point> & polygon, const bool search_backward) const
  -> std::optional<double>
{
  std::set<double> s_value_candidates = getCollisionPointsIn2D(polygon, search_backward);
  if (s_value_candidates.empty()) {
    return std::nullopt;
  }
  if (search_backward) {
    return *(s_value_candidates.rbegin());
  }
  return *(s_value_candidates.begin());
}

auto CatmullRomSpline::getCollisionPointIn2D(
  const geometry_msgs::msg::Point & point0, const geometry_msgs::msg::Point & point1,
  const bool search_backward) const -> std::optional<double>
{
  size_t n = curves_.size();
  if (search_backward) {
    for (size_t i = 0; i < n; i++) {
      auto s = curves_[n - 1 - i].getCollisionPointIn2D(point0, point1, search_backward, true);
      if (s) {
        return getSInSplineCurve(n - 1 - i, s.value());
      }
    }
    return std::nullopt;
  } else {
    for (size_t i = 0; i < n; i++) {
      auto s = curves_[i].getCollisionPointIn2D(point0, point1, search_backward, true);
      if (s) {
        return getSInSplineCurve(i, s.value());
      }
    }
    return std::nullopt;
  }
  return std::nullopt;
}

auto CatmullRomSpline::getSValue(
  const geometry_msgs::msg::Pose & pose, const double threshold_distance) const
  -> std::optional<double>
{
  switch (control_points.size()) {
    case 0:
      THROW_SIMULATION_ERROR(
        "Only 0 control point exists for the spline.",
        "If this message was displayed, something completely unexpected happens.",
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    case 1:
      if (
        std::pow(control_points[0].x - pose.position.x, 2) +
          std::pow(control_points[0].y - pose.position.y, 2) <=
        std::numeric_limits<double>::epsilon()) {
        return 0.0;
      }
      return std::nullopt;
    case 2:
      if (static_cast<int>(line_segments_.size()) != 1) {
        THROW_SIMULATION_ERROR(
          "Number of the line segments are invalid : ", static_cast<int>(line_segments_.size()),
          "Number of the line segments should be 1 when the spline have 2 control points.",
          "Something completely unexpected happened.",
          "This message is not originally intended to be displayed, if you see it, please "
          "contact the developer of traffic_simulator.");
      }
      return line_segments_[0].getSValue(pose, threshold_distance, true);
    default:
      double s = 0;
      for (size_t i = 0; i < curves_.size(); i++) {
        auto s_value = curves_[i].getSValue(pose, threshold_distance, true);
        if (s_value) {
          s = s + s_value.value();
          return s;
        }
        s = s + curves_[i].getLength();
      }
      return std::nullopt;
  }
}

auto CatmullRomSpline::getSquaredDistanceIn2D(
  const geometry_msgs::msg::Point & point, const double s) const -> double
{
  switch (control_points.size()) {
    case 0:
      THROW_SIMULATION_ERROR(
        "Only 0 control point exists for the spline.",
        "If this message was displayed, something completely unexpected happens.",
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    case 1:
      if (std::fabs(s) <= std::numeric_limits<double>::epsilon()) {
        return std::pow(control_points[0].x - point.x, 2) +
               std::pow(control_points[0].y - point.y, 2);
      }
      THROW_SIMULATION_ERROR(
        "Only 1 control point exists for the spline.",
        "So only when the s=0 can calculate point."
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    case 2:
      if (static_cast<int>(line_segments_.size()) != 1) {
        THROW_SIMULATION_ERROR(
          "Number of the line segments are invalid : ", static_cast<int>(line_segments_.size()),
          "Number of the line segments should be 1 when the spline have 2 control points.",
          "Something completely unexpected happened.",
          "This message is not originally intended to be displayed, if you see it, please "
          "contact the developer of traffic_simulator.");
      }
      return line_segments_[0].getSquaredDistanceIn2D(point, s, true);
    default:
      const auto index_and_s = getCurveIndexAndS(s);
      return curves_[index_and_s.first].getSquaredDistanceIn2D(point, index_and_s.second, true);
  }
}

auto CatmullRomSpline::getSquaredDistanceVector(
  const geometry_msgs::msg::Point & point, const double s) const -> geometry_msgs::msg::Vector3
{
  switch (control_points.size()) {
    case 0:
      THROW_SIMULATION_ERROR(
        "Only 0 control point exists for the spline.",
        "If this message was displayed, something completely unexpected happens.",
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    case 1:
      if (std::fabs(s) <= std::numeric_limits<double>::epsilon()) {
        return geometry_msgs::build<geometry_msgs::msg::Vector3>()
          .x(point.x - control_points[0].x)
          .y(point.y - control_points[0].y)
          .z(point.z - control_points[0].z);
      }
      THROW_SIMULATION_ERROR(
        "Only 1 control point exists for the spline.",
        "So only when the s=0 can calculate point."
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    case 2:
      if (static_cast<int>(line_segments_.size()) != 1) {
        THROW_SIMULATION_ERROR(
          "Number of the line segments are invalid : ", static_cast<int>(line_segments_.size()),
          "Number of the line segments should be 1 when the spline have 2 control points.",
          "Something completely unexpected happened.",
          "This message is not originally intended to be displayed, if you see it, please "
          "contact the developer of traffic_simulator.");
      }
      return line_segments_[0].getSquaredDistanceVector(point, s, true);
    default:
      const auto index_and_s = getCurveIndexAndS(s);
      return curves_[index_and_s.first].getSquaredDistanceVector(point, index_and_s.second, true);
  }
}

auto CatmullRomSpline::getPoint(const double s) const -> geometry_msgs::msg::Point
{
  switch (control_points.size()) {
    case 0:
      THROW_SIMULATION_ERROR(
        "Only 0 control point exists for the spline.",
        "If this message was displayed, something completely unexpected happens.",
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    case 1:
      if (std::fabs(s) <= std::numeric_limits<double>::epsilon()) {
        return control_points[0];
      }
      THROW_SIMULATION_ERROR(
        "Only 1 control point exists for the spline.",
        "So only when the s=0 can calculate point."
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    case 2:
      if (static_cast<int>(line_segments_.size()) != 1) {
        THROW_SIMULATION_ERROR(
          "Number of the line segments are invalid : ", static_cast<int>(line_segments_.size()),
          "Number of the line segments should be 1 when the spline have 2 control points.",
          "Something completely unexpected happened.",
          "This message is not originally intended to be displayed, if you see it, please "
          "contact the developer of traffic_simulator.");
      }
      return line_segments_[0].getPoint(s, true);
    default:
      const auto index_and_s = getCurveIndexAndS(s);
      return curves_[index_and_s.first].getPoint(index_and_s.second, true);
  }
}

auto CatmullRomSpline::getPoint(const double s, const double offset) const
  -> geometry_msgs::msg::Point
{
  geometry_msgs::msg::Vector3 vec = getNormalVector(s);
  double theta = std::atan2(vec.y, vec.x);
  geometry_msgs::msg::Point p = getPoint(s);
  geometry_msgs::msg::Point point;
  point.x = p.x + offset * std::cos(theta);
  point.y = p.y + offset * std::sin(theta);
  point.z = p.z;
  return point;
}

auto CatmullRomSpline::getMaximum2DCurvature() const -> double
{
  if (maximum_2d_curvatures_.empty()) {
    THROW_SIMULATION_ERROR("maximum 2D curvature vector size is 0.");  // LCOV_EXCL_LINE
  }
  const auto [min, max] =
    std::minmax_element(maximum_2d_curvatures_.begin(), maximum_2d_curvatures_.end());
  if (std::fabs(*min) > std::fabs(*max)) {
    return *min;
  }
  return *max;
}

auto CatmullRomSpline::getNormalVector(const double s) const -> geometry_msgs::msg::Vector3
{
  switch (control_points.size()) {
    case 0:
      THROW_SIMULATION_ERROR(
        "Only 0 control point exists for the spline.",
        "If this message was displayed, something completely unexpected happens.",
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    case 1:
      THROW_SIMULATION_ERROR(
        "Only 1 control point exists for the spline.",
        "So, the normal vector cannot be calculated.",
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    case 2:
      if (0 <= s && s <= getLength()) {
        return line_segments_[0].getNormalVector();
      }
      THROW_SIMULATION_ERROR(
        "Invalid S value is specified, while getting normal vector.",
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    default:
      const auto index_and_s = getCurveIndexAndS(s);
      return curves_[index_and_s.first].getNormalVector(index_and_s.second, true);
  }
}

auto CatmullRomSpline::getTangentVector(const double s) const -> geometry_msgs::msg::Vector3
{
  switch (control_points.size()) {
    case 0:
      THROW_SIMULATION_ERROR(
        "Only 0 control point exists for the spline.",
        "If this message was displayed, something completely unexpected happens.",
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    case 1:
      THROW_SIMULATION_ERROR(
        "Only 1 control point exists for the spline.",
        "So, the tangent vector cannot be calculated.",
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    case 2:
      if (static_cast<int>(line_segments_.size()) != 1) {
        THROW_SIMULATION_ERROR(
          "Number of the line segments are invalid : ", static_cast<int>(line_segments_.size()),
          "Number of the line segments should be 1 when the spline have 2 control points.",
          "Something completely unexpected happened.",
          "This message is not originally intended to be displayed, if you see it, please "
          "contact the developer of traffic_simulator.");
      }
      if (0 <= s && s <= getLength()) {
        return line_segments_[0].vector;
      }
      THROW_SIMULATION_ERROR(
        "Invalid S value is specified, while getting tangent vector.",
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    default:
      const auto index_and_s = getCurveIndexAndS(s);
      return curves_[index_and_s.first].getTangentVector(index_and_s.second, true);
  }
}

auto CatmullRomSpline::getPose(const double s, const bool fill_pitch) const
  -> geometry_msgs::msg::Pose
{
  switch (control_points.size()) {
    case 0:
      THROW_SIMULATION_ERROR(
        "Only 0 control point exists for the spline.",
        "If this message was displayed, something completely unexpected happens.",
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    case 1:
      THROW_SIMULATION_ERROR(
        "Only 1 control point exists for the spline.", "So, the pose cannot be calculated.",
        "This message is not originally intended to be displayed, if you see it, please "
        "contact the developer of traffic_simulator.");
    case 2:
      if (static_cast<int>(line_segments_.size()) != 1) {
        THROW_SIMULATION_ERROR(
          "Number of the line segments are invalid : ", static_cast<int>(line_segments_.size()),
          "Number of the line segments should be 1 when the spline have 2 control points.",
          "Something completely unexpected happened.",
          "This message is not originally intended to be displayed, if you see it, please "
          "contact the developer of traffic_simulator.");
      }
      return line_segments_[0].getPose(s, true, fill_pitch);
    default:
      const auto index_and_s = getCurveIndexAndS(s);
      return curves_[index_and_s.first].getPose(index_and_s.second, true, fill_pitch);
  }
}

auto CatmullRomSpline::checkConnection() const -> bool
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

auto CatmullRomSpline::equals(
  const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1) const -> bool
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
}  // namespace geometry
}  // namespace math
