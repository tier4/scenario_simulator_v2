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

#include <algorithm>
#include <array>
#include <cmath>
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
  const double width, const std::size_t num_points, const double z_offset)
  -> std::vector<geometry_msgs::msg::Point>
{
  if (num_points == 0) {
    return {};
  }
  std::vector<geometry_msgs::msg::Point> points;
  std::vector<geometry_msgs::msg::Point> left_bounds = getLeftBounds(width, num_points, z_offset);
  std::vector<geometry_msgs::msg::Point> right_bounds = getRightBounds(width, num_points, z_offset);
  for (std::size_t i = 0; i < left_bounds.size() - 1; i++) {
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
  const double width, const std::size_t num_points, const double z_offset) const
  -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> points;
  double step_size = getLength() / static_cast<double>(num_points);
  for (std::size_t i = 0; i < num_points + 1; i++) {
    double s = step_size * static_cast<double>(i);
    points.emplace_back(
      [this](const double local_width, const double local_s, const double local_z_offset) {
        geometry_msgs::msg::Vector3 vec = getNormalVector(local_s);
        double theta = std::atan2(vec.y, vec.x);
        geometry_msgs::msg::Point p = getPoint(local_s);
        geometry_msgs::msg::Point point;
        point.x = p.x + 0.5 * local_width * std::cos(theta);
        point.y = p.y + 0.5 * local_width * std::sin(theta);
        point.z = p.z + local_z_offset;
        return point;
      }(width, s, z_offset));
  }
  return points;
}

auto CatmullRomSpline::getLeftBounds(
  const double width, const std::size_t num_points, const double z_offset) const
  -> std::vector<geometry_msgs::msg::Point>
{
  std::vector<geometry_msgs::msg::Point> points;
  double step_size = getLength() / static_cast<double>(num_points);
  for (std::size_t i = 0; i < num_points + 1; i++) {
    double s = step_size * static_cast<double>(i);
    points.emplace_back(
      [this](const double local_width, const double local_s, const double local_z_offset) {
        geometry_msgs::msg::Vector3 vec = getNormalVector(local_s);
        double theta = std::atan2(vec.y, vec.x);
        geometry_msgs::msg::Point p = getPoint(local_s);
        geometry_msgs::msg::Point point;
        point.x = p.x - 0.5 * local_width * std::cos(theta);
        point.y = p.y - 0.5 * local_width * std::sin(theta);
        point.z = p.z + local_z_offset;
        return point;
      }(width, s, z_offset));
  }
  return points;
}

auto CatmullRomSpline::getTrajectory(
  const double start_s, const double end_s, const double resolution, const double offset) const
  -> std::vector<geometry_msgs::msg::Point>
{
  if (std::fabs(resolution) <= std::numeric_limits<double>::epsilon()) {
    THROW_SIMULATION_ERROR(
      "Resolution should not be zero.",
      "This message is not originally intended to be displayed, if you see it, please contact ",
      "the developer of traffic_simulator.");
  }
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

auto CatmullRomSpline::getTrajectoryPoses(
  const double start_s, const double end_s, const double resolution) const
  -> std::vector<geometry_msgs::msg::Pose>
{
  if (std::fabs(resolution) <= std::numeric_limits<double>::epsilon()) {
    THROW_SIMULATION_ERROR(
      "Resolution should not be zero.",
      "This message is not originally intended to be displayed, if you see it, please contact ",
      "the developer of traffic_simulator.");
  }
  if (start_s > end_s) {
    std::vector<geometry_msgs::msg::Pose> ret;
    double s = start_s;
    while (s > end_s) {
      ret.emplace_back(getPose(s));
      s = s - std::fabs(resolution);
    }
    ret.emplace_back(getPose(end_s));
    return ret;
  } else {
    std::vector<geometry_msgs::msg::Pose> ret;
    double s = start_s;
    while (s < end_s) {
      ret.emplace_back(getPose(s));
      s = s + std::fabs(resolution);
    }
    ret.emplace_back(getPose(end_s));
    return ret;
  }
}

CatmullRomSpline::CatmullRomSpline(
  const geometry_msgs::msg::Point & start_point,
  const std::shared_ptr<traffic_simulator_msgs::msg::PolylineTrajectory> & trajectory)
: CatmullRomSpline([start_point, trajectory] {
    if (!trajectory) {
      THROW_SIMULATION_ERROR(
        "Trajectory is empty.",
        "This message is not originally intended to be displayed, if you see it, please contact "
        "the developer of traffic_simulator.");
    }
    std::vector<geometry_msgs::msg::Point> control_points;
    control_points.reserve(trajectory->shape.vertices.size() + 2UL);
    control_points.emplace_back(start_point);
    for (const auto & vertex : trajectory->shape.vertices) {
      control_points.emplace_back(vertex.position.position);
    }
    if (trajectory->closed) {
      control_points.emplace_back(control_points[0]);
    }
    return control_points;
  }())
{
}

CatmullRomSpline::CatmullRomSpline(
  const std::vector<geometry_msgs::msg::Point> & raw_control_points)
: control_points([raw_control_points]() {
    std::vector<geometry_msgs::msg::Point> ret = raw_control_points;
    ret.erase(std::unique(ret.begin(), ret.end()), ret.end());
    return ret;
  }()),
  line_segments_(getLineSegments(control_points)),
  total_length_(0)
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
        /**
         * @note Centripetal Catmull-Rom (alpha = 0.5): the knot interval of a segment is the
         * square root of its chord length. The uniform variant (every interval 1) gives each
         * segment a tangent of half the chord of its neighbours regardless of its own length, so
         * a short segment between long ones overshoots its end point and turns back, reversing
         * the heading even when every control point lies on one line. With centripetal knots the
         * tangent at either end of a segment is less than twice its chord, so for collinear
         * monotonic points the segment stays monotonic (Fritsch-Carlson). For equal chord lengths
         * the curves are the same as the uniform ones.
         */
        const std::size_t n = control_points.size() - 1;
        const auto interval = [&control_points](const std::size_t i) {
          const auto & p0 = control_points[i];
          const auto & p1 = control_points[i + 1];
          /// @note std::unique removed equal neighbours; the guard covers an underflowing distance.
          return std::max(
            std::sqrt(std::hypot(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z)),
            std::numeric_limits<double>::min());
        };
        std::vector<double> intervals(n);
        for (std::size_t i = 0; i < n; i++) {
          intervals[i] = interval(i);
        }
        /// @note Derivative with respect to the knot parameter at the inner control point i.
        const auto velocity = [&control_points, &intervals](const std::size_t i) {
          const auto & p0 = control_points[i - 1];
          const auto & p1 = control_points[i];
          const auto & p2 = control_points[i + 1];
          const double d0 = intervals[i - 1];
          const double d1 = intervals[i];
          const auto component = [d0, d1](const double v0, const double v1, const double v2) {
            return (v1 - v0) / d0 - (v2 - v0) / (d0 + d1) + (v2 - v1) / d1;
          };
          return std::array<double, 3>{
            component(p0.x, p1.x, p2.x), component(p0.y, p1.y, p2.y), component(p0.z, p1.z, p2.z)};
        };
        for (std::size_t i = 0; i < n; i++) {
          const auto & p0 = control_points[i];
          const auto & p1 = control_points[i + 1];
          const std::array<double, 3> start{p0.x, p0.y, p0.z};
          const std::array<double, 3> chord{p1.x - p0.x, p1.y - p0.y, p1.z - p0.z};
          /// @note a t^3 + b t^2 + c t + d for each axis, t in [0, 1] over this segment.
          std::array<double, 3> a{}, b{}, c{}, d{};
          if (i == 0) {
            /// @note A quadratic ending with the tangent of the next segment's start.
            const auto v = velocity(1);
            for (std::size_t k = 0; k < 3; k++) {
              const double m1 = v[k] * intervals[0];
              b[k] = m1 - chord[k];
              c[k] = 2 * chord[k] - m1;
              d[k] = start[k];
            }
          } else if (i == (n - 1)) {
            /// @note A quadratic starting with the tangent of the previous segment's end.
            const auto v = velocity(i);
            for (std::size_t k = 0; k < 3; k++) {
              const double m0 = v[k] * intervals[i];
              b[k] = chord[k] - m0;
              c[k] = m0;
              d[k] = start[k];
            }
          } else {
            const auto v0 = velocity(i);
            const auto v1 = velocity(i + 1);
            for (std::size_t k = 0; k < 3; k++) {
              const double m0 = v0[k] * intervals[i];
              const double m1 = v1[k] * intervals[i];
              a[k] = -2 * chord[k] + m0 + m1;
              b[k] = 3 * chord[k] - 2 * m0 - m1;
              c[k] = m0;
              d[k] = start[k];
            }
          }
          curves_.emplace_back(
            a[0], b[0], c[0], d[0], a[1], b[1], c[1], d[1], a[2], b[2], c[2], d[2]);
        }
        for (const auto & curve : curves_) {
          length_list_.emplace_back(curve.getLength());
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

auto CatmullRomSpline::getCurveIndexAndS(const double s) const -> std::pair<std::size_t, double>
{
  if (s < 0) {
    return std::make_pair(0, s);
  }
  if (s >= total_length_) {
    return std::make_pair(
      curves_.size() - 1, s - (total_length_ - curves_[curves_.size() - 1].getLength()));
  }
  double current_s = 0;
  for (std::size_t i = 0; i < curves_.size(); i++) {
    double prev_s = current_s;
    current_s = current_s + length_list_[i];
    if (prev_s <= s && s < current_s) {
      return std::make_pair(i, s - prev_s);
    }
  }
  THROW_SIMULATION_ERROR("failed to calculate curve index");  // LCOV_EXCL_LINE
}

auto CatmullRomSpline::getSInSplineCurve(const std::size_t curve_index, const double s) const
  -> double
{
  std::size_t n = curves_.size();
  double ret = 0;
  for (std::size_t i = 0; i < n; i++) {
    if (i == curve_index) {
      return ret + s;
    } else {
      ret = ret + curves_[i].getLength();
    }
  }
  THROW_SEMANTIC_ERROR("curve index does not match");  // LCOV_EXCL_LINE
}

auto CatmullRomSpline::getCollisionPointsIn2D(
  const std::vector<geometry_msgs::msg::Point> & polygon, const bool search_backward,
  const std::optional<std::pair<double, double>> & s_range) const -> std::set<double>
{
  if (polygon.size() <= 1) {
    THROW_SIMULATION_ERROR(
      "Number of points in polygon are invalid, it requires more than 2 points but only ",
      static_cast<int>(polygon.size()), " exists.",
      " This message is not originally intended to be displayed, if you see it, please contact the "
      "developer of traffic_simulator.");
  }
  /// @note If the spline has three or more control points.
  const auto get_collision_point_2d_with_curve = [this, &s_range](
                                                   const auto & local_polygon,
                                                   const auto local_search_backward) {
    std::set<double> s_value_candidates;
    auto current_curve_start_s = 0.0;

    const auto is_curve_in_range = [this, &s_range](
                                     const auto & curve, const auto & current_curve_start_s) {
      bool is_in_range{false};
      // range starts after beginning of current curve or on current curve
      if (
        (current_curve_start_s >= s_range->first) ||
        (current_curve_start_s + curve.getLength()) > s_range->first) {
        // beginning of current curve is before end of range
        if (current_curve_start_s <= s_range->second) {
          is_in_range = true;
        }
      }
      return is_in_range;
    };

    for (const auto & curve : curves_) {
      if (s_range == std::nullopt || is_curve_in_range(curve, current_curve_start_s)) {
        /// @note The local_polygon is assumed to be closed
        const auto s =
          curve.getCollisionPointsIn2D(local_polygon, local_search_backward, true, true);
        std::for_each(
          s.begin(), s.end(), [&s_value_candidates, &current_curve_start_s, this](const auto & s) {
            s_value_candidates.insert(current_curve_start_s + s);
          });
      }
      current_curve_start_s += curve.getLength();
    }
    return s_value_candidates;
  };
  /// @note If the spline has two control points. (Same as single line segment.)
  const auto get_collision_point_2d_with_line = [this](const auto & local_polygon) {
    std::set<double> s_value_candidates;
    for (const auto & line : getLineSegments(local_polygon)) {
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
  const auto get_collision_point_2d_with_point = [this](const auto & local_polygon) {
    std::set<double> s_value_candidates;
    for (const auto & line : getLineSegments(local_polygon)) {
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
  const std::vector<geometry_msgs::msg::Point> & polygon, const bool search_backward,
  const std::optional<std::pair<double, double>> & s_range) const -> std::optional<double>
{
  std::set<double> s_value_candidates = getCollisionPointsIn2D(polygon, search_backward, s_range);
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
  std::size_t n = curves_.size();
  if (search_backward) {
    for (std::size_t i = 0; i < n; i++) {
      auto s = curves_[n - 1 - i].getCollisionPointIn2D(point0, point1, search_backward, true);
      if (s) {
        return getSInSplineCurve(n - 1 - i, s.value());
      }
    }
    return std::nullopt;
  } else {
    for (std::size_t i = 0; i < n; i++) {
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
      for (const auto & curve : curves_) {
        auto s_value = curve.getSValue(pose, threshold_distance, true);
        if (s_value) {
          s = s + s_value.value();
          return s;
        }
        s = s + curve.getLength();
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
      const auto [index, s_value] = getCurveIndexAndS(s);
      return curves_[index].getSquaredDistanceIn2D(point, s_value, true);
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
      const auto [index, s_value] = getCurveIndexAndS(s);
      return curves_[index].getSquaredDistanceVector(point, s_value, true);
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
      const auto [index, s_value] = getCurveIndexAndS(s);
      return curves_[index].getPoint(s_value, true);
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

/**
 * @brief Get approximation of min and max altitude - from the control points
 */
auto CatmullRomSpline::getAltitudeRange() const -> std::pair<double, double>
{
  if (control_points.empty()) {
    THROW_SIMULATION_ERROR("Control points are missing.");
  } else if (const auto [min_z_it, max_z_it] = std::minmax_element(
               control_points.begin(), control_points.end(),
               [](const auto & lhs, const auto & rhs) { return lhs.z < rhs.z; });
             min_z_it == control_points.end() || max_z_it == control_points.end()) {
    THROW_SIMULATION_ERROR("Control points are invalid.");
  } else {
    return {min_z_it->z, max_z_it->z};
  }
}

auto CatmullRomSpline::getMaximum2DCurvature() const -> double
{
  if (curves_.empty()) {
    THROW_SEMANTIC_ERROR(
      "Curves are empty. We cannot determine the maximum 2D curvature of the spline.",
      "This message is not originally intended to be displayed, if you see it, please contact "
      "the developer of traffic_simulator.");
  }
  /// @note Maximum 2D curvature is empty means that it is not calculated yet.
  if (maximum_2d_curvatures_.empty()) {
    for (const auto & curve : curves_) {
      maximum_2d_curvatures_.emplace_back(curve.getMaximum2DCurvature());
    }
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
      const auto [index, s_value] = getCurveIndexAndS(s);
      return curves_[index].getNormalVector(s_value, true);
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
      /**
       * @note The current implementation uses `index_and_s` instead of structured binding
       * (`const auto [index, s_value] = getCurveIndexAndS(s)`) because some tests fail
       * when using structured binding. The root cause of these test failures is under investigation.
       */
      // const auto [index, s_value] = getCurveIndexAndS(s);
      // return curves_[index].getTangentVector(s_value, true);

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
      const auto [index, s_value] = getCurveIndexAndS(s);
      return curves_[index].getPose(s_value, true, fill_pitch);
  }
}

auto CatmullRomSpline::checkConnection() const -> bool
{
  if (control_points.size() != (curves_.size() + 1)) {
    THROW_SIMULATION_ERROR(                                    // LCOV_EXCL_LINE
      "number of control points and curves does not match.");  // LCOV_EXCL_LINE
  }
  for (std::size_t i = 0; i < curves_.size(); i++) {
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
