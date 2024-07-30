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

#include <algorithm>
#include <cmath>
#include <geometry/bounding_box.hpp>
#include <geometry/quaternion/euler_to_quaternion.hpp>
#include <geometry/spline/hermite_curve.hpp>
#include <iostream>
#include <limits>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <vector>

namespace math
{
namespace geometry
{
HermiteCurve::HermiteCurve(
  double ax, double bx, double cx, double dx, double ay, double by, double cy, double dy, double az,
  double bz, double cz, double dz)
: ax_(ax),
  bx_(bx),
  cx_(cx),
  dx_(dx),
  ay_(ay),
  by_(by),
  cy_(cy),
  dy_(dy),
  az_(az),
  bz_(bz),
  cz_(cz),
  dz_(dz),
  length_(getLength(100))
{
}

HermiteCurve::HermiteCurve(
  geometry_msgs::msg::Pose start_pose, geometry_msgs::msg::Pose goal_pose,
  geometry_msgs::msg::Vector3 start_vec, geometry_msgs::msg::Vector3 goal_vec)
{
  ax_ = 2 * start_pose.position.x - 2 * goal_pose.position.x + start_vec.x + goal_vec.x;
  bx_ = -3 * start_pose.position.x + 3 * goal_pose.position.x - 2 * start_vec.x - goal_vec.x;
  cx_ = start_vec.x;
  dx_ = start_pose.position.x;

  ay_ = 2 * start_pose.position.y - 2 * goal_pose.position.y + start_vec.y + goal_vec.y;
  by_ = -3 * start_pose.position.y + 3 * goal_pose.position.y - 2 * start_vec.y - goal_vec.y;
  cy_ = start_vec.y;
  dy_ = start_pose.position.y;

  az_ = 2 * start_pose.position.z - 2 * goal_pose.position.z + start_vec.z + goal_vec.z;
  bz_ = -3 * start_pose.position.z + 3 * goal_pose.position.z - 2 * start_vec.z - goal_vec.z;
  cz_ = start_vec.z;
  dz_ = start_pose.position.z;
  length_ = getLength(100);
}

double HermiteCurve::getSquaredDistanceIn2D(
  const geometry_msgs::msg::Point & point, double s, bool denormalize_s) const
{
  const auto point_on_curve = getPoint(s, denormalize_s);
  return std::pow(point.x - point_on_curve.x, 2) + std::pow(point.y - point_on_curve.y, 2);
}

geometry_msgs::msg::Vector3 HermiteCurve::getSquaredDistanceVector(
  const geometry_msgs::msg::Point & point, double s, bool denormalize_s) const
{
  const auto point_on_curve = getPoint(s, denormalize_s);
  geometry_msgs::msg::Vector3 ret;
  ret.x = point.x - point_on_curve.x;
  ret.y = point.y - point_on_curve.y;
  ret.z = point.z - point_on_curve.z;
  return ret;
}

std::set<double> HermiteCurve::getCollisionPointsIn2D(
  const std::vector<geometry_msgs::msg::Point> & polygon, bool search_backward,
  bool close_start_end, bool denormalize_s) const
{
  size_t n = polygon.size();
  if (n <= 1) {
    return {};
  }
  std::set<double> s_values;
  for (size_t i = 0; i < (n - 1); i++) {
    const auto p0 = polygon[i];
    const auto p1 = polygon[i + 1];
    s_values.merge(getCollisionPointsIn2D(p0, p1, search_backward, denormalize_s));
  }
  if (close_start_end) {
    const auto p0 = polygon[n - 1];
    const auto p1 = polygon[0];
    s_values.merge(getCollisionPointsIn2D(p0, p1, search_backward, denormalize_s));
  }
  return s_values;
}

std::optional<double> HermiteCurve::getCollisionPointIn2D(
  const std::vector<geometry_msgs::msg::Point> & polygon, bool search_backward,
  bool close_start_end, bool denormalize_s) const
{
  auto s_values = getCollisionPointsIn2D(polygon, search_backward, close_start_end, denormalize_s);
  if (s_values.empty()) {
    return std::nullopt;
  }
  if (search_backward) {
    return *s_values.rbegin();
  }
  return *s_values.begin();
}

std::set<double> HermiteCurve::getCollisionPointsIn2D(
  const geometry_msgs::msg::Point & point0, const geometry_msgs::msg::Point & point1,
  bool search_backward, bool denormalize_s) const
{
  std::set<double> s_values;
  double fx = point0.x;
  double ex = (point1.x - point0.x);
  double fy = point0.y;
  double ey = (point1.y - point0.y);
  double a = ay_ * ex - ax_ * ey;
  double b = by_ * ex - bx_ * ey;
  double c = cy_ * ex - cx_ * ey;
  double d = dy_ * ex - dx_ * ey - ex * fy + ey * fx;

  const auto get_solutions = [search_backward, a, b, c, d, this]() -> std::vector<double> {
    try {
      /**
       * @note Obtain a solution to the cubic equation ax^3 + bx^2 + cx + d = 0 that falls within the range [0, 1].
       */
      return solver_.solveCubicEquation(a, b, c, d, 0, 1);
    }
    /**
     * @note PolynomialSolver::solveCubicEquation throws common::SimulationError when any x value can satisfy the equation, 
     * so the beginning and end point of this curve can collide with the line segment.
     * If search_backward = true, the line segment collisions at the end of the curve. So return 1.
     * If search_backward = false, the line segment collisions at the start of the curve. So return 0.
     */
    catch (const common::SimulationError &) {
      return {search_backward ? 1.0 : 0.0};
    }
  };

  /**
   * @note Denormalize given S value as necessary
   */
  const auto denormalize = [denormalize_s, this](double s) -> double {
    if (denormalize_s) {
      return s * getLength();
    }
    return s;
  };

  for (const auto solution : get_solutions()) {
    constexpr double epsilon = std::numeric_limits<double>::epsilon();
    double x = solver_.cubic(ax_, bx_, cx_, dx_, solution);
    double tx = (x - point0.x) / (point1.x - point0.x);
    double y = solver_.cubic(ay_, by_, cy_, dy_, solution);
    double ty = (y - point0.y) / (point1.y - point0.y);
    if (std::abs(tx - ty) > epsilon || std::isnan(tx) || std::isnan(ty)) {
      /**
       * @note If the curve and the line segment to be intersected are parallel to either of the x/y axes, one of the two parameters, 
       * tx, ty, will be in the range [0, 1] while the other will be out of that range because of division by zero.
       */
      if ((0 <= tx && tx <= 1) || (0 <= ty && ty <= 1)) {
        s_values.insert(denormalize(solution));
      }
    } else {
      if ((0 <= tx && tx <= 1) && (0 <= ty && ty <= 1)) {
        s_values.insert(denormalize(solution));
      }
    }
  }
  return s_values;
}

std::optional<double> HermiteCurve::getCollisionPointIn2D(
  const geometry_msgs::msg::Point & point0, const geometry_msgs::msg::Point & point1,
  bool search_backward, bool denormalize_s) const
{
  auto s_values = getCollisionPointsIn2D(point0, point1, search_backward, denormalize_s);
  if (s_values.empty()) {
    return std::nullopt;
  }
  if (search_backward) {
    return *s_values.rbegin();
  }
  return *s_values.begin();
}

std::optional<double> HermiteCurve::getSValue(
  const geometry_msgs::msg::Pose & pose, double threshold_distance, bool denormalize_s) const
{
  geometry_msgs::msg::Point p0, p1;
  p0.y = threshold_distance;
  p1.y = -threshold_distance;
  const auto line = math::geometry::transformPoints(pose, {p0, p1});
  const auto s = getCollisionPointIn2D(line[0], line[1], false);
  if (!s) {
    return std::nullopt;
  }
  if (denormalize_s) {
    return s.value() * getLength();
  }
  return s.value();
}

const std::vector<geometry_msgs::msg::Point> HermiteCurve::getTrajectory(
  double start_s, double end_s, double resolution, bool denormalize_s) const
{
  resolution = std::fabs(resolution);
  if (start_s <= end_s) {
    std::vector<geometry_msgs::msg::Point> ret;
    double s = start_s;
    while (s <= end_s) {
      s = s + resolution;
      ret.emplace_back(getPoint(s, denormalize_s));
    }
    return ret;
  } else {
    std::vector<geometry_msgs::msg::Point> ret;
    double s = start_s;
    while (s >= end_s) {
      s = s - resolution;
      ret.emplace_back(getPoint(s, denormalize_s));
    }
    return ret;
  }
}

std::vector<geometry_msgs::msg::Point> HermiteCurve::getTrajectory(size_t num_points) const
{
  std::vector<geometry_msgs::msg::Point> ret;
  if (num_points == 1) {  // safe check to not divide by zero in the loop
    ret.emplace_back(getPoint(0.0, false));
    return ret;
  }
  for (size_t i = 0; i < num_points; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(num_points - 1);
    ret.emplace_back(getPoint(t, false));
  }
  return ret;
}

const geometry_msgs::msg::Vector3 HermiteCurve::getNormalVector(double s, bool denormalize_s) const
{
  if (denormalize_s) {
    s = s / getLength();
  }
  geometry_msgs::msg::Vector3 tangent_vec = getTangentVector(s);
  double theta = M_PI / 2.0;
  geometry_msgs::msg::Vector3 vec;
  vec.x = tangent_vec.x * std::cos(theta) - tangent_vec.y * std::sin(theta);
  vec.y = tangent_vec.x * std::sin(theta) + tangent_vec.y * std::cos(theta);
  return vec;
}

const geometry_msgs::msg::Vector3 HermiteCurve::getTangentVector(double s, bool denormalize_s) const
{
  if (denormalize_s) {
    s = s / getLength();
  }
  geometry_msgs::msg::Vector3 vec;
  vec.x = 3 * ax_ * s * s + 2 * bx_ * s + cx_;
  vec.y = 3 * ay_ * s * s + 2 * by_ * s + cy_;
  vec.z = 3 * az_ * s * s + 2 * bz_ * s + cz_;
  return vec;
}

/// @note fill_pitch is introduced for backward-compatibility.
const geometry_msgs::msg::Pose HermiteCurve::getPose(
  double s, bool denormalize_s, bool fill_pitch) const
{
  if (denormalize_s) {
    s = s / getLength();
  }
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Vector3 tangent_vec = getTangentVector(s, false);
  geometry_msgs::msg::Vector3 rpy;
  rpy.x = 0.0;
  rpy.y = fill_pitch ? std::atan2(-tangent_vec.z, std::hypot(tangent_vec.x, tangent_vec.y)) : 0.0;
  rpy.z = std::atan2(tangent_vec.y, tangent_vec.x);
  pose.orientation = math::geometry::convertEulerAngleToQuaternion(rpy);
  pose.position = getPoint(s);
  return pose;
}

double HermiteCurve::get2DCurvature(double s, bool denormalize_s) const
{
  if (denormalize_s) {
    s = s / getLength();
  }
  double s2 = s * s;
  double x_dot = 3 * ax_ * s2 + 2 * bx_ * s + cx_;
  double x_dot_dot = 6 * ax_ * s + 2 * bx_;
  double y_dot = 3 * ay_ * s2 + 2 * by_ * s + cy_;
  double y_dot_dot = 6 * ay_ * s + 2 * by_;
  return (x_dot * y_dot_dot - x_dot_dot * y_dot) / std::pow(x_dot * x_dot + y_dot * y_dot, 1.5);
}

std::pair<double, double> HermiteCurve::get2DMinMaxCurvatureValue() const
{
  std::pair<double, double> ret;
  std::vector<double> curvatures;
  /**
   * @brief 0.1 is a sampling resolution of the curvature
   */
  for (double s = 0; s <= 1; s = s + 0.1) {
    double curvature = get2DCurvature(s);
    curvatures.push_back(curvature);
  }
  ret.first = *std::min_element(curvatures.begin(), curvatures.end());
  ret.second = *std::max_element(curvatures.begin(), curvatures.end());
  return ret;
}

double HermiteCurve::getMaximum2DCurvature() const
{
  const auto values = get2DMinMaxCurvatureValue();
  if (std::fabs(values.first) > std::fabs(values.second)) {
    return values.first;
  }
  return values.second;
}

/**
 * @brief get length of the hermite curve. Calculate distance of two points on hermite curve and accumulate it's distance
 * @param num_points
 * @return double length
 */
double HermiteCurve::getLength(size_t num_points) const
{
  double delta_s = 1.0 / num_points;
  double ret = 0.0;
  /**
   * @brief Approximate distance of two points on hermite curve, ignore terms above the second order of delta s.
   * @image html get_length_in_hermite_curve.png
   */
  for (size_t i = 0; i < num_points; i++) {
    double s = i * delta_s;
    double x_diff = (3 * s * s) * ax_ + 2 * s * bx_ + cx_;
    double y_diff = (3 * s * s) * ay_ + 2 * s * by_ + cy_;
    double z_diff = (3 * s * s) * az_ + 2 * s * bz_ + cz_;
    ret = ret + std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff) * delta_s;
  }
  return ret;
}

const geometry_msgs::msg::Point HermiteCurve::getPoint(double s, bool denormalize_s) const
{
  if (denormalize_s) {
    s = s / getLength();
  }
  geometry_msgs::msg::Point p;

  // optimization
  auto s2 = s * s;
  auto s3 = s2 * s;

  p.x = ax_ * s3 + bx_ * s2 + cx_ * s + dx_;
  p.y = ay_ * s3 + by_ * s2 + cy_ * s + dy_;
  p.z = az_ * s3 + bz_ * s2 + cz_ * s + dz_;
  // optimization

  return p;
}
}  // namespace geometry
}  // namespace math
