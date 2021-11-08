// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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
#include <iostream>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <traffic_simulator/math/hermite_curve.hpp>
#include <vector>

namespace traffic_simulator
{
namespace math
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
  geometry_msgs::msg::Point point, double s, bool autoscale) const
{
  const auto point_on_curve = getPoint(s, autoscale);
  double x_term = std::pow(point.x - point_on_curve.x, 2);
  double y_term = std::pow(point.y - point_on_curve.y, 2);
  double ret = x_term + y_term;
  return ret;
}

double HermiteCurve::getNewtonMethodStepSize(
  geometry_msgs::msg::Point point, double s /*, bool autoscale*/) const
{
  /*
  if (autoscale) {
    s = s / getLength();
  }
  */
  const auto point_on_curve = getPoint(s, false /*autoscale*/);
  double s2 = std::pow(s, 2);
  double x_term_diff = 2 * (point.x - point_on_curve.x) * (-3 * ax_ * s2 - 2 * bx_ * s - cx_);
  double y_term_diff = 2 * (point.y - point_on_curve.y) * (-3 * ay_ * s2 - 2 * by_ * s - cy_);
  double ret = getSquaredDistanceIn2D(point, s, false /*autoscale*/) / (x_term_diff + y_term_diff);
  return ret;
}

boost::optional<double> HermiteCurve::getCollisionPointIn2D(
  const std::vector<geometry_msgs::msg::Point> & polygon, bool search_backward,
  bool close_start_end) const
{
  size_t n = polygon.size();
  if (n <= 1) {
    return boost::none;
  }
  std::vector<double> s_values;
  for (size_t i = 0; i < (n - 1); i++) {
    const auto p0 = polygon[i];
    const auto p1 = polygon[i + 1];
    auto s = getCollisionPointIn2D(p0, p1, search_backward);
    if (s) {
      s_values.emplace_back(s.get());
    }
  }
  if (close_start_end) {
    const auto p0 = polygon[n - 1];
    const auto p1 = polygon[0];
    auto s = getCollisionPointIn2D(p0, p1, search_backward);
    if (s) {
      s_values.emplace_back(s.get());
    }
  }
  if (s_values.empty()) {
    return boost::none;
  }
  if (search_backward) {
    return *std::max_element(s_values.begin(), s_values.end());
  }
  return *std::min_element(s_values.begin(), s_values.end());
}

boost::optional<double> HermiteCurve::getCollisionPointIn2D(
  const geometry_msgs::msg::Point & point0, const geometry_msgs::msg::Point & point1,
  bool search_backward) const
{
  std::vector<double> s_values;
  double fx = point0.x;
  double ex = (point1.x - point0.x);
  double fy = point0.y;
  double ey = (point1.y - point0.y);
  double a = ay_ * ex - ax_ * ey;
  double b = by_ * ex - bx_ * ey;
  double c = cy_ * ex - cx_ * ey;
  double d = dy_ * ex - dx_ * ey - ex * fy + ey * fx;
  auto solutions = solver_.solveCubicEquation(a, b, c, d);
  for (const auto solution : solutions) {
    double x = solver_.cubicFunction(ax_, bx_, cx_, dx_, solution);
    double tx = (x - point0.x) / (point1.x - point0.x);
    double y = solver_.cubicFunction(ay_, by_, cy_, dy_, solution);
    double ty = (y - point0.y) / (point1.y - point0.y);
    if (0 > tx || tx > 1) {
      continue;
    }
    if (0 > ty || ty > 1) {
      continue;
    }
    if (0 > solution || solution > 1) {
      continue;
    }
    s_values.emplace_back(solution);
  }
  if (s_values.empty()) {
    return boost::none;
  }
  if (search_backward) {
    return *std::max_element(s_values.begin(), s_values.end());
  }
  return *std::min_element(s_values.begin(), s_values.end());
}

boost::optional<double> HermiteCurve::getSValue(
  geometry_msgs::msg::Point point, double threshold_distance, unsigned int initial_resolution,
  unsigned int max_iteration, double tolerance, bool autoscale) const
{
  double step_size = static_cast<double>(1.0) / static_cast<double>(initial_resolution);
  double ret = 0.0;
  std::vector<double> initial_value_candidates(initial_resolution);
  std::vector<double> initial_errors(initial_resolution);
  for (unsigned int i = 0; i < initial_resolution; i++) {
    initial_value_candidates[i] = (0.5 + static_cast<double>(i)) * step_size;
    initial_errors[i] = std::fabs(getSquaredDistanceIn2D(point, initial_value_candidates[i]));
  }
  std::vector<double>::iterator iter =
    std::min_element(initial_errors.begin(), initial_errors.end());
  size_t index = std::distance(initial_errors.begin(), iter);
  ret = initial_value_candidates[index];
  std::vector<double> errors;
  std::vector<double> s_values;
  for (unsigned i = 0; i < max_iteration; i++) {
    double error = getSquaredDistanceIn2D(point, ret);
    if (std::fabs(error) < (tolerance * tolerance)) {
      return ret;
    }
    s_values.push_back(ret);
    errors.push_back(error);
    ret = ret - getNewtonMethodStepSize(point, ret);
  }
  std::vector<double>::iterator min_iter = std::min_element(errors.begin(), errors.end());
  double min_error = *std::min_element(errors.begin(), errors.end());
  if (min_error > (threshold_distance * threshold_distance)) {
    return boost::none;
  }
  size_t value_index = std::distance(errors.begin(), min_iter);
  ret = s_values[value_index];
  if (ret < 0) {
    double error = getSquaredDistanceIn2D(point, 0, false);
    if (error < (threshold_distance * threshold_distance)) {
      ret = 0;
    } else {
      return boost::none;
    }
  }
  if (ret > 1) {
    double error = getSquaredDistanceIn2D(point, 0, false);
    if (error < (threshold_distance * threshold_distance)) {
      ret = 1;
    } else {
      return boost::none;
    }
  }
  if (autoscale) {
    ret = ret * getLength();
  }
  return ret;
}

const std::vector<geometry_msgs::msg::Point> HermiteCurve::getTrajectory(
  double start_s, double end_s, double resolution, bool autoscale) const
{
  resolution = std::fabs(resolution);
  if (start_s <= end_s) {
    std::vector<geometry_msgs::msg::Point> ret;
    double s = start_s;
    while (s <= end_s) {
      s = s + resolution;
      ret.emplace_back(getPoint(s, autoscale));
    }
    return ret;
  } else {
    std::vector<geometry_msgs::msg::Point> ret;
    double s = start_s;
    while (s >= end_s) {
      s = s - resolution;
      ret.emplace_back(getPoint(s, autoscale));
    }
    return ret;
  }
}

std::vector<geometry_msgs::msg::Point> HermiteCurve::getTrajectory(size_t num_points) const
{
  std::vector<geometry_msgs::msg::Point> ret;
  for (size_t i = 0; i <= num_points; i++) {
    double t = static_cast<double>(i) / 100.0;
    ret.emplace_back(getPoint(t, false));
  }
  return ret;
}

const geometry_msgs::msg::Vector3 HermiteCurve::getNormalVector(double s, bool autoscale) const
{
  if (autoscale) {
    s = s / getLength();
  }
  geometry_msgs::msg::Vector3 tangent_vec = getTangentVector(s);
  double theta = M_PI / 2.0;
  geometry_msgs::msg::Vector3 vec;
  vec.x = tangent_vec.x * std::cos(theta) - tangent_vec.y * std::sin(theta);
  vec.y = tangent_vec.x * std::sin(theta) + tangent_vec.y * std::cos(theta);
  return vec;
}

const geometry_msgs::msg::Vector3 HermiteCurve::getTangentVector(double s, bool autoscale) const
{
  if (autoscale) {
    s = s / getLength();
  }
  geometry_msgs::msg::Vector3 vec;
  vec.x = 3 * ax_ * s * s + 2 * bx_ * s + cx_;
  vec.y = 3 * ay_ * s * s + 2 * by_ * s + cy_;
  vec.z = 3 * az_ * s * s + 2 * bz_ * s + cz_;
  return vec;
}

const geometry_msgs::msg::Pose HermiteCurve::getPose(double s, bool autoscale) const
{
  if (autoscale) {
    s = s / getLength();
  }
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Vector3 tangent_vec = getTangentVector(s, false);
  geometry_msgs::msg::Vector3 rpy;
  rpy.x = 0.0;
  rpy.y = 0.0;
  rpy.z = std::atan2(tangent_vec.y, tangent_vec.x);
  pose.orientation = quaternion_operation::convertEulerAngleToQuaternion(rpy);
  pose.position = getPoint(s);
  return pose;
}

double HermiteCurve::get2DCurvature(double s, bool autoscale) const
{
  if (autoscale) {
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

double HermiteCurve::getLength(size_t num_points) const
{
  auto trajectory = getTrajectory(num_points);
  double ret = 0.0;
  for (size_t i = 0; i < trajectory.size() - 1; i++) {
    ret = ret + std::sqrt(
                  std::pow(trajectory[i + 1].x - trajectory[i].x, 2) +
                  std::pow(trajectory[i + 1].y - trajectory[i].y, 2) +
                  std::pow(trajectory[i + 1].z - trajectory[i].z, 2));
  }
  return ret;
}

const geometry_msgs::msg::Point HermiteCurve::getPoint(double s, bool autoscale) const
{
  if (autoscale) {
    s = s / getLength();
  }
  geometry_msgs::msg::Point p;
  p.x = ax_ * std::pow(s, 3) + bx_ * std::pow(s, 2) + cx_ * s + dx_;
  p.y = ay_ * std::pow(s, 3) + by_ * std::pow(s, 2) + cy_ * s + dy_;
  p.z = az_ * std::pow(s, 3) + bz_ * std::pow(s, 2) + cz_ * s + dz_;
  return p;
}
}  // namespace math
}  // namespace traffic_simulator
