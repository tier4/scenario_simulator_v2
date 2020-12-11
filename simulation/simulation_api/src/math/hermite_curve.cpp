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

#include <simulation_api/math/hermite_curve.hpp>

#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <limits>

namespace simulation_api
{
namespace math
{
HermiteCurve::HermiteCurve(
  double ax, double bx, double cx, double dx,
  double ay, double by, double cy, double dy,
  double az, double bz, double cz, double dz)
: ax_(ax), bx_(bx), cx_(cx), dx_(dx),
  ay_(ay), by_(by), cy_(cy), dy_(dy),
  az_(az), bz_(bz), cz_(cz), dz_(dz)
{}

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
}

double HermiteCurve::getSquaredDistanceIn2D(
  geometry_msgs::msg::Point point, double s,
  bool autoscale) const
{
  const auto point_on_curve = getPoint(s, autoscale);
  double x_term = std::pow(point.x - point_on_curve.x, 2);
  double y_term = std::pow(point.y - point_on_curve.y, 2);
  double ret = x_term + y_term;
  return ret;
}

double HermiteCurve::getNewtonMethodStepSize(
  geometry_msgs::msg::Point point, double s,
  bool autoscale) const
{
  if (autoscale) {
    s = s / getLength();
  }
  const auto point_on_curve = getPoint(s, autoscale);
  double s2 = std::pow(s, 2);
  double x_term_diff = 2 * (point.x - point_on_curve.x) *
    (-3 * ax_ * s2 - 2 * bx_ * s - cx_);
  double y_term_diff = 2 * (point.y - point_on_curve.y) *
    (-3 * ay_ * s2 - 2 * by_ * s - cy_);
  double ret = getSquaredDistanceIn2D(point, s, autoscale) / (x_term_diff + y_term_diff);
  return ret;
}

boost::optional<double> HermiteCurve::getCollisionPointIn2D(
  std::vector<geometry_msgs::msg::Point> polygon,
  bool search_backward
) const
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
  if (s_values.size() == 0) {
    return boost::none;
  }
  if (search_backward) {
    return *std::max_element(s_values.begin(), s_values.end());
  }
  return *std::min_element(s_values.begin(), s_values.end());
}

boost::optional<double> HermiteCurve::getCollisionPointIn2D(
  geometry_msgs::msg::Point point0,
  geometry_msgs::msg::Point point1,
  bool search_backward
) const
{
  std::vector<double> s_values;
  double l = std::hypot(point0.x - point1.x, point0.y - point1.y);
  double fx = point0.x;
  double ex = (point0.x - point1.x) / l;
  double fy = point0.y;
  double ey = (point0.y - point1.y) / l;
  constexpr double e = std::numeric_limits<double>::epsilon();
  if (std::abs(point0.x - point1.x) <= e) {
    if (std::abs(point0.y - point1.y) <= e) {
      return boost::none;
    }
    auto solutions = solveCubicEquation(ax_, bx_, cx_, dx_ - fx);
    for (const auto solution : solutions) {
      if (std::fabs(cubicFunction(ay_, by_, cy_, dy_ - fy, solution) < e)) {
        s_values.emplace_back(solution);
      }
    }
  } else if (std::abs(point0.y - point1.y) <= e) {
    auto solutions = solveCubicEquation(ay_, by_, cy_, dy_ - fy);
    for (const auto solution : solutions) {
      if (std::fabs(cubicFunction(ax_, bx_, cx_, dx_ - fy, solution) < e)) {
        s_values.emplace_back(solution);
      }
    }
  } else {
    double ratio = ey / ex;
    double a = ax_ * ratio - ay_;
    double b = bx_ * ratio - by_;
    double c = cx_ * ratio - cy_;
    double d = (dx_ - fx) * ratio - (dy_ - fy);
    auto solutions = solveCubicEquation(a, b, c, d);
    for (const auto solution : solutions) {
      if (std::fabs(cubicFunction(ax_, bx_, cx_, dx_ - fy, solution) < e)) {
        s_values.emplace_back(solution);
      }
    }
  }
  if (s_values.size() == 0) {
    return boost::none;
  }
  if (search_backward) {
    return *std::max_element(s_values.begin(), s_values.end());
  }
  return *std::min_element(s_values.begin(), s_values.end());
}

/**
  * @brief solve linear equation a*x + b = 0
  *
  * @param a
  * @param b
  * @return std::vector<double> real root of the quadratic functions (from 0 to 1)
  */
std::vector<double> HermiteCurve::solveLinearEquation(double a, double b) const
{
  constexpr double e = std::numeric_limits<float>::epsilon();
  if (std::fabs(a) < e) {
    if (std::fabs(b) < e) {
      return {0};
    }
    return {};
  }
  double ret = -b / a;
  if (0 <= ret && ret <= 1) {
    std::cout << "result = " << ret << std::endl;
    return {ret};
  }
  return {};
}

std::vector<double> HermiteCurve::solveQuadraticEquation(double a, double b, double c) const
{
  std::vector<double> candidates, ret;
  constexpr double e = std::numeric_limits<float>::epsilon();
  if (std::fabs(a) < e) {
    return solveLinearEquation(b, c);
  }
  double root = b * b - 4 * a * c;
  if (std::fabs(root) < e) {
    candidates = {-b / (2 * a)};
  } else if (root < 0) {
    candidates = {};
  } else {
    candidates = {(-b - root) / (2 * a), (-b + root) / (2 * a)};
  }
  for (const auto candidate : candidates) {
    if (0 <= candidate && candidate <= 1) {
      ret.emplace_back(candidate);
    }
  }
  return ret;
}

std::vector<double> HermiteCurve::solveCubicEquation(double a, double b, double c, double d) const
{
  constexpr double e = std::numeric_limits<float>::epsilon();
  if (std::fabs(a) < e) {
    return solveQuadraticEquation(b, c, d);
  }
  std::vector<double> solutions, candidates, ret;
  auto result = solveP3(solutions, b / a, c / a, d / a);
  if (result == 3) {
    candidates = solutions;
  } else if (result == 2) {
    candidates = {solutions[0], solutions[1]};
  } else if (result == 1) {
    candidates = {solutions[0]};
  }
  for (const auto candidate : candidates) {
    if (0 <= candidate && candidate <= 1) {
      ret.emplace_back(candidate);
    }
  }
  return ret;
}

int HermiteCurve::solveP3(std::vector<double> & x, double a, double b, double c) const
{
  x = std::vector<double>(3);
  const double eps = 1e-14;
  double a2 = a * a;
  double q = (a2 - 3 * b) / 9;
  double r = (a * (2 * a2 - 9 * b) + 27 * c) / 54;
  // equation x^3 + q*x + r = 0
  double r2 = r * r;
  double q3 = q * q * q;
  double A, B;
  if (r2 <= (q3 + eps)) {      //<<-- FIXED!
    double t = r / sqrt(q3);
    if (t < -1) {t = -1;}
    if (t > 1) {t = 1;}
    t = acos(t);
    a /= 3; q = -2 * sqrt(q);
    x[0] = q * cos(t / 3) - a;
    x[1] = q * cos((t + M_PI * 2) / 3) - a;
    x[2] = q * cos((t - M_PI * 2) / 3) - a;
    return 3;
  } else {
    // A =-pow(fabs(r)+sqrt(r2-q3),1./3);
    A = -root3(fabs(r) + sqrt(r2 - q3));
    if (r < 0) {A = -A;}
    if (A == 0) {
      B = 0;
    } else {
      B = q / A;
    }
    a /= 3;
    x[0] = (A + B) - a;
    x[1] = -0.5 * (A + B) - a;
    x[2] = 0.5 * sqrt(3.) * (A - B);
    if (fabs(x[2]) < eps) {
      x[2] = x[1];
      return 2;
    }
    return 1;
  }
}

boost::optional<double> HermiteCurve::getSValue(
  geometry_msgs::msg::Point point,
  double threadhold_distance,
  unsigned int initial_resolution,
  unsigned int max_iteration,
  double torelance,
  bool autoscale) const
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
    if (std::fabs(error) < (torelance * torelance)) {
      return ret;
    }
    s_values.push_back(ret);
    errors.push_back(error);
    ret = ret - getNewtonMethodStepSize(point, ret);
  }
  std::vector<double>::iterator min_iter = std::min_element(errors.begin(), errors.end());
  double min_error = *std::min_element(errors.begin(), errors.end());
  if (min_error > (threadhold_distance * threadhold_distance)) {
    return boost::none;
  }
  size_t value_index = std::distance(errors.begin(), min_iter);
  ret = s_values[value_index];
  if (ret < 0) {
    double error = getSquaredDistanceIn2D(point, 0, false);
    if (error < (threadhold_distance * threadhold_distance)) {
      ret = 0;
    } else {
      return boost::none;
    }
  }
  if (ret > 1) {
    double error = getSquaredDistanceIn2D(point, 0, false);
    if (error < (threadhold_distance * threadhold_distance)) {
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

std::vector<geometry_msgs::msg::Point> HermiteCurve::getTrajectory() const
{
  std::vector<geometry_msgs::msg::Point> ret;
  for (int i = 0; i <= 100; i++) {
    double t = static_cast<double>(i) / 100.0;
    geometry_msgs::msg::Point p = getPoint(t);
    ret.push_back(p);
  }
  return ret;
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

double HermiteCurve::getMaximu2DCurvature() const
{
  std::vector<double> curvatures;
  for (double s = 0; s <= 1; s = s + 0.01) {
    double curvature = get2DCurvature(s);
    curvatures.push_back(curvature);
  }
  return *std::max_element(curvatures.begin(), curvatures.end());
}

double HermiteCurve::getLength() const
{
  auto trajectory = getTrajectory();
  double ret = 0.0;
  for (size_t i = 0; i < trajectory.size() - 1; i++) {
    ret = ret + std::sqrt(std::pow(trajectory[i + 1].x - trajectory[i].x, 2) +
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
}  // namespace simulation_api
