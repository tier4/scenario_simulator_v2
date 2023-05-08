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

#include <cmath>
#include <geometry/solver/polynomial_solver.hpp>
#include <iostream>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <scenario_simulator_exception/exception.hpp>
#include <vector>

namespace math
{
namespace geometry
{
double PolynomialSolver::linearFunction(double a, double b, double t) const { return a * t + b; }

double PolynomialSolver::cubicFunction(double a, double b, double c, double d, double t) const
{
  return a * t * t * t + b * t * t + c * t + d;
}

double PolynomialSolver::quadraticFunction(double a, double b, double c, double t) const
{
  return a * t * t + b * t + c;
}

std::vector<double> PolynomialSolver::solveLinearEquation(
  double a, double b, double min_value, double max_value) const
{
  if (std::abs(a) <= tolerance) {
    THROW_SIMULATION_ERROR(
      "Not computable because a=0 in the linear equation ax+b=0, "
      "so any value of x = ",
      min_value, "~", max_value, " will be the solution.",
      "There are no expected cases where this exception is thrown.",
      "Please contact the scenario_simulator_v2 developers, ",
      "especially Masaya Kataoka (@hakuturu583).");
  }
  double ret = -b / a;
  if (min_value <= ret && ret <= max_value) {
    return {ret};
  } else if (std::abs(ret - max_value) <= tolerance) {
    return {max_value};
  } else if (std::abs(ret - min_value) <= tolerance) {
    return {min_value};
  }
  return {};
}

std::vector<double> PolynomialSolver::solveQuadraticEquation(
  double a, double b, double c, double min_value, double max_value) const
{
  std::vector<double> candidates, ret;
  if (std::abs(a) <= tolerance) {
    return solveLinearEquation(b, c);
  }
  double root = b * b - 4 * a * c;
  if (std::abs(root) <= tolerance) {
    candidates = {-b / (2 * a)};
  } else if (root < 0) {
    candidates = {};
  } else {
    candidates = {(-b - std::sqrt(root)) / (2 * a), (-b + std::sqrt(root)) / (2 * a)};
  }
  for (const auto candidate : candidates) {
    if (min_value <= candidate && candidate <= max_value) {
      ret.push_back(candidate);
    } else if (std::abs(candidate - max_value) <= tolerance) {
      ret.push_back(max_value);
    } else if (std::abs(candidate - min_value) <= tolerance) {
      ret.push_back(min_value);
    }
  }
  return ret;
}

std::vector<double> PolynomialSolver::solveCubicEquation(
  double a, double b, double c, double d, double min_value, double max_value) const
{
  if (std::abs(a) <= tolerance) {
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
    if (min_value <= candidate && candidate <= max_value) {
      ret.push_back(candidate);
    } else if (std::abs(candidate - max_value) <= tolerance) {
      ret.push_back(max_value);
    } else if (std::abs(candidate - min_value) <= tolerance) {
      ret.push_back(min_value);
    }
  }
  return ret;
}

int PolynomialSolver::solveP3(std::vector<double> & x, double a, double b, double c) const
{
  x = std::vector<double>(3);
  double a2 = a * a;
  double q = (a2 - 3 * b) / 9;
  double r = (a * (2 * a2 - 9 * b) + 27 * c) / 54;
  // equation x^3 + q*x + r = 0
  double r2 = r * r;
  double q3 = q * q * q;
  double A, B;
  if (r2 <= (q3 + tolerance)) {
    double t = r / sqrt(q3);
    if (t < -1) {
      t = -1;
    }
    if (t > 1) {
      t = 1;
    }
    t = acos(t);
    a /= 3;
    q = -2 * sqrt(q);
    x[0] = q * cos(t / 3) - a;
    x[1] = q * cos((t + M_PI * 2) / 3) - a;
    x[2] = q * cos((t - M_PI * 2) / 3) - a;
    return 3;
  } else {
    A = -root3(std::abs(r) + sqrt(r2 - q3));
    if (r < 0) {
      A = -A;
    }
    if (A == 0) {
      B = 0;
    } else {
      B = q / A;
    }
    a /= 3;
    x[0] = (A + B) - a;
    x[1] = -0.5 * (A + B) - a;
    x[2] = 0.5 * sqrt(3.) * (A - B);
    if (std::abs(x[2]) <= tolerance) {
      x[2] = x[1];
      return 2;
    }
    return 1;
  }
  return 0;
}

double PolynomialSolver::_root3(double x) const
{
  double s = 1.;
  while (x < 1.) {
    x *= 8.;
    s *= 0.5;
  }
  while (x > 8.) {
    x *= 0.125;
    s *= 2.;
  }
  double r = 1.5;
  r -= 1. / 3. * (r - x / (r * r));
  r -= 1. / 3. * (r - x / (r * r));
  r -= 1. / 3. * (r - x / (r * r));
  r -= 1. / 3. * (r - x / (r * r));
  r -= 1. / 3. * (r - x / (r * r));
  r -= 1. / 3. * (r - x / (r * r));
  return r * s;
}

double PolynomialSolver::root3(double x) const
{
  if (x > 0) {
    return _root3(x);
  } else if (x < 0) {
    return -_root3(-x);
  } else {
    return 0.;
  }
}
}  // namespace geometry
}  // namespace math
