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
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <geometry/solver/polynomial_solver.hpp>
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
auto PolynomialSolver::linear(const double a, const double b, const double t) const -> double
{
  return a * t + b;
}

auto PolynomialSolver::quadratic(
  const double a, const double b, const double c, const double t) const -> double
{
  return a * t * t + b * t + c;
}

auto PolynomialSolver::cubic(
  const double a, const double b, const double c, const double d, const double t) const -> double
{
  return a * t * t * t + b * t * t + c * t + d;
}

auto PolynomialSolver::solveLinearEquation(
  const double a, const double b, const double min_value, const double max_value) const
  -> std::vector<double>
{
  const auto solve_without_limit =
    [this](const double coef_a, const double coef_b) -> std::vector<double> {
    /// @note In this case, ax*b = 0 (a=0) can cause division by zero. So give special treatment to this case.
    if (isApproximatelyEqualTo(coef_a, 0)) {
      if (isApproximatelyEqualTo(coef_b, 0)) {
        THROW_SIMULATION_ERROR(
          "Not computable x because of the linear equation ", coef_a, " x + ", coef_b,
          "=0, and a = ", coef_a, ", b = ", coef_b,
          " is very close to zero ,so any value of x will be the solution.",
          "There are no expected cases where this exception is thrown.",
          "Please contact the scenario_simulator_v2 developers, ",
          "especially Masaya Kataoka (@hakuturu583).");
      }
      /// @note In this case, ax*b = 0 (a=0,b!=0) so any x cannot satisfy this equation.
      return {};
    }
    /// @note In this case, ax*b = 0 (a!=0, b!=0) so x = -b/a is a only solution.
    return {-coef_b / coef_a};
  };

  /// @note No fallback because of the order cannot be lowered any further.
  return filterByRange(solve_without_limit(a, b), min_value, max_value);
}

auto PolynomialSolver::solveQuadraticEquation(
  const double a, const double b, const double c, const double min_value,
  const double max_value) const -> std::vector<double>
{
  const auto solve_without_limit =
    [this](const double coef_a, const double coef_b, const double coef_c) -> std::vector<double> {
    if (const double discriminant = coef_b * coef_b - 4 * coef_a * coef_c;
        isApproximatelyEqualTo(discriminant, 0)) {
      return {-coef_b / (2 * coef_a)};
    } else if (discriminant < 0) {
      return {};
    } else {
      return {
        (-coef_b - std::sqrt(discriminant)) / (2 * coef_a),
        (-coef_b + std::sqrt(discriminant)) / (2 * coef_a)};
    }
  };

  /// @note Fallback to linear equation solver if a = 0
  return isApproximatelyEqualTo(a, 0)
           ? solveLinearEquation(b, c, min_value, max_value)
           : filterByRange(solve_without_limit(a, b, c), min_value, max_value);
}

auto PolynomialSolver::solveCubicEquation(
  const double a, const double b, const double c, const double d, const double min_value,
  const double max_value) const -> std::vector<double>
{
  const auto get_real_values =
    [](const std::vector<std::complex<double>> & complex_values) -> std::vector<double> {
    /**
       * @note Function that takes a complex number as input and returns the real part if it is a real number (imaginary part is 0) 
       * or std::nullopt if it is an imaginary or complex number.
       */
    const auto is_real_value = [](const std::complex<double> & complex_value) {
      constexpr double epsilon = std::numeric_limits<double>::epsilon();
      return (std::abs(complex_value.imag()) <= epsilon)
               ? std::optional<double>(complex_value.real())
               : std::nullopt;
    };
    /// @note Iterate all complex values and check the value is real value or not.
    std::vector<double> real_values = {};
    std::for_each(
      complex_values.begin(), complex_values.end(),
      [&real_values, is_real_value](const auto & complex_value) mutable {
        if (const auto real_value = is_real_value(complex_value)) {
          real_values.push_back(real_value.value());
        }
      });
    return real_values;
  };

  const auto solve_without_limit =
    [=](const double coef_a, const double coef_b, const double coef_c, const double coef_d) {
      /// @note Function that takes a std::vector of complex numbers and selects only real numbers from it and returns them

      /// @note Finds the complex solution of the monic cubic equation and returns only those that are real numbers.
      return get_real_values(
        solveMonicCubicEquationWithComplex(coef_b / coef_a, coef_c / coef_a, coef_d / coef_a));
    };

  /// @note Fallback to quadratic equation solver if a = 0
  return isApproximatelyEqualTo(a, 0)
           ? solveQuadraticEquation(b, c, d, min_value, max_value)
           : filterByRange(solve_without_limit(a, b, c, d), min_value, max_value);
}

auto PolynomialSolver::filterByRange(
  const std::vector<double> & values, const double min_value, const double max_value) const
  -> std::vector<double>
{
  /**
   * @note Function to check if value exists between [min_value,max_value] considering the tolerance,
   * returning std::nullopt if not present. If not, return std::nullopt, otherwise return value.
   */
  const auto is_in_range = [](const double value, const double min, const double max) {
    if (min <= value && value <= max) {
      return std::optional(value);
    } else if (std::abs(value - max) <= tolerance) {
      return std::optional(max);
    } else if (std::abs(value - min) <= tolerance) {
      return std::optional(min);
    }
    return std::optional<double>();
  };
  /// @note Iterate values and check the value is in range or not.
  std::vector<double> filtered_values = {};
  std::for_each(
    values.begin(), values.end(),
    [&filtered_values, is_in_range, min_value, max_value](const double value) mutable {
      if (const auto filtered_value = is_in_range(value, min_value, max_value)) {
        filtered_values.push_back(filtered_value.value());
      }
    });
  return filtered_values;
}

/// @note this code is public domain (http://math.ivanovo.ac.ru/dalgebra/Khashin/poly/index.html)
auto PolynomialSolver::solveMonicCubicEquationWithComplex(
  const double a, const double b, const double c) const -> std::vector<std::complex<double>>
{
  /**
   * @note Tschirnhaus transformation, transform into x^3 + 3q*x + 2r = 0
   * @sa https://oshima-gakushujuku.com/blog/math/formula-qubic-equation/
   */
  const auto tschirnhaus_transformation =
    [](const auto coef_a, const auto coef_b, const auto coef_c) {
      /// @note The first element of the return value is q, the second element is r
      return std::tuple<double, double>(
        (coef_a * coef_a - 3 * coef_b) / 9,
        (coef_a * (2 * coef_a * coef_a - 9 * coef_b) + 27 * coef_c) / 54);
    };

  const auto solve_without_limit =
    // clang-format off
    [this, a](const auto q, const auto r) -> std::vector<std::complex<double>> {
    // clang-format on
    if (const double q3 = q * q * q; r * r <= (q3 + tolerance)) {
      /**
       * @note If 3 real solutions are found.
       * The URL specified in @sa is a reference material for developers who wish to follow the formulas,
       * and the code that exists in the material is not included in this library.
       * @sa https://onihusube.hatenablog.com/entry/2018/10/08/140426
       */
      const double t = std::acos(std::clamp(r / std::sqrt(q3), -1.0, 1.0));
      return {
        // clang-format off
        std::complex<double>(-2 * std::sqrt(q) * std::cos( t                                             / 3) - a / 3, 0),
        std::complex<double>(-2 * std::sqrt(q) * std::cos((t + boost::math::constants::two_pi<double>()) / 3) - a / 3, 0),
        std::complex<double>(-2 * std::sqrt(q) * std::cos((t - boost::math::constants::two_pi<double>()) / 3) - a / 3, 0)
        // clang-format on
      };
    } else {
      /// @note If imaginary solutions exist.
      const double A = [r, q3]() {
        const auto calculate_real_solution = [r, q3]() {
          return -std::cbrt(std::abs(r) + std::sqrt(r * r - q3));
        };
        return r < 0 ? -1 * calculate_real_solution() : calculate_real_solution();
      }();
      const double B = isApproximatelyEqualTo(A, 0) ? 0 : q / A;
      /// @note If the imaginary part of the complex almost zero, this equation has a multiple solution.
      const double imaginary_part = 0.5 * std::sqrt(3.0) * (A - B);
      return isApproximatelyEqualTo(imaginary_part, 0)
               ? std::vector<std::complex<double>>({
                   // clang-format off
                    std::complex<double>(       (A + B) - a / 3, 0),
                    std::complex<double>(-0.5 * (A + B) - a / 3, 0)
                   // clang-format on
                 })
               : std::vector<std::complex<double>>({
                   // clang-format off
                    std::complex<double>(       (A + B) - a / 3,               0),
                    std::complex<double>(-0.5 * (A + B) - a / 3, -imaginary_part),
                    std::complex<double>(-0.5 * (A + B) - a / 3,  imaginary_part)
                   // clang-format on
                 });
    }
    /// @note No solutions are found.
    return {};
  };
  return std::apply(solve_without_limit, tschirnhaus_transformation(a, b, c));
}

auto PolynomialSolver::isApproximatelyEqualTo(const double value0, const double value1) const
  -> bool
{
  return std::abs(value0 - value1) <= tolerance;
}
}  // namespace geometry
}  // namespace math
