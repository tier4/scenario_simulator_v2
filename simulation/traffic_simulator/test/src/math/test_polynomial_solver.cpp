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

#include <gtest/gtest.h>

#include <geometry_math/hermite_curve.hpp>
#include <geometry_math/polynomial_solver.hpp>
#include <scenario_simulator_exception/exception.hpp>

bool checkValueWithTolerance(double value, double expected, double tolerance)
{
  if (tolerance < 0) {
    throw std::logic_error("tolerance should be over 0");
  }
  if (std::fabs(value - expected) < tolerance) {
    return true;
  }
  return false;
}

TEST(PolynomialSolverTest, LinearFunction)
{
  geometry_math::PolynomialSolver solver;
  EXPECT_DOUBLE_EQ(solver.linearFunction(1, 1, 2), 3);
  EXPECT_DOUBLE_EQ(solver.linearFunction(1, 0, 2), 2);
  EXPECT_DOUBLE_EQ(solver.linearFunction(0, 0, 2), 0);
}

TEST(PolynomialSolverTest, SolveLinearEquation)
{
  geometry_math::PolynomialSolver solver;
  for (double a = -20; a < 20; a = a + 0.1) {
    for (double b = -20; b < 20; b = b + 0.1) {
      auto ret = solver.solveLinearEquation(a, b, 0, 1);
      for (const auto & solution : ret) {
        EXPECT_TRUE(checkValueWithTolerance(solver.linearFunction(a, b, solution), 0.0, 1e-10));
      }
    }
  }
}

TEST(PolynomialSolverTest, QuadraticFunction)
{
  geometry_math::PolynomialSolver solver;
  EXPECT_DOUBLE_EQ(solver.quadraticFunction(1, 1, 1, 2), 7);
  EXPECT_DOUBLE_EQ(solver.quadraticFunction(1, 1, 0, 2), 6);
  EXPECT_DOUBLE_EQ(solver.quadraticFunction(0, 0, 0, 2), 0);
}

TEST(PolynomialSolverTest, SolveQuadraticEquation)
{
  geometry_math::PolynomialSolver solver;
  for (int a = -20; a < 20; a = a + 1) {
    for (int b = -20; b < 20; b = b + 1) {
      for (int c = -20; c < 20; c = c + 1) {
        auto ret = solver.solveQuadraticEquation(
          static_cast<double>(a), static_cast<double>(b), static_cast<double>(c), 0, 1);
        for (const auto & solution : ret) {
          EXPECT_TRUE(checkValueWithTolerance(
            solver.quadraticFunction(
              static_cast<double>(a), static_cast<double>(b), static_cast<double>(c), solution),
            0.0, 1e-10));
        }
      }
    }
  }
}

TEST(PolynomialSolverTest, SolveCubicEquation)
{
  geometry_math::PolynomialSolver solver;
  for (int a = -10; a < 10; a = a + 1) {
    for (int b = -10; b < 10; b = b + 1) {
      for (int c = -10; c < 10; c = c + 1) {
        for (int d = -10; d < 10; d = d + 1) {
          auto ret = solver.solveCubicEquation(
            static_cast<double>(a), static_cast<double>(b), static_cast<double>(c),
            static_cast<double>(d), 0, 1);
          for (const auto & solution : ret) {
            EXPECT_TRUE(checkValueWithTolerance(
              solver.cubicFunction(
                static_cast<double>(a), static_cast<double>(b), static_cast<double>(c),
                static_cast<double>(d), solution),
              0.0, 1e-10));
          }
        }
      }
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
