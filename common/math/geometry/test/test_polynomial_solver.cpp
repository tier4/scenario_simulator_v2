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

#include <geometry/solver/polynomial_solver.hpp>
#include <geometry/spline/hermite_curve.hpp>
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

/**
 * @note Testcase for ax+b
 */
TEST(PolynomialSolverTest, LinearFunction)
{
  math::geometry::PolynomialSolver solver;
  EXPECT_DOUBLE_EQ(solver.linearFunction(1, 1, 2), 3);
  EXPECT_DOUBLE_EQ(solver.linearFunction(1, 0, 2), 2);
  EXPECT_DOUBLE_EQ(solver.linearFunction(0, 0, 2), 0);
}

/**
 * @note Testcase for ax+b = 0
 */
TEST(PolynomialSolverTest, SolveLinearEquation)
{
  math::geometry::PolynomialSolver solver;
  for (double a = -20; a < 20; a = a + 0.1) {
    for (double b = -20; b < 20; b = b + 0.1) {
      /**
       * @note If the ax+b=0 (a=0,b=0), any x value are solution, 
       * so throwing common::SimulationError errors are expected.
       */
      if (
        std::abs(a) <= math::geometry::PolynomialSolver::tolerance &&
        std::abs(b) <= math::geometry::PolynomialSolver::tolerance) {
        EXPECT_THROW(solver.solveLinearEquation(a, b, 0, 1), common::SimulationError);
      }
      /**
       * @note Other cases, solver must be able to obtain an exact solution.
       */
      else {
        auto ret = solver.solveLinearEquation(a, b, 0, 1);
        for (const auto & solution : ret) {
          EXPECT_TRUE(checkValueWithTolerance(solver.linearFunction(a, b, solution), 0.0, 1e-10));
        }
      }
    }
  }
}

/**
 * @note Testcase for ax^2+bx+c
 */
TEST(PolynomialSolverTest, QuadraticFunction)
{
  math::geometry::PolynomialSolver solver;
  EXPECT_DOUBLE_EQ(solver.quadraticFunction(1, 1, 1, 2), 7);
  EXPECT_DOUBLE_EQ(solver.quadraticFunction(1, 1, 0, 2), 6);
  EXPECT_DOUBLE_EQ(solver.quadraticFunction(0, 0, 0, 2), 0);
}

/**
 * @note Testcase for ax^3+bx^2+cx+d = 0
 */
TEST(PolynomialSolverTest, SolveQuadraticEquation)
{
  math::geometry::PolynomialSolver solver;
  for (double a = -20; a < 20; a = a + 1) {
    for (double b = -20; b < 20; b = b + 1) {
      for (double c = -20; c < 20; c = c + 1) {
        /**
         * @note If the ax^2+bx+c=0 (a=0,b=0,c=0), any x value are solution, 
         * so throwing common::SimulationError errors are expected.
         */
        if (
          std::abs(a) <= math::geometry::PolynomialSolver::tolerance &&
          std::abs(b) <= math::geometry::PolynomialSolver::tolerance &&
          std::abs(c) <= math::geometry::PolynomialSolver::tolerance) {
          EXPECT_THROW(solver.solveQuadraticEquation(a, b, c, 0, 1), common::SimulationError);
        }
        /**
         * @note Other cases, solver must be able to obtain an exact solution.
         */
        else {
          auto ret = solver.solveQuadraticEquation(a, b, c, 0, 1);
          for (const auto & solution : ret) {
            EXPECT_TRUE(
              checkValueWithTolerance(solver.quadraticFunction(a, b, c, solution), 0.0, 1e-10));
          }
        }
      }
    }
  }
}

/**
 * @note Testcase for ax^3+bx^2+cx+d = 0
 */
TEST(PolynomialSolverTest, SolveCubicEquation)
{
  math::geometry::PolynomialSolver solver;
  for (double a = -10; a < 10; a = a + 1) {
    for (double b = -10; b < 10; b = b + 1) {
      for (double c = -10; c < 10; c = c + 1) {
        for (double d = -10; d < 10; d = d + 1) {
          /**
           * @note If the ax^3+bx^2+cx+d=0 (a=0,b=0,c=0,d=0), any x value are solution, 
           * so throwing common::SimulationError errors are expected.
           */
          if (
            std::abs(a) <= math::geometry::PolynomialSolver::tolerance &&
            std::abs(b) <= math::geometry::PolynomialSolver::tolerance &&
            std::abs(c) <= math::geometry::PolynomialSolver::tolerance &&
            std::abs(d) <= math::geometry::PolynomialSolver::tolerance) {
            EXPECT_THROW(solver.solveCubicEquation(a, b, c, d, 0, 1), common::SimulationError);
          }
          /**
           * @note Other cases, solver must be able to obtain an exact solution.
           */
          else {
            auto ret = solver.solveCubicEquation(a, b, c, d, 0, 1);
            for (const auto & solution : ret) {
              EXPECT_TRUE(
                checkValueWithTolerance(solver.cubicFunction(a, b, c, d, solution), 0.0, 1e-10));
            }
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
