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

#include <traffic_simulator/simulation_clock/simulation_clock.hpp>

/**
 * @note Test basic functionality used in API.
 * Test initialization logic by calling update without initialized clock
 * - the goal is to verify that mandatory initialization works.
 */
TEST(SimulationClock, Initialize)
{
  const double realtime_factor = 1.0;
  const double frame_rate = 10.0;
  const bool use_sim_time = true;
  auto simulation_clock =
    traffic_simulator::SimulationClock(use_sim_time, realtime_factor, frame_rate);

  EXPECT_FALSE(simulation_clock.started());
  simulation_clock.update();
  EXPECT_NO_THROW(simulation_clock.start());

  EXPECT_TRUE(simulation_clock.started());
  simulation_clock.update();
  EXPECT_THROW(simulation_clock.start(), std::runtime_error);
}

/**
 * @note Test basic functionality used in API. Test time obtaining correctness with initialized object
 * and not using raw clock - the goal is to test whether the time has increased
 * according to the times of update function calls.
 */
TEST(SimulationClock, getCurrentRosTime)
{
  const double realtime_factor = 2.0;
  const double frame_rate = 10.0;
  const bool use_sim_time = true;
  auto simulation_clock =
    traffic_simulator::SimulationClock(use_sim_time, realtime_factor, frame_rate);
  simulation_clock.start();

  const auto initial_time = simulation_clock.getCurrentRosTime();

  const int iterations = 5;
  for (int i = 0; i < iterations; ++i) {
    simulation_clock.update();
  }

  const auto current_time = simulation_clock.getCurrentRosTime();
  const double result_elapsed_time = (current_time - initial_time).seconds();
  const double actual_elapsed_time = static_cast<double>(iterations) * realtime_factor / frame_rate;
  const double epsilon = 1e-6;

  EXPECT_NEAR(result_elapsed_time, actual_elapsed_time, epsilon);
}

/**
 * @note Test basic functionality used in API. Test scenario time calculation correctness with initialized obejct,
 * npc logic started after several update() calls and additional update() calls after starting npc logic.
 */
TEST(SimulationClock, getCurrentScenarioTime)
{
  const double realtime_factor = 1.0;
  const double frame_rate = 30.0;
  const bool use_sim_time = true;
  auto simulation_clock =
    traffic_simulator::SimulationClock(use_sim_time, realtime_factor, frame_rate);

  simulation_clock.start();

  EXPECT_DOUBLE_EQ(simulation_clock.getCurrentScenarioTime(), 0.0);

  const int iterations = 5;
  for (int i = 0; i < iterations; ++i) {
    simulation_clock.update();
  }

  const double result_elapsed_time = simulation_clock.getCurrentScenarioTime();
  const double actual_elapsed_time = static_cast<double>(iterations) * realtime_factor / frame_rate;
  const double epsilon = 1e-6;

  EXPECT_NEAR(actual_elapsed_time, result_elapsed_time, epsilon);
}

/**
 * @note Test basic functionality used in API. Test updating correctness with initialized object
 * by calling update several times expecting the current time to increase accordingly.
 */
TEST(SimulationClock, Update)
{
  const double realtime_factor = 1.0;
  const double frame_rate = 10.0;
  const bool use_sim_time = true;
  auto simulation_clock =
    traffic_simulator::SimulationClock(use_sim_time, realtime_factor, frame_rate);

  simulation_clock.start();

  const double initial_simulation_time = simulation_clock.getCurrentSimulationTime();

  const int iterations = 10;
  const double step_time = simulation_clock.getStepTime();
  const double tolerance = 1e-6;

  for (int i = 0; i < iterations; ++i) {
    simulation_clock.update();
    const double expected_simulation_time =
      initial_simulation_time + static_cast<double>(i + 1) * step_time;
    const double actual_simulation_time = simulation_clock.getCurrentSimulationTime();
    EXPECT_NEAR(actual_simulation_time, expected_simulation_time, tolerance);
  }
}
