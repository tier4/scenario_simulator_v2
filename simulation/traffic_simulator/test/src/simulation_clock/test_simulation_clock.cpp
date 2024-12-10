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

#include <scenario_simulator_exception/exception.hpp>
#include <traffic_simulator/simulation_clock/simulation_clock.hpp>

/**
 * @note Test basic functionality used in API.
 * Test initialization logic by calling update without initialized clock
 * - the goal is to verify that mandatory initialization works.
 */
TEST(SimulationClock, SimulationClock)
{
  auto simulation_clock = traffic_simulator::SimulationClock(true, 1.0, 10.0);

  EXPECT_FALSE(simulation_clock.started());
  simulation_clock.update();
  EXPECT_NO_THROW(simulation_clock.start());

  EXPECT_TRUE(simulation_clock.started());
  simulation_clock.update();
  EXPECT_THROW(simulation_clock.start(), common::SimulationError);
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
  auto simulation_clock = traffic_simulator::SimulationClock(true, realtime_factor, frame_rate);
  simulation_clock.start();

  const auto initial_time = simulation_clock.getCurrentRosTime();

  const int iterations = 5;
  for (int i = 0; i < iterations; ++i) {
    simulation_clock.update();
  }

  EXPECT_NEAR(
    (simulation_clock.getCurrentRosTime() - initial_time).seconds(),
    static_cast<double>(iterations) * realtime_factor / frame_rate, 1e-6);
}

/**
 * @note Test basic functionality used in API. Test scenario time calculation correctness with initialized object,
 * npc logic started after several update() calls and additional update() calls after starting npc logic.
 */
TEST(SimulationClock, getCurrentScenarioTime)
{
  const double realtime_factor = 1.0;
  const double frame_rate = 30.0;
  auto simulation_clock = traffic_simulator::SimulationClock(true, realtime_factor, frame_rate);

  simulation_clock.start();

  EXPECT_DOUBLE_EQ(simulation_clock.getCurrentScenarioTime(), 0.0);

  const int iterations = 5;
  for (int i = 0; i < iterations; ++i) {
    simulation_clock.update();
  }

  EXPECT_NEAR(
    static_cast<double>(iterations) * realtime_factor / frame_rate,
    simulation_clock.getCurrentScenarioTime(), 1e-6);
}

/**
 * @note Test basic functionality used in API. Test updating correctness with initialized object
 * by calling update several times expecting the current time to increase accordingly.
 */
TEST(SimulationClock, Update)
{
  auto simulation_clock = traffic_simulator::SimulationClock(true, 1.0, 10.0);

  simulation_clock.start();

  const double initial_simulation_time = simulation_clock.getCurrentSimulationTime();
  const double step_time = simulation_clock.getStepTime();

  for (int i = 0; i < 10; ++i) {
    simulation_clock.update();
    const double expected_simulation_time =
      initial_simulation_time + static_cast<double>(i + 1) * step_time;
    const double actual_simulation_time = simulation_clock.getCurrentSimulationTime();
    EXPECT_NEAR(actual_simulation_time, expected_simulation_time, 1e-6);
  }
}
