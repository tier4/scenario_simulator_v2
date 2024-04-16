#include <gtest/gtest.h>

#include <traffic_simulator/simulation_clock/simulation_clock.hpp>

TEST(SimulationClock, getCurrentRosTime)
{
  constexpr double realtime_factor = 2.0;
  constexpr double frame_rate = 10.0;
  traffic_simulator::SimulationClock simulation_clock(true, realtime_factor, frame_rate);
  simulation_clock.start();

  const auto initial_time = simulation_clock.getCurrentRosTime();

  for (int i = 0; i < 5; ++i) {
    simulation_clock.update();
  }

  const auto current_time = simulation_clock.getCurrentRosTime();
  const double elapsed_time = (current_time - initial_time).seconds();
  const double expected_elapsed_time = 5.0 * realtime_factor / frame_rate;

  EXPECT_NEAR(elapsed_time, expected_elapsed_time, 1e-6);
}

TEST(SimulationClock, getCurrentScenarioTime)
{
  traffic_simulator::SimulationClock clock(true, 1.0, 30.0);

  clock.start();

  EXPECT_DOUBLE_EQ(clock.getCurrentScenarioTime(), 0.0);

  for (int i = 0; i < 10; ++i) {
    clock.update();
    const double expected_scenario_time = (i + 1) * clock.getStepTime();
    EXPECT_DOUBLE_EQ(clock.getCurrentScenarioTime(), expected_scenario_time);
  }

  for (int i = 0; i < 5; ++i) {
    clock.update();
    const double expected_scenario_time = (10 + 1 + i) * clock.getStepTime();
    EXPECT_DOUBLE_EQ(clock.getCurrentScenarioTime(), expected_scenario_time);
  }
}

TEST(SimulationClock, Update)
{
  const bool use_sim_time = true;
  const double realtime_factor = 1.0;
  const double frame_rate = 10.0;
  traffic_simulator::SimulationClock simulation_clock(use_sim_time, realtime_factor, frame_rate);

  simulation_clock.start();
  const double initial_simulation_time = simulation_clock.getCurrentSimulationTime();

  const int num_updates = 10;
  const double step_time = simulation_clock.getStepTime();
  const double tolerance = 1e-6;

  for (int i = 0; i < num_updates; ++i) {
    simulation_clock.update();
    const double expected_simulation_time = initial_simulation_time + (i + 1) * step_time;
    const double actual_simulation_time = simulation_clock.getCurrentSimulationTime();
    EXPECT_NEAR(actual_simulation_time, expected_simulation_time, tolerance);
  }
}

TEST(SimulationClock, Initialize)
{
  traffic_simulator::SimulationClock sim_clock(true, 1.0, 10.0);

  sim_clock.started();
  sim_clock.update();

  EXPECT_NO_THROW(sim_clock.start());

  sim_clock.started();
  sim_clock.update();

  EXPECT_THROW(sim_clock.start(), std::runtime_error);
}