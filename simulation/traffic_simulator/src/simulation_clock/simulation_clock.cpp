#include <traffic_simulator/simulation_clock/simulation_clock.hpp>

namespace traffic_simulator
{
SimulationClock::SimulationClock()
: rclcpp::Clock(RCL_SYSTEM_TIME), step_time_duration_(0), initialized_(false)
{
}

void SimulationClock::initialize(double initial_simulation_time, double step_time)
{
  initialized_ = true;
  initial_simulation_time_ = initial_simulation_time;
  current_simulation_time_ = initial_simulation_time_;
  step_time_ = step_time;
  step_time_duration_ = rclcpp::Duration::from_seconds(step_time_);
  system_time_on_initialize_ = now();
}

void SimulationClock::update()
{
  if (!initialized_) {
    THROW_SIMULATION_CLOCK_ERROR("SimulationClock does not initialized yet.");
  }
  current_simulation_time_ = current_simulation_time_ + step_time_;
}

rosgraph_msgs::msg::Clock SimulationClock::getCurrentRosTimeAsMsg() const
{
  rosgraph_msgs::msg::Clock clock;
  clock.clock = getCurrentRosTime();
  return clock;
}

rclcpp::Time SimulationClock::getCurrentRosTime() const
{
  if (!initialized_) {
    THROW_SIMULATION_CLOCK_ERROR("SimulationClock does not initialized yet.");
  }
  return system_time_on_initialize_ +
         rclcpp::Duration::from_seconds(current_simulation_time_ - initial_simulation_time_);
}
}  // namespace traffic_simulator