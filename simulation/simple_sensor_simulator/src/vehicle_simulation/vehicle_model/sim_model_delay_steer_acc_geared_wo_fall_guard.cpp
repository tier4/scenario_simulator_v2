// Copyright 2025 The Autoware Foundation.
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
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_delay_steer_acc_geared_wo_fall_guard.hpp>

namespace autoware::simulator::simple_planning_simulator
{

SimModelDelaySteerAccGearedWoFallGuard::SimModelDelaySteerAccGearedWoFallGuard(
  double vx_lim, double acc_lim, double brake_lim, double acc_rate_lim, double brake_rate_lim, double steer_lim, double steer_rate_lim, double wheelbase,
  double dt, double acc_delay, double brake_delay, double acc_time_constant, double brake_time_constant,
  double acc_accuracy_error, double brake_accuracy_error, double brake_hysteresis_width, double acc_dead_band, double brake_dead_band, double brake_jump_value, double acc_offset, double brake_offset, double acc_resolution, double brake_resolution,
  double steer_delay,
  double steer_time_constant, double steer_dead_band, double steer_bias,
  double steer_accuracy_error, double steer_resolution, double steer_hysteresis_width,
  double vel_sensor_delay, double vel_sensor_resolution, double vel_sensor_noise_stddev, int vel_sensor_noise_seed, double vel_sensor_accuracy_error, double vel_sensor_offset,
  double debug_acc_scaling_factor, double debug_steer_scaling_factor, double rolling_resistance, double air_drag_coef)
: SimModelInterface(7 /* dim x */, 4 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  acc_lim_(acc_lim),
  brake_lim_(brake_lim),
  acc_rate_lim_(acc_rate_lim),
  brake_rate_lim_(brake_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  acc_delay_(acc_delay),
  brake_delay_(brake_delay),
  acc_time_constant_(std::max(acc_time_constant, MIN_TIME_CONSTANT)),
  brake_time_constant_(std::max(brake_time_constant, MIN_TIME_CONSTANT)),
  acc_accuracy_error_(acc_accuracy_error),
  brake_accuracy_error_(brake_accuracy_error),
  brake_hysteresis_width_(brake_hysteresis_width),
  acc_dead_band_(acc_dead_band),
  brake_dead_band_(brake_dead_band),
  brake_jump_value_(brake_jump_value),
  acc_offset_(acc_offset),
  brake_offset_(brake_offset),
  acc_resolution_(acc_resolution),
  brake_resolution_(brake_resolution),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT)),
  steer_dead_band_(steer_dead_band),
  steer_bias_(steer_bias),
  steer_accuracy_error_(steer_accuracy_error),
  steer_resolution_(steer_resolution),
  steer_hysteresis_width_(steer_hysteresis_width),
  vel_sensor_delay_(vel_sensor_delay),
  vel_sensor_resolution_(vel_sensor_resolution),
  vel_sensor_noise_stddev_(std::max(vel_sensor_noise_stddev, 0.0)),
  vel_sensor_accuracy_error_(vel_sensor_accuracy_error),
  vel_sensor_offset_(vel_sensor_offset),
  debug_acc_scaling_factor_(std::max(debug_acc_scaling_factor, 0.0)),
  debug_steer_scaling_factor_(std::max(debug_steer_scaling_factor, 0.0)),
  rolling_resistance_(std::max(rolling_resistance, 0.0)),
  air_drag_coef_(std::max(air_drag_coef, 0.0)),
  brake_hysteresis_state_(0.0),
  delayed_vx_(0.0),
  vel_rng_(vel_sensor_noise_seed),
  vel_dist_(0.0, 1.0)
{
  initializeInputQueue(dt);
}

double SimModelDelaySteerAccGearedWoFallGuard::getX() { return state_(IDX::X); }

double SimModelDelaySteerAccGearedWoFallGuard::getY() { return state_(IDX::Y); }

double SimModelDelaySteerAccGearedWoFallGuard::getYaw() { return state_(IDX::YAW); }

double SimModelDelaySteerAccGearedWoFallGuard::getVx() { return delayed_vx_; }

double SimModelDelaySteerAccGearedWoFallGuard::getVy() { return 0.0; }

double SimModelDelaySteerAccGearedWoFallGuard::getAx() { return state_(IDX::ACCX); }

double SimModelDelaySteerAccGearedWoFallGuard::getWz()
{
  return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / wheelbase_;
}

double SimModelDelaySteerAccGearedWoFallGuard::getSteer()
{
  return (state_(IDX::STEER) - steer_bias_) / (1.0 + steer_accuracy_error_);
}

void SimModelDelaySteerAccGearedWoFallGuard::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  // Separation of acceleration and brake signals at the input stage
  const double raw_pedal_cmd = input_(IDX_U::PEDAL_ACCX_DES);

  if (raw_pedal_cmd >= 0.0) {
    // Acceleration command: insert into acceleration queue, insert 0.0 into brake queue
    acc_input_queue_.push_back(raw_pedal_cmd);
    brake_input_queue_.push_back(0.0);
  } else {
    // Braking command: insert 0.0 into acceleration queue, insert into brake queue
    acc_input_queue_.push_back(0.0);
    brake_input_queue_.push_back(raw_pedal_cmd);
  }

  // Dequeue values after their respective delay times have passed
  const double acc_delayed_val = acc_input_queue_.front();
  acc_input_queue_.pop_front();
  const double brake_delayed_val = brake_input_queue_.front();
  brake_input_queue_.pop_front();

  // Brake Override System (BOS)
  // Prioritize brake command if both acceleration and brake commands are active simultaneously
  if (brake_delayed_val < -1e-5) {  // tolerance for floating-point zero evaluation
    delayed_input(IDX_U::PEDAL_ACCX_DES) = brake_delayed_val;
  } else {
    // Use acceleration value (including 0.0 for coasting) when no brake command is active
    delayed_input(IDX_U::PEDAL_ACCX_DES) = acc_delayed_val;
  }

  steer_motor_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_motor_input_queue_.front();
  steer_motor_input_queue_.pop_front();
  delayed_input(IDX_U::GEAR) = input_(IDX_U::GEAR);
  delayed_input(IDX_U::SLOPE_ACCX) = input_(IDX_U::SLOPE_ACCX);

  // Nonlinear filter calculation
  // 1. Acceleration and brake filter
  double pedal_acc_des = delayed_input(IDX_U::PEDAL_ACCX_DES) * debug_acc_scaling_factor_;

  bool is_brake_pad_contacting = false;

  // Pre-calculate baseline pedal acceleration
  const double baseline_acc = acc_offset_ - brake_offset_;
  const double actual_jump_value = brake_jump_value_ * (1.0 + brake_accuracy_error_);

  if (pedal_acc_des < 0.0) {
    double brake_cmd = std::abs(pedal_acc_des);

    if (brake_resolution_ > 1e-5) { // tolerance to check if the parameter is configured (non-zero)
      brake_cmd = std::round(brake_cmd / brake_resolution_) * brake_resolution_;
    }

    // Reset hysteresis when the pedal is fully released
    double hist_cmd = 0.0;
    if (brake_cmd < 1e-5) { // tolerance to determine if the pedal is fully released
      hist_cmd = 0.0;
    } else {
      hist_cmd = std::clamp(brake_hysteresis_state_, brake_cmd - (brake_hysteresis_width_ / 2.0), brake_cmd + (brake_hysteresis_width_ / 2.0));
    }
    hist_cmd = std::max(0.0, hist_cmd);
    brake_hysteresis_state_ = hist_cmd;

    double jump_cmd = 0.0;
    if (hist_cmd > brake_dead_band_) {
      // Pad contact detection (exceeded dead band)
      is_brake_pad_contacting = true;

      const double deadzoned_cmd = hist_cmd - brake_dead_band_;
      jump_cmd = deadzoned_cmd + brake_jump_value_;
      jump_cmd = jump_cmd * (1.0 + brake_accuracy_error_);

      // Calculate target pedal acceleration for the initial braking jump
      const double apply_jump_target = baseline_acc - actual_jump_value;

      if (state_(IDX::PEDAL_ACCX) <= baseline_acc && state_(IDX::PEDAL_ACCX) > apply_jump_target) {
        state_(IDX::PEDAL_ACCX) = apply_jump_target;
      }
    }

    pedal_acc_des = -jump_cmd;
  } else {
    brake_hysteresis_state_ = 0.0;

    if (acc_resolution_ > 1e-5) { // tolerance to check if the parameter is configured (non-zero)
      pedal_acc_des = std::round(pedal_acc_des / acc_resolution_) * acc_resolution_;
    }

    if (pedal_acc_des > acc_dead_band_) {
      pedal_acc_des = pedal_acc_des - acc_dead_band_;

      pedal_acc_des = pedal_acc_des * (1.0 + acc_accuracy_error_);
    } else {
      pedal_acc_des = 0.0;
    }
  }

  // Prevent unnatural brake dragging when the pedal is released
  if (!is_brake_pad_contacting) {
    if (state_(IDX::PEDAL_ACCX) < baseline_acc && state_(IDX::PEDAL_ACCX) >= baseline_acc - actual_jump_value) {
      state_(IDX::PEDAL_ACCX) = baseline_acc;
    }
  }

  pedal_acc_des = pedal_acc_des + baseline_acc;

  delayed_input(IDX_U::PEDAL_ACCX_DES) = std::clamp(pedal_acc_des, -brake_lim_, acc_lim_);

  // 2. Steering filter
  double steer_des = delayed_input(IDX_U::STEER_DES) * debug_steer_scaling_factor_;

  if (steer_resolution_ > 1e-5) { // tolerance to check if the parameter is configured (non-zero)
    steer_des = std::round(steer_des / steer_resolution_) * steer_resolution_;
  }

  const double current_motor_angle = (state_(IDX::STEER) - steer_bias_) / (1.0 + steer_accuracy_error_);
  const double steer_hist = std::clamp(current_motor_angle, steer_des - (steer_hysteresis_width_ / 2.0), steer_des + (steer_hysteresis_width_ / 2.0));

  delayed_input(IDX_U::STEER_DES) = std::clamp(steer_hist, -steer_lim_, steer_lim_);

  const auto prev_state = state_;

  // Use 4th-order Runge-Kutta (RK4) method for precise physical simulation
  updateRungeKutta(dt, delayed_input);

  // Speed limit and stop evaluation
  state_(IDX::VX) = std::max(-vx_lim_, std::min(state_(IDX::VX), vx_lim_));

  // Calculate physical steering limits based on motor limits, gear ratio, and bias
  const double tire_steer_upper_lim = steer_lim_ * (1.0 + steer_accuracy_error_) + steer_bias_;
  const double tire_steer_lower_lim = -steer_lim_ * (1.0 + steer_accuracy_error_) + steer_bias_;
  // Clamp with a failsafe to prevent upper and lower limit reversal
  state_(IDX::STEER) = std::clamp(
    state_(IDX::STEER),
    std::min(tire_steer_upper_lim, tire_steer_lower_lim),
    std::max(tire_steer_upper_lim, tire_steer_lower_lim)
  );

  state_(IDX::PEDAL_ACCX) = std::clamp(state_(IDX::PEDAL_ACCX), -brake_lim_, acc_lim_);

  // Zero-snap processing to prevent floating-point errors
  // Round off to exactly 0.0 if the RK4 integration result for velocity is extremely close to zero
  // (Measure to prevent ADK state transition deadlocks)
  const double snap_epsilon = 0.001;  // threshold to snap velocity to exactly 0.0, preventing floating-point drift
  if (delayed_input(IDX_U::PEDAL_ACCX_DES) < 0.0) { // When brake command is active
    if (std::abs(state_(IDX::VX)) < snap_epsilon) {
      state_(IDX::VX) = 0.0;
      // Fix minor positional drifts
      state_(IDX::X) = prev_state(IDX::X);
      state_(IDX::Y) = prev_state(IDX::Y);
      state_(IDX::YAW) = prev_state(IDX::YAW);
    }
  }

  state_(IDX::ACCX) = (state_(IDX::VX) - prev_state(IDX::VX)) / dt;

  double raw_delayed_vx = 0.0;

  if (vel_history_queue_.empty()) {
    raw_delayed_vx = state_(IDX::VX);
  } else {
    vel_history_queue_.push_back(state_(IDX::VX));
    raw_delayed_vx = vel_history_queue_.front();
    vel_history_queue_.pop_front();
  }

  // Calculate sensor output once per step
  if (std::abs(raw_delayed_vx) < 1e-3) {  // threshold to snap sensor velocity output to 0.0
    delayed_vx_ = 0.0;
  } else {
    double vx = raw_delayed_vx * (1.0 + vel_sensor_accuracy_error_);
    vx += vel_sensor_offset_;
    if (vel_sensor_noise_stddev_ > 1e-5) {  // tolerance to check if the parameter is configured (non-zero)
      vx += vel_dist_(vel_rng_) * vel_sensor_noise_stddev_;
    }
    if (vel_sensor_resolution_ > 1e-5) {  // tolerance to check if the parameter is configured (non-zero)
      vx = std::round(vx / vel_sensor_resolution_) * vel_sensor_resolution_;
    }
    delayed_vx_ = vx;
  }
}

void SimModelDelaySteerAccGearedWoFallGuard::initializeInputQueue(const double & dt)
{
  // Calculate initial acceleration and brake commands
  const auto [initial_acc_cmd, initial_brake_cmd] = [&]() -> std::pair<double, double> {
    const double pedal_acc = state_(IDX::PEDAL_ACCX);
    if (pedal_acc > 0.0) {
      return {(pedal_acc / (1.0 + acc_accuracy_error_)) + acc_dead_band_, 0.0};
    }
    if (pedal_acc < 0.0) {
      const double jump_cmd = std::abs(pedal_acc);
      const double deadzoned_cmd = (jump_cmd / (1.0 + brake_accuracy_error_)) - brake_jump_value_;
      const double brake_cmd_abs = std::max(0.0, deadzoned_cmd) + brake_dead_band_;
      return {0.0, -brake_cmd_abs};
    }
    return {0.0, 0.0};
  }();

  // Initialize acceleration and brake queues
  const size_t acc_queue_size = static_cast<size_t>(std::round(acc_delay_ / dt));
  acc_input_queue_.assign(acc_queue_size, initial_acc_cmd);
  const size_t brake_queue_size = static_cast<size_t>(std::round(brake_delay_ / dt));
  brake_input_queue_.assign(brake_queue_size, initial_brake_cmd);

  brake_hysteresis_state_ = std::abs(initial_brake_cmd);

  // Calculate initial steering motor command and initialize steering motor queue
  const double initial_steer_motor_cmd = (state_(IDX::STEER) - steer_bias_) / (1.0 + steer_accuracy_error_);
  const size_t steer_motor_queue_size = static_cast<size_t>(std::round(steer_delay_ / dt));
  steer_motor_input_queue_.assign(steer_motor_queue_size, initial_steer_motor_cmd);

  // Calculate initial velocity and initialize velocity history queue
  const double initial_vel = state_(IDX::VX);
  const size_t vel_queue_size = static_cast<size_t>(std::round(vel_sensor_delay_ / dt));
  vel_history_queue_.assign(vel_queue_size, initial_vel);

  delayed_vx_ = initial_vel;
}

Eigen::VectorXd SimModelDelaySteerAccGearedWoFallGuard::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  using autoware_vehicle_msgs::msg::GearCommand;

  // Extract states with safety clamps
  const double vel = std::clamp(state(IDX::VX), -vx_lim_, vx_lim_);
  const double pedal_acc = std::clamp(state(IDX::PEDAL_ACCX), -brake_lim_, acc_lim_);
  const double yaw = state(IDX::YAW);
  // Prevent NaN explosion in std::tan() during Runge-Kutta integration steps
  const double current_steer = std::clamp(state(IDX::STEER), -steer_lim_, steer_lim_);

  const double pedal_acc_des = input(IDX_U::PEDAL_ACCX_DES);
  const double steer_motor_des = input(IDX_U::STEER_DES);
  const double slope_accx = input(IDX_U::SLOPE_ACCX);
  const auto gear = input(IDX_U::GEAR);

  // Dynamically select time constants and jerk limits based on pedal commands
  constexpr double eps = 1e-5;  // Threshold for zero evaluation
  const double current_tc = [&]() {
    if (pedal_acc_des > (acc_offset_ + eps)) {
      return acc_time_constant_;  // Active acceleration
    }
    if (pedal_acc_des < (-brake_offset_ - eps)) {
      return brake_time_constant_;  // Active braking
    }
    // Coasting: release remaining forces based on the actual pedal state
    return (pedal_acc < 0.0) ? brake_time_constant_ : acc_time_constant_;
  }();
  const double current_jerk_lim = [&]() {
    if (pedal_acc_des > (acc_offset_ + eps)) {
      return acc_rate_lim_;
    }
    if (pedal_acc_des < (-brake_offset_ - eps)) {
      return brake_rate_lim_;
    }
    return (pedal_acc < 0.0) ? brake_rate_lim_ : acc_rate_lim_;
  }();

  // Evaluate steering motor control error with deadband
  const double current_steer_motor = (current_steer - steer_bias_) / (1.0 + steer_accuracy_error_);
  const double steer_motor_diff = current_steer_motor - steer_motor_des;
  const double steer_motor_diff_with_dead_band = [&]() {
    if (steer_motor_diff > steer_dead_band_) {
      return steer_motor_diff - steer_dead_band_;
    }
    if (steer_motor_diff < -steer_dead_band_) {
      return steer_motor_diff + steer_dead_band_;
    }
    return 0.0;
  }();

  // Compute longitudinal acceleration (d_vx) using Newtonian mechanics and static friction models
  const double d_vx = [&] {
    if (gear == GearCommand::NONE || gear == GearCommand::PARK) {
      return 0.0;
    }

    // Aerodynamic drag: proportional to the square of velocity, opposing the motion
    const double air_drag = -air_drag_coef_ * vel * std::abs(vel);

    // Engine thrust: generated only when the accelerator is pressed, depending on the gear
    const double engine_acc = [&]() {
      if (pedal_acc >= 0.0) {
        if (gear == GearCommand::NEUTRAL) {
          return 0.0;
        }
        if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
          return -pedal_acc;
        }
        return pedal_acc;
      }
      return 0.0;
    }();

    // Static friction model (Approximated Coulomb friction via virtual spring)
    const double vel_epsilon = 0.02;    // Threshold for near-zero velocity
    const double k = 1.0 / vel_epsilon; // Virtual spring constant (viscous damping coefficient)

    // Total external acceleration acting on the vehicle
    const double external_acc = engine_acc + slope_accx + air_drag;

    // Available static friction force limits (Brake force + Rolling resistance)
    const double brake_force = (pedal_acc < 0.0) ? -pedal_acc : 0.0;
    const double friction_limit = brake_force + rolling_resistance_;

    // Combined motion equation:
    // The vehicle targets zero velocity (-k * vel) near standstill, capped by the available friction limit.
    return std::clamp(-k * vel, external_acc - friction_limit, external_acc + friction_limit);
  }();

  // Construct the final state derivatives
  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X)          = vel * std::cos(yaw);
  d_state(IDX::Y)          = vel * std::sin(yaw);
  d_state(IDX::YAW)        = vel * std::tan(current_steer) / wheelbase_;
  d_state(IDX::VX)         = d_vx;
  d_state(IDX::STEER)      = std::clamp(-steer_motor_diff_with_dead_band / steer_time_constant_, -steer_rate_lim_, steer_rate_lim_) * (1.0 + steer_accuracy_error_);
  d_state(IDX::PEDAL_ACCX) = std::clamp(-(pedal_acc - pedal_acc_des) / current_tc, -current_jerk_lim, current_jerk_lim);

  return d_state;
}

}  // namespace autoware::simulator::simple_planning_simulator
