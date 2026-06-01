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
  acc_input_queue_.push_back((raw_pedal_cmd >= 0.0) ? raw_pedal_cmd : 0.0);
  brake_input_queue_.push_back((raw_pedal_cmd < 0.0) ? raw_pedal_cmd : 0.0);

  // Dequeue values after their respective delay times have passed
  const double acc_delayed_val = acc_input_queue_.front();
  acc_input_queue_.pop_front();
  const double brake_delayed_val = brake_input_queue_.front();
  brake_input_queue_.pop_front();

  // Brake Override System (BOS)
  // Prioritize brake command if both acceleration and brake commands are active simultaneously
  delayed_input(IDX_U::PEDAL_ACCX_DES) =
    (brake_delayed_val < -1e-5) ? brake_delayed_val : acc_delayed_val;

  // Steering motor queue processing
  steer_motor_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_motor_input_queue_.front();
  steer_motor_input_queue_.pop_front();

  delayed_input(IDX_U::GEAR) = input_(IDX_U::GEAR);
  delayed_input(IDX_U::SLOPE_ACCX) = input_(IDX_U::SLOPE_ACCX);

  // Pedal (Acceleration & Braking) command processing
  const double baseline_acc = acc_offset_ - brake_offset_;
  const double actual_jump_value = brake_jump_value_ * (1.0 + brake_accuracy_error_);
  bool is_brake_pad_contacting = false;

  delayed_input(IDX_U::PEDAL_ACCX_DES) = [&]() {
    double cmd = delayed_input(IDX_U::PEDAL_ACCX_DES) * debug_acc_scaling_factor_;

    if (cmd < 0.0) {  // --- Braking ---
      double brake_cmd = std::abs(cmd);

      // Apply resolution if configured
      if (brake_resolution_ > 1e-5) {
        brake_cmd = std::round(brake_cmd / brake_resolution_) * brake_resolution_;
      }

      // Update hysteresis state
      if (brake_cmd < 1e-5) {
        brake_hysteresis_state_ = 0.0;  // Reset when the pedal is fully released
      } else {
        brake_hysteresis_state_ = std::max(0.0, std::clamp(
          brake_hysteresis_state_,
          brake_cmd - (brake_hysteresis_width_ / 2.0),
          brake_cmd + (brake_hysteresis_width_ / 2.0)
        ));
      }

      // Calculate final braking command
      cmd = 0.0;  // Default to zero friction (inside deadband)
      if (brake_hysteresis_state_ > brake_dead_band_) {
        is_brake_pad_contacting = true;
        cmd = -(brake_hysteresis_state_ - brake_dead_band_ + brake_jump_value_) * (1.0 + brake_accuracy_error_);
      }
    } else {  // --- Acceleration or Coasting ---
      // Reset brake internal state
      brake_hysteresis_state_ = 0.0;

      // Apply resolution if configured
      if (acc_resolution_ > 1e-5) {
        cmd = std::round(cmd / acc_resolution_) * acc_resolution_;
      }

      // Apply deadband and accuracy error
      cmd = std::max(0.0, cmd - acc_dead_band_) * (1.0 + acc_accuracy_error_);
    }

    // Apply baseline and absolute physical limits
    return std::clamp(cmd + baseline_acc, -brake_lim_, acc_lim_);
  }();

  // Override continuous state to simulate discrete mechanical behavior
  if (is_brake_pad_contacting) {  // Brake pad touched
    // Apply initial braking jump directly to vehicle state (overcoming clearance)
    const double apply_jump_target = baseline_acc - actual_jump_value;
    if (state_(IDX::PEDAL_ACCX) <= baseline_acc && state_(IDX::PEDAL_ACCX) > apply_jump_target) {
      state_(IDX::PEDAL_ACCX) = apply_jump_target;
    }
  } else {  // Brake pad released
    // Instantly clear residual braking force to prevent unnatural drag
    if (state_(IDX::PEDAL_ACCX) < baseline_acc && state_(IDX::PEDAL_ACCX) >= baseline_acc - actual_jump_value) {
      state_(IDX::PEDAL_ACCX) = baseline_acc;
    }
  }

  // Steering motor command processing
  delayed_input(IDX_U::STEER_DES) = [&]() {
    double cmd = delayed_input(IDX_U::STEER_DES) * debug_steer_scaling_factor_;

    // Apply resolution if configured
    if (steer_resolution_ > 1e-5) {
      cmd = std::round(cmd / steer_resolution_) * steer_resolution_;
    }

    // Apply Hysteresis
    const double steer_motor_hist = std::clamp(
      (state_(IDX::STEER) - steer_bias_) / (1.0 + steer_accuracy_error_), // Current steering motor angle
      cmd - (steer_hysteresis_width_ / 2.0),
      cmd + (steer_hysteresis_width_ / 2.0)
    );

    // Apply final limit
    return std::clamp(steer_motor_hist, -steer_lim_, steer_lim_);
  }();

  // Cache state before integration, for post-processing and constraints evaluation
  const auto prev_state = state_;

  updateRungeKutta(dt, delayed_input);

  // Apply physical limits to raw integration results
  state_(IDX::VX) = std::clamp(state_(IDX::VX), -vx_lim_, vx_lim_);
  state_(IDX::PEDAL_ACCX) = std::clamp(state_(IDX::PEDAL_ACCX), -brake_lim_, acc_lim_);

  // Enforce physical steering limits based on steering motor limits, accuracy error, and bias
  state_(IDX::STEER) = [&]() {
    const double upper = steer_lim_ * (1.0 + steer_accuracy_error_) + steer_bias_;
    const double lower = -steer_lim_ * (1.0 + steer_accuracy_error_) + steer_bias_;

    // Failsafe clamp to prevent upper/lower limit reversal
    return std::clamp(state_(IDX::STEER), std::min(upper, lower), std::max(upper, lower));
  }();

  // Snap velocity to 0.0 and freeze position when braking to a halt.
  // Prevents floating-point drift and guarantees stable ADK state transitions.
  constexpr double stop_epsilon = 1e-3;
  if (delayed_input(IDX_U::PEDAL_ACCX_DES) < 0.0 && std::abs(state_(IDX::VX)) < stop_epsilon) {
    state_(IDX::VX) = 0.0;
    state_(IDX::X) = prev_state(IDX::X);
    state_(IDX::Y) = prev_state(IDX::Y);
    state_(IDX::YAW) = prev_state(IDX::YAW);
  }

  // Calculate actual acceleration based on the finalized velocity delta
  state_(IDX::ACCX) = (state_(IDX::VX) - prev_state(IDX::VX)) / dt;

  // Update velocity history queue and retrieve delayed velocity
  const double raw_delayed_vx = [&]() {
    if (vel_history_queue_.empty()) {
      return state_(IDX::VX);
    }

    vel_history_queue_.push_back(state_(IDX::VX));
    const double front_val = vel_history_queue_.front();
    vel_history_queue_.pop_front();
    return front_val;
  }();

  // Apply sensor characteristics (accuracy, offset, noise, resolution)
  delayed_vx_ = [&]() {
    if (std::abs(raw_delayed_vx) < stop_epsilon) {
      return 0.0;
    }

    double vx = raw_delayed_vx * (1.0 + vel_sensor_accuracy_error_) + vel_sensor_offset_;
    if (vel_sensor_noise_stddev_ > 1e-5) {
      vx += vel_dist_(vel_rng_) * vel_sensor_noise_stddev_;
    }
    if (vel_sensor_resolution_ > 1e-5) {
      vx = std::round(vx / vel_sensor_resolution_) * vel_sensor_resolution_;
    }
    return vx;
  }();
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
