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
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <cmath>
#include <functional>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_taiga_dyn.hpp>

namespace autoware::simulator::simple_planning_simulator
{

SimModelTaigaDyn::SimModelTaigaDyn(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double dt, double acc_delay, double acc_time_constant, double steer_delay,
  double steer_time_constant, double steer_dead_band, double steer_bias,
  double debug_acc_scaling_factor, double debug_steer_scaling_factor, double mass,
  double inertia_z, double lf, double lr, double cornering_stiffness_front,
  double cornering_stiffness_rear, double vx_min_dyn)
: SimModelInterface(9 /* dim x */, 4 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  acc_delay_(acc_delay),
  acc_time_constant_(std::max(acc_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT)),
  steer_dead_band_(steer_dead_band),
  steer_bias_(steer_bias),
  debug_acc_scaling_factor_(std::max(debug_acc_scaling_factor, 0.0)),
  debug_steer_scaling_factor_(std::max(debug_steer_scaling_factor, 0.0)),
  mass_(mass),
  inertia_z_(inertia_z),
  lf_(lf),
  lr_(lr),
  cf_(cornering_stiffness_front),
  cr_(cornering_stiffness_rear),
  vx_min_dyn_(std::max(vx_min_dyn, MIN_TIME_CONSTANT))
{
  initializeInputQueue(dt);
}

double SimModelTaigaDyn::calc_kinematic_yaw_rate(double vel, double steer) const
{
  // Ideal kinematic bicycle yaw rate; used only as the low-speed fallback target
  // where the dynamic equations (which divide by vx) would be singular.
  return vel * std::tan(steer) / wheelbase_;
}

int SimModelTaigaDyn::lateral_substeps(double vel, double dt) const
{
  // Largest lateral eigenvalue magnitude (the stiff direction). Use the same
  // floored speed as the dynamic branch so the estimate is finite at low speed.
  const double vx_eff = std::max(std::abs(vel), vx_min_dyn_);
  const double lambda_vy = (cf_ + cr_) / (mass_ * vx_eff);
  const double lambda_wz = (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / (inertia_z_ * vx_eff);
  const double lambda_max = std::max(lambda_vy, lambda_wz);
  // Explicit Euler is stable for λ·h < 2; target λ·h < 1 for margin.
  const int n = static_cast<int>(std::ceil(dt * lambda_max));
  return std::clamp(n, 1, 64);
}

double SimModelTaigaDyn::getX() { return state_(IDX::X); }

double SimModelTaigaDyn::getY() { return state_(IDX::Y); }

double SimModelTaigaDyn::getYaw() { return state_(IDX::YAW); }

double SimModelTaigaDyn::getVx() { return state_(IDX::VX); }

double SimModelTaigaDyn::getVy() { return state_(IDX::VY); }

double SimModelTaigaDyn::getAx() { return state_(IDX::ACCX); }

double SimModelTaigaDyn::getWz() { return state_(IDX::WZ); }

double SimModelTaigaDyn::getSteer()
{
  // return measured value with bias added to the actual tire angle
  return state_(IDX::STEER) + steer_bias_;
}

void SimModelTaigaDyn::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  acc_input_queue_.push_back(input_(IDX_U::PEDAL_ACCX_DES));
  delayed_input(IDX_U::PEDAL_ACCX_DES) = acc_input_queue_.front();
  acc_input_queue_.pop_front();
  steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();
  delayed_input(IDX_U::GEAR) = input_(IDX_U::GEAR);
  delayed_input(IDX_U::SLOPE_ACCX) = input_(IDX_U::SLOPE_ACCX);

  const auto prev_state = state_;
  // The lateral (vy, wz) block is stiff at low speed; integrate the outer dt as
  // several explicit-Euler substeps so it stays stable across the speed range.
  const int n_sub = lateral_substeps(state_(IDX::VX), dt);
  const double h = dt / static_cast<double>(n_sub);
  for (int i = 0; i < n_sub; ++i) {
    updateEuler(h, delayed_input);
  }

  // take velocity limit explicitly
  state_(IDX::VX) = std::max(-vx_lim_, std::min(state_(IDX::VX), vx_lim_));

  if (
    prev_state(IDX::VX) * state_(IDX::VX) <= 0.0 &&
    -state_(IDX::PEDAL_ACCX) >= std::abs(delayed_input(IDX_U::SLOPE_ACCX))) {
    // stop condition is satisfied
    state_(IDX::VX) = 0.0;
  }

  state_(IDX::ACCX) = (state_(IDX::VX) - prev_state(IDX::VX)) / dt;
}

void SimModelTaigaDyn::initializeInputQueue(const double & dt)
{
  size_t acc_input_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
  acc_input_queue_.resize(acc_input_queue_size);
  std::fill(acc_input_queue_.begin(), acc_input_queue_.end(), 0.0);

  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  steer_input_queue_.resize(steer_input_queue_size);
  std::fill(steer_input_queue_.begin(), steer_input_queue_.end(), 0.0);
}

void SimModelTaigaDyn::setInputQueues(
  const std::deque<double> & acc_queue, const std::deque<double> & steer_queue)
{
  acc_input_queue_ = acc_queue;
  steer_input_queue_ = steer_queue;
}

int SimModelTaigaDyn::getAccQueueSize() const
{
  return static_cast<int>(acc_input_queue_.size());
}

int SimModelTaigaDyn::getSteerQueueSize() const
{
  return static_cast<int>(steer_input_queue_.size());
}

Eigen::VectorXd SimModelTaigaDyn::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };

  const double vel = sat(state(IDX::VX), vx_lim_, -vx_lim_);
  const double pedal_acc = sat(state(IDX::PEDAL_ACCX), vx_rate_lim_, -vx_rate_lim_);
  const double yaw = state(IDX::YAW);
  const double steer = state(IDX::STEER);  // actual tire angle (no bias) drives the physics
  const double vy = state(IDX::VY);
  const double wz = state(IDX::WZ);
  const double pedal_acc_des =
    sat(input(IDX_U::PEDAL_ACCX_DES), vx_rate_lim_, -vx_rate_lim_) * debug_acc_scaling_factor_;
  const double steer_des =
    sat(input(IDX_U::STEER_DES), steer_lim_, -steer_lim_) * debug_steer_scaling_factor_;

  // NOTE: `steer_des` is calculated by control from measured values, so the diff is
  // taken against the measured steer (actual + bias), mirroring the geared longitudinal model.
  const double measured_steer = steer + steer_bias_;
  const double steer_diff = measured_steer - steer_des;
  const double steer_diff_with_dead_band = std::invoke([&]() {
    if (steer_diff > steer_dead_band_) {
      return steer_diff - steer_dead_band_;
    } else if (steer_diff < -steer_dead_band_) {
      return steer_diff + steer_dead_band_;
    } else {
      return 0.0;
    }
  });
  const double steer_rate =
    sat(-steer_diff_with_dead_band / steer_time_constant_, steer_rate_lim_, -steer_rate_lim_);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);

  // Planar kinematics including the lateral velocity component.
  d_state(IDX::X) = vel * std::cos(yaw) - vy * std::sin(yaw);
  d_state(IDX::Y) = vel * std::sin(yaw) + vy * std::cos(yaw);
  d_state(IDX::YAW) = wz;

  // Longitudinal channel (gear-aware, identical to the geared longitudinal model).
  d_state(IDX::VX) = [&] {
    if (pedal_acc >= 0.0) {
      using autoware_vehicle_msgs::msg::GearCommand;
      const auto gear = input(IDX_U::GEAR);
      if (gear == GearCommand::NONE || gear == GearCommand::PARK) {
        return 0.0;
      } else if (gear == GearCommand::NEUTRAL) {
        return input(IDX_U::SLOPE_ACCX);
      } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
        return -pedal_acc + input(IDX_U::SLOPE_ACCX);
      } else {
        return pedal_acc + input(IDX_U::SLOPE_ACCX);
      }
    } else {
      if (vel > 0.0) {
        return pedal_acc + input(IDX_U::SLOPE_ACCX);
      } else if (vel < 0.0) {
        return -pedal_acc + input(IDX_U::SLOPE_ACCX);
      } else if (-pedal_acc >= std::abs(input(IDX_U::SLOPE_ACCX))) {
        return 0.0;
      } else {
        return input(IDX_U::SLOPE_ACCX);
      }
    }
  }();

  d_state(IDX::STEER) = steer_rate;
  d_state(IDX::PEDAL_ACCX) = -(pedal_acc - pedal_acc_des) / acc_time_constant_;

  // Lateral channel: linear dynamic bicycle above the speed threshold, kinematic
  // relaxation below it (the dynamic terms are singular as vx -> 0).
  if (std::abs(vel) > vx_min_dyn_) {
    const double vx_eff = std::abs(vel);
    const double a11 = -(cf_ + cr_) / (mass_ * vx_eff);
    const double a12 = (lr_ * cr_ - lf_ * cf_) / (mass_ * vx_eff) - vel;
    const double a21 = (lr_ * cr_ - lf_ * cf_) / (inertia_z_ * vx_eff);
    const double a22 = -(lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / (inertia_z_ * vx_eff);
    const double b1 = cf_ / mass_;
    const double b2 = lf_ * cf_ / inertia_z_;
    d_state(IDX::VY) = a11 * vy + a12 * wz + b1 * steer;
    d_state(IDX::WZ) = a21 * vy + a22 * wz + b2 * steer;
  } else {
    const double wz_kin = calc_kinematic_yaw_rate(vel, steer);
    d_state(IDX::VY) = (0.0 - vy) / steer_time_constant_;
    d_state(IDX::WZ) = (wz_kin - wz) / steer_time_constant_;
  }

  return d_state;
}

}  // namespace autoware::simulator::simple_planning_simulator
