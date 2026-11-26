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
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_delay_steer_acc_geared_wo_fall_guard.hpp>

namespace autoware::simulator::simple_planning_simulator
{

SimModelDelaySteerAccGearedWoFallGuard::SimModelDelaySteerAccGearedWoFallGuard(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double dt, double acc_delay, double acc_time_constant, double steer_delay,
  double steer_time_constant, double steer_dead_band, double steer_bias,
  double debug_acc_scaling_factor, double debug_steer_scaling_factor, double k_us, double dec_delay,
  double dec_time_constant, double debug_dec_scaling_factor)
: SimModelInterface(8 /* dim x */, 4 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  acc_delay_(acc_delay),
  acc_time_constant_(std::max(acc_time_constant, MIN_TIME_CONSTANT)),
  dec_delay_(dec_delay),
  dec_time_constant_(std::max(dec_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT)),
  steer_dead_band_(steer_dead_band),
  steer_bias_(steer_bias),
  debug_acc_scaling_factor_(std::max(debug_acc_scaling_factor, 0.0)),
  debug_dec_scaling_factor_(std::max(debug_dec_scaling_factor, 0.0)),
  debug_steer_scaling_factor_(std::max(debug_steer_scaling_factor, 0.0)),
  k_us_(k_us)
{
  initializeInputQueue(dt);
}

double SimModelDelaySteerAccGearedWoFallGuard::calc_yaw_rate(double vel, double steer) const
{
  const double denom = wheelbase_ + k_us_ * vel * vel;
  return vel * std::tan(steer) / denom;
}

double SimModelDelaySteerAccGearedWoFallGuard::calc_acc_in_body(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input) const
{
  using autoware_vehicle_msgs::msg::GearCommand;
  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };
  // Each channel is clipped on its own: the acceleration limit is a property of the vehicle, not
  // of one actuator, so neither channel may spend the other's headroom.
  const double acc_pedal_acc = sat(state(IDX::ACC_ACCX), vx_rate_lim_, -vx_rate_lim_);
  const auto gear = input(IDX_UD::UD_GEAR);
  if (gear == GearCommand::NEUTRAL || gear == GearCommand::NONE || gear == GearCommand::PARK) {
    return 0.0;
  } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
    return -acc_pedal_acc;
  } else {
    return acc_pedal_acc;
  }
}

double SimModelDelaySteerAccGearedWoFallGuard::calc_dec_acc(const Eigen::VectorXd & state) const
{
  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };
  return sat(state(IDX::DEC_ACCX), vx_rate_lim_, -vx_rate_lim_);
}

bool SimModelDelaySteerAccGearedWoFallGuard::can_hold_at_standstill(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input) const
{
  const double unbalanced = calc_acc_in_body(state, input) + input(IDX_UD::UD_SLOPE_ACCX);
  return std::abs(unbalanced) <= -calc_dec_acc(state);
}

double SimModelDelaySteerAccGearedWoFallGuard::getX() { return state_(IDX::X); }

double SimModelDelaySteerAccGearedWoFallGuard::getY() { return state_(IDX::Y); }

double SimModelDelaySteerAccGearedWoFallGuard::getYaw() { return state_(IDX::YAW); }

double SimModelDelaySteerAccGearedWoFallGuard::getVx() { return state_(IDX::VX); }

double SimModelDelaySteerAccGearedWoFallGuard::getVy() { return 0.0; }

double SimModelDelaySteerAccGearedWoFallGuard::getAx() { return state_(IDX::ACCX); }

double SimModelDelaySteerAccGearedWoFallGuard::getWz()
{
  return calc_yaw_rate(state_(IDX::VX), state_(IDX::STEER));
}

double SimModelDelaySteerAccGearedWoFallGuard::getSteer()
{
  // return measured values with bias added to actual values
  return state_(IDX::STEER) + steer_bias_;
}

void SimModelDelaySteerAccGearedWoFallGuard::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(IDX_UD::UD_DIM);

  // The pedal acceleration command is split into an accel channel and a decel channel, and each
  // keeps its own dead time from here on. The split is additive because max(u, 0) + min(u, 0) == u
  // for every u. The two channels are *not* recombined: they stay apart through their own
  // first-order lags and through the gear and direction handling in `calcModel()`, so nothing
  // downstream has to recover which actuator is acting.
  //
  // The split point is fixed at zero on purpose. A non-zero threshold d would put +d on the accel
  // channel and -d on the decel channel even for a zero command, which cancels in the sum but not
  // in the targets once the two gains differ: a residual d * (s_acc - s_dec) would be left at zero
  // command. The real accel/brake boundary is velocity dependent anyway -- it comes out of the
  // accel and brake maps in `autoware_raw_vehicle_cmd_converter` -- so a constant would be the
  // wrong shape for it even if it were harmless.
  const double pedal_accx_des = input_(IDX_U::PEDAL_ACCX_DES);
  acc_input_queue_.push_back(std::max(pedal_accx_des, 0.0));
  dec_input_queue_.push_back(std::min(pedal_accx_des, 0.0));
  delayed_input(IDX_UD::UD_ACC_ACCX_DES) = acc_input_queue_.front();
  delayed_input(IDX_UD::UD_DEC_ACCX_DES) = dec_input_queue_.front();
  acc_input_queue_.pop_front();
  dec_input_queue_.pop_front();

  steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_UD::UD_STEER_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();

  delayed_input(IDX_UD::UD_GEAR) = input_(IDX_U::GEAR);
  delayed_input(IDX_UD::UD_SLOPE_ACCX) = input_(IDX_U::SLOPE_ACCX);

  const auto prev_state = state_;
  updateEuler(dt, delayed_input);
  // we cannot use updateRungeKutta() because the differentiability or the continuity condition is
  // not satisfied, but we can use Runge-Kutta method with code reconstruction.

  // take velocity limit explicitly
  state_(IDX::VX) = std::max(-vx_lim_, std::min(state_(IDX::VX), vx_lim_));

  // NOTE: the test uses `prev_state`, the state the Euler step was evaluated at, so that the
  // velocity update and the decision about whether the vehicle may stop are made from the same
  // actuator values rather than from values half a step apart. It asks the same question as the
  // zero-velocity case of `calcModel()`, so a brake that carries the velocity through zero parks
  // it there instead of chattering across zero.
  if (
    prev_state(IDX::VX) * state_(IDX::VX) <= 0.0 &&
    can_hold_at_standstill(prev_state, delayed_input)) {
    // stop condition is satisfied
    state_(IDX::VX) = 0.0;
  }

  state_(IDX::ACCX) = (state_(IDX::VX) - prev_state(IDX::VX)) / dt;
}

void SimModelDelaySteerAccGearedWoFallGuard::initializeInputQueue(const double & dt)
{
  size_t acc_input_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
  acc_input_queue_.resize(acc_input_queue_size);
  std::fill(acc_input_queue_.begin(), acc_input_queue_.end(), 0.0);

  size_t dec_input_queue_size = static_cast<size_t>(round(dec_delay_ / dt));
  dec_input_queue_.resize(dec_input_queue_size);
  std::fill(dec_input_queue_.begin(), dec_input_queue_.end(), 0.0);

  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  steer_input_queue_.resize(steer_input_queue_size);
  std::fill(steer_input_queue_.begin(), steer_input_queue_.end(), 0.0);
}

Eigen::VectorXd SimModelDelaySteerAccGearedWoFallGuard::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };

  const double vel = sat(state(IDX::VX), vx_lim_, -vx_lim_);
  const double acc_pedal_acc = sat(state(IDX::ACC_ACCX), vx_rate_lim_, -vx_rate_lim_);
  const double dec_pedal_acc = calc_dec_acc(state);
  const double yaw = state(IDX::YAW);
  const double steer = state(IDX::STEER);
  // NOTE: the clip is applied *after* the gain, not before. Clipping first and scaling after lets
  // a gain above one place the target outside the acceleration limit; since the derivative below
  // uses the clipped state, the raw state then integrates past the limit without bound and the
  // output stays pinned at the limit long after the command is released.
  // Each channel relaxes toward its own target with its own gain and its own time constant. No
  // branch is needed, and none is taken: when both actuators are commanded at once, which happens
  // while one dead time has expired and the other has not, both simply act.
  const double acc_pedal_acc_des =
    sat(input(IDX_UD::UD_ACC_ACCX_DES) * debug_acc_scaling_factor_, vx_rate_lim_, -vx_rate_lim_);
  const double dec_pedal_acc_des =
    sat(input(IDX_UD::UD_DEC_ACCX_DES) * debug_dec_scaling_factor_, vx_rate_lim_, -vx_rate_lim_);
  const double steer_des =
    sat(input(IDX_UD::UD_STEER_DES), steer_lim_, -steer_lim_) * debug_steer_scaling_factor_;
  // NOTE: `steer_des` is calculated by control from measured values. getSteer() also gets the
  // measured value. The steer_rate used in the motion calculation is obtained from these
  // differences.
  const double steer_diff = getSteer() - steer_des;
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

  d_state(IDX::X) = vel * cos(yaw);
  d_state(IDX::Y) = vel * sin(yaw);
  d_state(IDX::YAW) = calc_yaw_rate(vel, steer);
  // Each channel is carried into the body frame on its own before the two are added. The accel
  // channel is gated by the gear, because a drivetrain produces nothing in neutral and pushes the
  // other way in reverse. The decel channel is not gated by the gear: a brake always opposes the
  // direction of travel. Deciding this from the sum instead discards a brake that acts while the
  // accel state has not yet decayed -- in neutral the sum is then positive and the whole term is
  // dropped, so a fully applied brake produces no deceleration at all.
  d_state(IDX::VX) = [&] {
    using autoware_vehicle_msgs::msg::GearCommand;
    const auto gear = input(IDX_UD::UD_GEAR);
    const double slope_acc = input(IDX_UD::UD_SLOPE_ACCX);

    // NONE and PARK hold the vehicle outright, as before.
    if (gear == GearCommand::NONE || gear == GearCommand::PARK) {
      return 0.0;
    }

    const double acc_in_body = calc_acc_in_body(state, input);

    // `dec_pedal_acc` is at most zero, so adding it while moving forward and subtracting it while
    // moving backward always slows the vehicle down.
    if (vel > 0.0) {
      return acc_in_body + dec_pedal_acc + slope_acc;
    } else if (vel < 0.0) {
      return acc_in_body - dec_pedal_acc + slope_acc;
    } else {
      // At a standstill the brake holds whatever the drive and the slope would otherwise do, up
      // to its own magnitude. Anything beyond that is what starts the vehicle moving.
      if (can_hold_at_standstill(state, input)) {
        return 0.0;
      }
      const double unbalanced = acc_in_body + slope_acc;
      return unbalanced - std::copysign(-dec_pedal_acc, unbalanced);
    }
  }();
  d_state(IDX::STEER) = steer_rate;
  d_state(IDX::ACC_ACCX) = -(acc_pedal_acc - acc_pedal_acc_des) / acc_time_constant_;
  d_state(IDX::DEC_ACCX) = -(dec_pedal_acc - dec_pedal_acc_des) / dec_time_constant_;

  return d_state;
}

}  // namespace autoware::simulator::simple_planning_simulator
