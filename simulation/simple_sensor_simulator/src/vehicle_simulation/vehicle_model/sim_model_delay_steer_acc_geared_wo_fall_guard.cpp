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
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_delay_steer_acc_geared_wo_fall_guard.hpp>

namespace autoware::simulator::simple_planning_simulator
{

SimModelDelaySteerAccGearedWoFallGuard::SimModelDelaySteerAccGearedWoFallGuard(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double dt, double acc_delay, double acc_time_constant, double steer_delay,
  double steer_time_constant, double steer_dead_band, double steer_bias,
  double debug_acc_scaling_factor, double debug_steer_scaling_factor, double k_us,
  double brake_time_constant, double lon_drag_c0, double lon_drag_c1, double lon_drag_c2,
  double lon_lat_coupling)
: SimModelInterface(7 /* dim x */, 4 /* dim u */),
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
  k_us_(k_us),
  // brake_time_constant <= 0 keeps the single-tau behaviour (== acc_time_constant).
  brake_time_constant_(
    brake_time_constant > 0.0 ? std::max(brake_time_constant, MIN_TIME_CONSTANT)
                              : std::max(acc_time_constant, MIN_TIME_CONSTANT)),
  lon_drag_c0_(lon_drag_c0),
  lon_drag_c1_(lon_drag_c1),
  lon_drag_c2_(lon_drag_c2),
  lon_lat_coupling_(lon_lat_coupling)
{
  initializeInputQueue(dt);
}

double SimModelDelaySteerAccGearedWoFallGuard::calc_yaw_rate(double vel, double steer) const
{
  // Augmented-bicycle yaw rate with yaw-alignment bias. With k_us = 0 and steer_bias_ = 0 this is
  // exactly the ideal kinematic bicycle, so existing setups remain bit-for-bit identical.
  const double denom = wheelbase_ + k_us_ * vel * vel;
  return vel * std::tan(steer + steer_bias_) / denom;
}

double SimModelDelaySteerAccGearedWoFallGuard::calc_drag(double vel) const
{
  // Steady-state running resistance / drag offset poly(v) added to the accel target.
  return lon_drag_c0_ + lon_drag_c1_ * vel + lon_drag_c2_ * vel * vel;
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
  // Steer bias now enters the yaw equation (calc_yaw_rate, as the viewer's β), not the
  // measured-steer/tracking loop, so that the bias produces a net yaw offset instead of being
  // cancelled by the steer controller. The reported/tracked steer is the actual state value.
  return state_(IDX::STEER);
}

void SimModelDelaySteerAccGearedWoFallGuard::update(const double & dt)
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
  updateEuler(dt, delayed_input);
  // we cannot use updateRungeKutta() because the differentiability or the continuity condition is
  // not satisfied, but we can use Runge-Kutta method with code reconstruction.

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

void SimModelDelaySteerAccGearedWoFallGuard::initializeInputQueue(const double & dt)
{
  size_t acc_input_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
  acc_input_queue_.resize(acc_input_queue_size);
  std::fill(acc_input_queue_.begin(), acc_input_queue_.end(), 0.0);

  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  steer_input_queue_.resize(steer_input_queue_size);
  std::fill(steer_input_queue_.begin(), steer_input_queue_.end(), 0.0);
}

void SimModelDelaySteerAccGearedWoFallGuard::setInputQueues(
  const std::deque<double> & acc_queue, const std::deque<double> & steer_queue)
{
  acc_input_queue_ = acc_queue;
  steer_input_queue_ = steer_queue;
}

int SimModelDelaySteerAccGearedWoFallGuard::getAccQueueSize() const
{
  return static_cast<int>(acc_input_queue_.size());
}

int SimModelDelaySteerAccGearedWoFallGuard::getSteerQueueSize() const
{
  return static_cast<int>(steer_input_queue_.size());
}

Eigen::VectorXd SimModelDelaySteerAccGearedWoFallGuard::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };

  const double vel = sat(state(IDX::VX), vx_lim_, -vx_lim_);
  const double pedal_acc = sat(state(IDX::PEDAL_ACCX), vx_rate_lim_, -vx_rate_lim_);
  const double yaw = state(IDX::YAW);
  const double steer = state(IDX::STEER);
  const double pedal_acc_des =
    sat(input(IDX_U::PEDAL_ACCX_DES), vx_rate_lim_, -vx_rate_lim_) * debug_acc_scaling_factor_;
  const double steer_des =
    sat(input(IDX_U::STEER_DES), steer_lim_, -steer_lim_) * debug_steer_scaling_factor_;

  // Yaw rate (with k_us understeer and steer-bias β); reused for the corner-coupling term below.
  const double yaw_rate = calc_yaw_rate(vel, steer);
  // Acceleration target with running-resistance offset poly(v) and corner coupling c·(vx·ω)²,
  // matching the verification viewer's a_target = a_cmd + poly(v) + c·(vx·ω)². The actuator
  // (PEDAL_ACCX) tracks this target with a first-order lag whose time constant is split between
  // throttle (a_cmd >= 0) and brake (a_cmd < 0). All extra terms vanish when their coefficients
  // are zero, leaving the original single-tau behaviour.
  const double lat_acc = vel * yaw_rate;
  const double pedal_acc_target =
    pedal_acc_des + calc_drag(vel) + lon_lat_coupling_ * lat_acc * lat_acc;
  const double acc_tau = (pedal_acc_des >= 0.0) ? acc_time_constant_ : brake_time_constant_;
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
  d_state(IDX::YAW) = yaw_rate;
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
  d_state(IDX::PEDAL_ACCX) = -(pedal_acc - pedal_acc_target) / acc_tau;

  return d_state;
}

}  // namespace autoware::simulator::simple_planning_simulator
