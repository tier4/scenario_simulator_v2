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
  double vx_lim, double acc_lim, double brake_lim, double acc_rate_lim, double brake_rate_lim, double steer_lim, double steer_rate_lim, double wheelbase,
  double dt, double acc_delay, double brake_delay, double acc_time_constant, double brake_time_constant,
  double acc_accuracy_error, double brake_accuracy_error, double brake_hysteresis_width, double acc_dead_band, double brake_dead_band, double brake_jump_value, double acc_offset, double brake_offset, double acc_resolution, double brake_resolution,
  double steer_delay,
  double steer_time_constant, double steer_dead_band, double steer_bias,
  double steer_accuracy_error, double steer_resolution, double steer_hysteresis_width,
  double vel_sensor_delay, double vel_sensor_resolution, double vel_sensor_noise_stddev, int vel_sensor_noise_seed, double vel_sensor_accuracy_error, double vel_sensor_offset,
  double debug_acc_scaling_factor, double debug_steer_scaling_factor)
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
  prev_brake_cmd_(0.0),
  prev_steer_cmd_(0.0),
  delayed_vx_(0.0),
  vel_rng_(vel_sensor_noise_seed),
  vel_dist_(0.0, 1.0)
{
  initializeInputQueue(dt);
}

double SimModelDelaySteerAccGearedWoFallGuard::getX() { return state_(IDX::X); }

double SimModelDelaySteerAccGearedWoFallGuard::getY() { return state_(IDX::Y); }

double SimModelDelaySteerAccGearedWoFallGuard::getYaw() { return state_(IDX::YAW); }

double SimModelDelaySteerAccGearedWoFallGuard::getVx()
{
  double vx = delayed_vx_;
  vx = vx * (1.0 + vel_sensor_accuracy_error_);
  vx += vel_sensor_offset_;
  if (vel_sensor_noise_stddev_ > 1e-5) {
    vx += vel_dist_(vel_rng_) * vel_sensor_noise_stddev_;
  }
  if (vel_sensor_resolution_ > 1e-5) {
    vx = std::round(vx / vel_sensor_resolution_) * vel_sensor_resolution_;
  }
  return vx;
}
double SimModelDelaySteerAccGearedWoFallGuard::getVy() { return 0.0; }

double SimModelDelaySteerAccGearedWoFallGuard::getAx() { return state_(IDX::ACCX); }

double SimModelDelaySteerAccGearedWoFallGuard::getWz()
{
  return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / wheelbase_;
}

double SimModelDelaySteerAccGearedWoFallGuard::getSteer()
{

  return state_(IDX::STEER) + steer_bias_;
}

void SimModelDelaySteerAccGearedWoFallGuard::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  acc_input_queue_.push_back(input_(IDX_U::PEDAL_ACCX_DES));
  brake_input_queue_.push_back(input_(IDX_U::PEDAL_ACCX_DES));

  const double acc_delayed_val = acc_input_queue_.front();
  acc_input_queue_.pop_front();
  const double brake_delayed_val = brake_input_queue_.front();
  brake_input_queue_.pop_front();

  if (acc_delayed_val >= 0.0) {
    delayed_input(IDX_U::PEDAL_ACCX_DES) = acc_delayed_val;
  } else {
    delayed_input(IDX_U::PEDAL_ACCX_DES) = brake_delayed_val;
  }

  steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();
  delayed_input(IDX_U::GEAR) = input_(IDX_U::GEAR);
  delayed_input(IDX_U::SLOPE_ACCX) = input_(IDX_U::SLOPE_ACCX);

  // =========================================================================
  // 🌟 非線形フィルタ計算（デジタルの世界）
  // =========================================================================
  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };

  // 1. アクセル・ブレーキ フィルタ
  double pedal_acc_des = sat(delayed_input(IDX_U::PEDAL_ACCX_DES), acc_lim_, -brake_lim_) * debug_acc_scaling_factor_;
  if (pedal_acc_des < 0.0) {
    double brake_cmd = std::abs(pedal_acc_des);
    brake_cmd = brake_cmd * (1.0 + brake_accuracy_error_);

    double hist_cmd = std::clamp(prev_brake_cmd_, brake_cmd - (brake_hysteresis_width_ / 2.0), brake_cmd + (brake_hysteresis_width_ / 2.0));
    hist_cmd = std::max(0.0, hist_cmd);
    prev_brake_cmd_ = hist_cmd;

    double jump_cmd = 0.0;
    if (hist_cmd > brake_dead_band_) {
      // 空振りした分（不感帯）を引き算して捨てる（アクセルと同じ処理！）
      double deadzoned_cmd = hist_cmd - brake_dead_band_;

      // パッドが触れた瞬間の反力（Jump）を足して出力とする
      jump_cmd = deadzoned_cmd + brake_jump_value_;
    }

    double res_cmd = jump_cmd;
    if (brake_resolution_ > 1e-5) {
      res_cmd = std::round(jump_cmd / brake_resolution_) * brake_resolution_;
    }
    pedal_acc_des = -res_cmd;

    pedal_acc_des = pedal_acc_des - brake_offset_;
  } else {
    prev_brake_cmd_ = 0.0;

    pedal_acc_des = pedal_acc_des * (1.0 + acc_accuracy_error_);

    if (pedal_acc_des > acc_dead_band_) {
      pedal_acc_des = pedal_acc_des - acc_dead_band_;
    } else {
      pedal_acc_des = 0.0;
    }

    if (acc_resolution_ > 1e-5) {
      pedal_acc_des = std::round(pedal_acc_des / acc_resolution_) * acc_resolution_;
    }

    pedal_acc_des = pedal_acc_des + acc_offset_;
  }
  delayed_input(IDX_U::PEDAL_ACCX_DES) = pedal_acc_des;

  // 2. ステアリング フィルタ
  double steer_des = sat(delayed_input(IDX_U::STEER_DES), steer_lim_, -steer_lim_) * debug_steer_scaling_factor_;
  steer_des *= (1.0 + steer_accuracy_error_);

  double steer_hist = std::clamp(prev_steer_cmd_, steer_des - (steer_hysteresis_width_ / 2.0), steer_des + (steer_hysteresis_width_ / 2.0));
  prev_steer_cmd_ = steer_hist;

  if (steer_resolution_ > 1e-5) {
    steer_hist = std::round(steer_hist / steer_resolution_) * steer_resolution_;
  }
  delayed_input(IDX_U::STEER_DES) = steer_hist;
  // =========================================================================

  const auto prev_state = state_;

  // 🌟【最終改修1】真のブレーキジャンプ（踏み込みと抜きの両方）
  if (delayed_input(IDX_U::PEDAL_ACCX_DES) <= -brake_jump_value_) {
    // 踏み込み時：物理加速度がジャンプ値に達していない場合、即座に引き下げる
    if (state_(IDX::PEDAL_ACCX) > -brake_jump_value_) {
      state_(IDX::PEDAL_ACCX) = -brake_jump_value_;
    }
  } else if (delayed_input(IDX_U::PEDAL_ACCX_DES) > -brake_jump_value_) {
    // 抜き時：ブレーキ指令が完全にゼロ（不感帯内）に戻ったら、引きずりを即座にゼロにする
    if (state_(IDX::PEDAL_ACCX) < 0.0 && state_(IDX::PEDAL_ACCX) >= -brake_jump_value_) {
      state_(IDX::PEDAL_ACCX) = 0.0;
    }
  }

  // 🌟 物理演算を高精度なルンゲ＝クッタ法（RK4）に切り替え
  updateRungeKutta(dt, delayed_input);

  // 速度制限と停止判定
  state_(IDX::VX) = std::max(-vx_lim_, std::min(state_(IDX::VX), vx_lim_));

  state_(IDX::STEER) = sat(state_(IDX::STEER), steer_lim_, -steer_lim_);
  state_(IDX::PEDAL_ACCX) = sat(state_(IDX::PEDAL_ACCX), acc_lim_, -brake_lim_);
  if (
    prev_state(IDX::VX) * state_(IDX::VX) <= 0.0 &&
    -state_(IDX::PEDAL_ACCX) >= std::abs(delayed_input(IDX_U::SLOPE_ACCX))) {
    state_(IDX::VX) = 0.0;
  }

  const auto apply_hsa_stop = [&]() {
    state_(IDX::VX) = 0.0;
    state_(IDX::X) = prev_state(IDX::X);
    state_(IDX::Y) = prev_state(IDX::Y);
    state_(IDX::YAW) = prev_state(IDX::YAW);
  };

  using autoware_vehicle_msgs::msg::GearCommand;
  const auto gear = delayed_input(IDX_U::GEAR);
  if (
    gear == GearCommand::DRIVE || gear == GearCommand::DRIVE_2 || gear == GearCommand::DRIVE_3 ||
    gear == GearCommand::DRIVE_4 || gear == GearCommand::DRIVE_5 || gear == GearCommand::DRIVE_6 ||
    gear == GearCommand::DRIVE_7 || gear == GearCommand::DRIVE_8 || gear == GearCommand::DRIVE_9 ||
    gear == GearCommand::DRIVE_10 || gear == GearCommand::DRIVE_11 ||
    gear == GearCommand::DRIVE_12 || gear == GearCommand::DRIVE_13 ||
    gear == GearCommand::DRIVE_14 || gear == GearCommand::DRIVE_15 ||
    gear == GearCommand::DRIVE_16 || gear == GearCommand::DRIVE_17 ||
    gear == GearCommand::DRIVE_18 || gear == GearCommand::LOW || gear == GearCommand::LOW_2) {
    if (state_(IDX::VX) < 0.0) apply_hsa_stop();
  } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
    if (state_(IDX::VX) > 0.0) apply_hsa_stop();
  } else if (gear == GearCommand::PARK) {
    apply_hsa_stop();
  }

  state_(IDX::ACCX) = (state_(IDX::VX) - prev_state(IDX::VX)) / dt;

  if (vel_history_queue_.empty()) {
    delayed_vx_ = state_(IDX::VX);
  } else {
    vel_history_queue_.push_back(state_(IDX::VX));
    delayed_vx_ = vel_history_queue_.front();
    vel_history_queue_.pop_front();
  }
}

void SimModelDelaySteerAccGearedWoFallGuard::initializeInputQueue(const double & dt)
{
  size_t acc_input_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
  acc_input_queue_.resize(acc_input_queue_size);
  std::fill(acc_input_queue_.begin(), acc_input_queue_.end(), 0.0);

  size_t brake_input_queue_size = static_cast<size_t>(round(brake_delay_ / dt));
  brake_input_queue_.resize(brake_input_queue_size);
  std::fill(brake_input_queue_.begin(), brake_input_queue_.end(), 0.0);

  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  steer_input_queue_.resize(steer_input_queue_size);
  std::fill(steer_input_queue_.begin(), steer_input_queue_.end(), 0.0);

  size_t vel_input_queue_size = static_cast<size_t>(std::round(vel_sensor_delay_ / dt));
  vel_history_queue_.resize(vel_input_queue_size);
  std::fill(vel_history_queue_.begin(), vel_history_queue_.end(), 0.0);
}

Eigen::VectorXd SimModelDelaySteerAccGearedWoFallGuard::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };

  const double vel = sat(state(IDX::VX), vx_lim_, -vx_lim_);
  const double pedal_acc = sat(state(IDX::PEDAL_ACCX), acc_lim_, -brake_lim_);
  const double yaw = state(IDX::YAW);
  const double steer = state(IDX::STEER);

  // 🌟 update()で計算済みの固定指令値を使用
  const double pedal_acc_des = input(IDX_U::PEDAL_ACCX_DES);
  const double steer_des = input(IDX_U::STEER_DES);
  const double current_tc = (pedal_acc_des < 0.0) ? brake_time_constant_ : acc_time_constant_;
  const double current_jerk_lim = (pedal_acc_des < 0.0) ? brake_rate_lim_ : acc_rate_lim_;

  // 🌟 RK4の中間状態(state)を反映するため直接バイアスを足す
  const double current_steer_with_bias = state(IDX::STEER) + steer_bias_;
  const double steer_diff = current_steer_with_bias - steer_des;

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
  d_state(IDX::YAW) = vel * std::tan(steer) / wheelbase_;
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
        // ブレーキが負けて転がり落ちる場合でも、ブレーキ力(pedal_acc < 0)を抵抗として計算する
        if (input(IDX_U::SLOPE_ACCX) > 0.0) {
          return input(IDX_U::SLOPE_ACCX) + pedal_acc;
        } else {
          return input(IDX_U::SLOPE_ACCX) - pedal_acc;
        }
      }
    }
  }();

  const double raw_acc_rate = -(pedal_acc - pedal_acc_des) / current_tc;
  const double pedal_acc_rate = sat(raw_acc_rate, current_jerk_lim, -current_jerk_lim);

  d_state(IDX::STEER) = steer_rate;
  d_state(IDX::PEDAL_ACCX) = pedal_acc_rate;

  return d_state;
}

}  // namespace autoware::simulator::simple_planning_simulator
