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
  double debug_acc_scaling_factor, double debug_steer_scaling_factor, double rolling_resistance, double air_drag_coef, double vel_epsilon)
: SimModelInterface(8 /* dim x */, 4 /* dim u */),
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
  vel_epsilon_(std::max(vel_epsilon, 0.001)),
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
  // 1. 入力信号の遅延バッファ処理（アクセル・ブレーキを独立保持）
  const double raw_pedal_cmd = input_(IDX_U::PEDAL_ACCX_DES);
  acc_input_queue_.push_back((raw_pedal_cmd >= 0.0) ? raw_pedal_cmd : 0.0);
  brake_input_queue_.push_back((raw_pedal_cmd < 0.0) ? std::abs(raw_pedal_cmd) : 0.0);

  const double acc_delayed_val = acc_input_queue_.front();
  acc_input_queue_.pop_front();
  const double brake_delayed_val = brake_input_queue_.front();
  brake_input_queue_.pop_front();

  // ステアリングバッファ処理
  steer_motor_input_queue_.push_back(input_(IDX_U::STEER_DES));
  const double delayed_steer_des = steer_motor_input_queue_.front();
  steer_motor_input_queue_.pop_front();

  // 2. 目標モータ駆動力 (acc_target) の算出
  const double acc_target = [&]() {
    double cmd = acc_delayed_val * debug_acc_scaling_factor_;
    if (acc_resolution_ > 1e-5) {
      cmd = std::round(cmd / acc_resolution_) * acc_resolution_;
    }
    cmd = std::max(0.0, cmd - acc_dead_band_) * (1.0 + acc_accuracy_error_);
    return std::clamp(cmd + acc_offset_, 0.0, acc_lim_);
  }();

  // 3. 目標ブレーキ制動力 (brake_target) の算出
  const double brake_target = [&]() {
    double brake_cmd = brake_delayed_val * debug_acc_scaling_factor_;
    if (brake_resolution_ > 1e-5) {
      brake_cmd = std::round(brake_cmd / brake_resolution_) * brake_resolution_;
    }

    if (brake_cmd < 1e-5) {
      brake_hysteresis_state_ = 0.0;
    } else {
      brake_hysteresis_state_ = std::max(0.0, std::clamp(
        brake_hysteresis_state_,
        brake_cmd - (brake_hysteresis_width_ / 2.0),
        brake_cmd + (brake_hysteresis_width_ / 2.0)
      ));
    }

    double cmd = 0.0;
    if (brake_hysteresis_state_ > brake_dead_band_) {
      cmd = (brake_hysteresis_state_ - brake_dead_band_ + brake_jump_value_) * (1.0 + brake_accuracy_error_);
    }
    return std::clamp(cmd + brake_offset_, 0.0, brake_lim_);
  }();

  // 4. 目標ステアリング角の算出
  const double steer_target = [&]() {
    double cmd = delayed_steer_des * debug_steer_scaling_factor_;

    // 車両ECUのソフトウェア不感帯（スレッショルド型ニュートラルカット）
    if (std::abs(cmd) < steer_dead_band_) {
      cmd = 0.0;
    }

    if (steer_resolution_ > 1e-5) {
      cmd = std::round(cmd / steer_resolution_) * steer_resolution_;
    }
    const double steer_motor_hist = std::clamp(
      (state_(IDX::STEER) - steer_bias_) / (1.0 + steer_accuracy_error_),
      cmd - (steer_hysteresis_width_ / 2.0),
      cmd + (steer_hysteresis_width_ / 2.0)
    );
    return std::clamp(steer_motor_hist, -steer_lim_, steer_lim_);
  }();

  // 5. calcModel へ渡す内部入力ベクトル（5次元）を準備
  Eigen::VectorXd inner_input(5);
  inner_input(IDX_U_INNER::ACC_DES)          = acc_target;
  inner_input(IDX_U_INNER::BRAKE_DES)        = brake_target;
  inner_input(IDX_U_INNER::GEAR_INNER)       = input_(IDX_U::GEAR);
  inner_input(IDX_U_INNER::SLOPE_ACCX_INNER) = input_(IDX_U::SLOPE_ACCX);
  inner_input(IDX_U_INNER::STEER_DES_INNER)  = steer_target;

  const auto prev_state = state_;

  // 6. ルンゲ＝クッタ数値積分（連続微分方程式に基づく計算）
  updateRungeKutta(dt, inner_input);

  // 7. 状態量の物理限界クランプ
  state_(IDX::VX)         = std::clamp(state_(IDX::VX), -vx_lim_, vx_lim_);
  state_(IDX::DRIVE_ACCX) = std::clamp(state_(IDX::DRIVE_ACCX), 0.0, acc_lim_);
  state_(IDX::BRAKE_ACCX) = std::clamp(state_(IDX::BRAKE_ACCX), 0.0, brake_lim_);

  state_(IDX::STEER) = [&]() {
    const double upper = steer_lim_ * (1.0 + steer_accuracy_error_) + steer_bias_;
    const double lower = -steer_lim_ * (1.0 + steer_accuracy_error_) + steer_bias_;
    return std::clamp(state_(IDX::STEER), std::min(upper, lower), std::max(upper, lower));
  }();

  // 🌟 8. 純粋な物理力の平衡判定（静止摩擦モデル）に基づく完全停止フリーズ制御
  constexpr double stop_epsilon = 1e-3;
  if (std::abs(state_(IDX::VX)) < stop_epsilon) {
    using autoware_vehicle_msgs::msg::GearCommand;
    const auto gear = static_cast<uint8_t>(inner_input(IDX_U_INNER::GEAR_INNER));

    const double drive_force = [&]() {
      if (gear == GearCommand::NONE || gear == GearCommand::PARK || gear == GearCommand::NEUTRAL) {
        return 0.0;
      }
      if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
        return -state_(IDX::DRIVE_ACCX);
      }
      return state_(IDX::DRIVE_ACCX);
    }();

    const double air_drag = -air_drag_coef_ * state_(IDX::VX) * std::abs(state_(IDX::VX));
    const double external_acc = drive_force + inner_input(IDX_U_INNER::SLOPE_ACCX_INNER) + air_drag;
    const double friction_limit = state_(IDX::BRAKE_ACCX) + rolling_resistance_;

    // 外力の総和が静止摩擦限界（ブレーキ＋転がり抵抗）に収まっている場合のみ完全静止
    if (std::abs(external_acc) <= friction_limit) {
      state_(IDX::VX) = 0.0;
    }
  }

  // 加速度算出とセンサ遅延処理
  state_(IDX::ACCX) = (state_(IDX::VX) - prev_state(IDX::VX)) / dt;

  const double raw_delayed_vx = [&]() {
    if (vel_history_queue_.empty()) return state_(IDX::VX);
    vel_history_queue_.push_back(state_(IDX::VX));
    const double front_val = vel_history_queue_.front();
    vel_history_queue_.pop_front();
    return front_val;
  }();

  delayed_vx_ = [&]() {
    if (std::abs(raw_delayed_vx) < stop_epsilon) return 0.0;
    double vx = raw_delayed_vx * (1.0 + vel_sensor_accuracy_error_) + vel_sensor_offset_;
    if (vel_sensor_noise_stddev_ > 1e-5) vx += vel_dist_(vel_rng_) * vel_sensor_noise_stddev_;
    if (vel_sensor_resolution_ > 1e-5) vx = std::round(vx / vel_sensor_resolution_) * vel_sensor_resolution_;
    return vx;
  }();
}

Eigen::VectorXd SimModelDelaySteerAccGearedWoFallGuard::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  using autoware_vehicle_msgs::msg::GearCommand;

  // 1. 状態量の抽出
  const double vel       = std::clamp(state(IDX::VX), -vx_lim_, vx_lim_);
  const double drive_acc = std::clamp(state(IDX::DRIVE_ACCX), 0.0, acc_lim_);
  const double brake_acc = std::clamp(state(IDX::BRAKE_ACCX), 0.0, brake_lim_);
  const double yaw       = state(IDX::YAW);

  const double current_steer = [&]() {
    const double upper = steer_lim_ * (1.0 + steer_accuracy_error_) + steer_bias_;
    const double lower = -steer_lim_ * (1.0 + steer_accuracy_error_) + steer_bias_;
    return std::clamp(state(IDX::STEER), std::min(upper, lower), std::max(upper, lower));
  }();

  // 2. 内部入力の抽出
  const double acc_des         = input(IDX_U_INNER::ACC_DES);
  const double brake_des       = input(IDX_U_INNER::BRAKE_DES);
  const auto gear              = static_cast<uint8_t>(input(IDX_U_INNER::GEAR_INNER));
  const double slope_accx      = input(IDX_U_INNER::SLOPE_ACCX_INNER);
  const double steer_motor_des = input(IDX_U_INNER::STEER_DES_INNER);

  // 3. ステアリング偏差計算
  const double current_steer_motor = (current_steer - steer_bias_) / (1.0 + steer_accuracy_error_);
  const double steer_motor_diff = current_steer_motor - steer_motor_des;

  // 🌟 4. 「力の綱引き」運動方程式による前後加速度 d_vx の計算
  const double d_vx = [&] {
    if (gear == GearCommand::NONE || gear == GearCommand::PARK) {
      return 0.0;
    }

    const double engine_acc = [&]() {
      if (gear == GearCommand::NEUTRAL) return 0.0;
      if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) return -drive_acc;
      return drive_acc;
    }();

    const double air_drag = -air_drag_coef_ * vel * std::abs(vel);
    const double external_acc = engine_acc + slope_accx + air_drag;
    const double friction_limit = brake_acc + rolling_resistance_;

    // クーロン摩擦の粘性近傍モデルによる滑らかな運動計算
    const double k = 1.0 / vel_epsilon_; // ハードコードを撤廃

    return std::clamp(-k * vel, external_acc - friction_limit, external_acc + friction_limit);
  }();

  // 🌟 5. 8次元状態微分の構成（条件分岐を完全排除した連続モデル）
  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X)          = vel * std::cos(yaw);
  d_state(IDX::Y)          = vel * std::sin(yaw);
  d_state(IDX::YAW)        = vel * std::tan(current_steer) / wheelbase_;
  d_state(IDX::VX)         = d_vx;
  d_state(IDX::STEER)      = std::clamp(-steer_motor_diff / steer_time_constant_, -steer_rate_lim_, steer_rate_lim_) * (1.0 + steer_accuracy_error_);
  d_state(IDX::ACCX)       = 0.0;

  // モータ駆動トルクおよびブレーキ圧の独立1次遅れ系
  d_state(IDX::DRIVE_ACCX) = std::clamp(-(drive_acc - acc_des) / acc_time_constant_, -acc_rate_lim_, acc_rate_lim_);
  d_state(IDX::BRAKE_ACCX) = std::clamp(-(brake_acc - brake_des) / brake_time_constant_, -brake_rate_lim_, brake_rate_lim_);

  return d_state;
}

void SimModelDelaySteerAccGearedWoFallGuard::initializeInputQueue(const double & dt)
{
  const double initial_drive = state_(IDX::DRIVE_ACCX);
  const double initial_brake = state_(IDX::BRAKE_ACCX);

  const double initial_acc_cmd = (initial_drive > 0.0)
    ? (initial_drive / (1.0 + acc_accuracy_error_)) + acc_dead_band_
    : 0.0;
  const double initial_brake_cmd = (initial_brake > 0.0)
    ? (initial_brake / (1.0 + brake_accuracy_error_)) + brake_dead_band_
    : 0.0;

  const size_t acc_queue_size = static_cast<size_t>(std::round(acc_delay_ / dt));
  acc_input_queue_.assign(acc_queue_size, initial_acc_cmd);
  const size_t brake_queue_size = static_cast<size_t>(std::round(brake_delay_ / dt));
  brake_input_queue_.assign(brake_queue_size, initial_brake_cmd);

  brake_hysteresis_state_ = initial_brake_cmd;

  double initial_steer_motor_cmd = (state_(IDX::STEER) - steer_bias_) / (1.0 + steer_accuracy_error_);

  // 初期化キューのスレッショルド型不感帯処理
  if (std::abs(initial_steer_motor_cmd) < steer_dead_band_) {
    initial_steer_motor_cmd = 0.0;
  }
  const size_t steer_motor_queue_size = static_cast<size_t>(std::round(steer_delay_ / dt));
  steer_motor_input_queue_.assign(steer_motor_queue_size, initial_steer_motor_cmd);

  const double initial_vel = state_(IDX::VX);
  const size_t vel_queue_size = static_cast<size_t>(std::round(vel_sensor_delay_ / dt));
  vel_history_queue_.assign(vel_queue_size, initial_vel);

  delayed_vx_ = initial_vel;
}

}  // namespace autoware::simulator::simple_planning_simulator