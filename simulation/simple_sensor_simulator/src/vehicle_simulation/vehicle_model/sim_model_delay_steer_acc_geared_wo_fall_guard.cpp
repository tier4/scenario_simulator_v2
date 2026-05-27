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

  // =========================================================================
  // 🌟【欠陥Aの解消（Level 1.5）】入力段階でのアクセルとブレーキ信号の完全分離
  // =========================================================================
  const double raw_pedal_cmd = input_(IDX_U::PEDAL_ACCX_DES);

  if (raw_pedal_cmd >= 0.0) {
    // 加速指令：アクセルキューには指令値を、ブレーキキューには「全離し(0.0)」を入れる
    acc_input_queue_.push_back(raw_pedal_cmd);
    brake_input_queue_.push_back(0.0);
  } else {
    // 制動指令：アクセルキューには「全離し(0.0)」を、ブレーキキューには指令値を入れる
    acc_input_queue_.push_back(0.0);
    brake_input_queue_.push_back(raw_pedal_cmd);
  }

  // それぞれの遅延時間が経過した値をキューから取り出す
  const double acc_delayed_val = acc_input_queue_.front();
  acc_input_queue_.pop_front();
  const double brake_delayed_val = brake_input_queue_.front();
  brake_input_queue_.pop_front();

  // 💡 ブレーキ・オーバーライド（BOS）論理による結合
  // 万が一、遅延のタイミング差で「アクセル」と「ブレーキ」が同時に出てきた場合は、
  // 実車の安全機構と同じく「ブレーキの指令」を優先して採用する。
  if (brake_delayed_val < -1e-5) {
    delayed_input(IDX_U::PEDAL_ACCX_DES) = brake_delayed_val;
  } else {
    // ブレーキが出ていない時は、アクセルの値（0.0のコースティング状態も含む）を採用
    delayed_input(IDX_U::PEDAL_ACCX_DES) = acc_delayed_val;
  }
  // =========================================================================

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
  double pedal_acc_des = delayed_input(IDX_U::PEDAL_ACCX_DES) * debug_acc_scaling_factor_;

  bool is_brake_pad_contacting = false;

  // 挿入：ベースラインをすべてのジャンプ処理の基準として先行計算
  double baseline_acc = acc_offset_ - brake_offset_;

  if (pedal_acc_des < 0.0) {
    double brake_cmd = std::abs(pedal_acc_des);

    if (brake_resolution_ > 1e-5) {
      brake_cmd = std::round(brake_cmd / brake_resolution_) * brake_resolution_;
    }

    // 🌟 挿入：足を完全に離した時はヒステリシスを0に戻す
    double hist_cmd = 0.0;
    if (brake_cmd < 1e-5) {
      hist_cmd = 0.0;
    } else {
      hist_cmd = std::clamp(brake_hysteresis_state_, brake_cmd - (brake_hysteresis_width_ / 2.0), brake_cmd + (brake_hysteresis_width_ / 2.0));
    }
    hist_cmd = std::max(0.0, hist_cmd);
    brake_hysteresis_state_ = hist_cmd;

    double jump_cmd = 0.0;
    if (hist_cmd > brake_dead_band_) {
      // 🌟 挿入：不感帯を抜けた＝パッドが接触している！
      is_brake_pad_contacting = true;

      double deadzoned_cmd = hist_cmd - brake_dead_band_;
      jump_cmd = deadzoned_cmd + brake_jump_value_;
      jump_cmd = jump_cmd * (1.0 + brake_accuracy_error_);

      // 🌟 挿入：実際のジャンプ値（誤差込み）を計算し、ワープの到達点とする
      double actual_jump_value = brake_jump_value_ * (1.0 + brake_accuracy_error_);
      double apply_jump_target = baseline_acc - actual_jump_value;

      if (state_(IDX::PEDAL_ACCX) <= baseline_acc && state_(IDX::PEDAL_ACCX) > apply_jump_target) {
        state_(IDX::PEDAL_ACCX) = apply_jump_target;
      }
    }

    pedal_acc_des = -jump_cmd;
  } else {
    brake_hysteresis_state_ = 0.0;

    if (acc_resolution_ > 1e-5) {
      pedal_acc_des = std::round(pedal_acc_des / acc_resolution_) * acc_resolution_;
    }

    if (pedal_acc_des > acc_dead_band_) {
      pedal_acc_des = pedal_acc_des - acc_dead_band_;

      pedal_acc_des = pedal_acc_des * (1.0 + acc_accuracy_error_);
    } else {
      pedal_acc_des = 0.0;
    }
  }

  // 🌟 挿入：離す時も、誤差込みの実際のジャンプ値を使って残存摩擦を判定する
  if (!is_brake_pad_contacting) {
    double actual_jump_value = brake_jump_value_ * (1.0 + brake_accuracy_error_);
    if (state_(IDX::PEDAL_ACCX) < baseline_acc && state_(IDX::PEDAL_ACCX) >= baseline_acc - actual_jump_value) {
      state_(IDX::PEDAL_ACCX) = baseline_acc;
    }
  }

  pedal_acc_des = pedal_acc_des + baseline_acc;

  delayed_input(IDX_U::PEDAL_ACCX_DES) = sat(pedal_acc_des, acc_lim_, -brake_lim_);

  // 2. ステアリング フィルタ
  double steer_des = delayed_input(IDX_U::STEER_DES) * debug_steer_scaling_factor_;

  if (steer_resolution_ > 1e-5) {
    steer_des = std::round(steer_des / steer_resolution_) * steer_resolution_;
  }

  const double current_motor_angle = (state_(IDX::STEER) - steer_bias_) / (1.0 + steer_accuracy_error_);
  double steer_hist = std::clamp(current_motor_angle, steer_des - (steer_hysteresis_width_ / 2.0), steer_des + (steer_hysteresis_width_ / 2.0));

  delayed_input(IDX_U::STEER_DES) = sat(steer_hist, steer_lim_, -steer_lim_);
  // =========================================================================

  const auto prev_state = state_;

  // 🌟 物理演算を高精度なルンゲ＝クッタ法（RK4）に切り替え
  updateRungeKutta(dt, delayed_input);

  // 速度制限と停止判定
  state_(IDX::VX) = std::max(-vx_lim_, std::min(state_(IDX::VX), vx_lim_));

  // 🌟 タイヤの物理的な可動限界は、モーターの限界（steer_lim_）にギア比とバイアスが乗った値になる
  const double tire_steer_upper_lim = steer_lim_ * (1.0 + steer_accuracy_error_) + steer_bias_;
  const double tire_steer_lower_lim = -steer_lim_ * (1.0 + steer_accuracy_error_) + steer_bias_;
  // 念のため上下限の逆転を防ぐ安全策を施してクランプ
  state_(IDX::STEER) = sat(
    state_(IDX::STEER),
    std::max(tire_steer_upper_lim, tire_steer_lower_lim),
    std::min(tire_steer_upper_lim, tire_steer_lower_lim)
  );

  state_(IDX::PEDAL_ACCX) = sat(state_(IDX::PEDAL_ACCX), acc_lim_, -brake_lim_);

  // 🌟 挿入：ゼロ・スナップ処理（浮動小数点誤差のクリーニング）
  // =========================================================================
  // RK4の積分結果、速度が極めてゼロに近づいた場合は完全に 0.0 に丸める
  // （ADKのステート遷移スタックを防止するための措置）
  const double snap_epsilon = 0.001;
  if (delayed_input(IDX_U::PEDAL_ACCX_DES) < 0.0) { // ブレーキ指令が出ている時
    if (std::abs(state_(IDX::VX)) < snap_epsilon) {
      state_(IDX::VX) = 0.0;
      // 微小な位置のドリフトも固定する
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

  // 📡 フェーズ7のセンサー計算をここに引っ越し（1ステップに1回だけ確定させる）
  if (std::abs(raw_delayed_vx) < 1e-3) {
    delayed_vx_ = 0.0;
  } else {
    double vx = raw_delayed_vx * (1.0 + vel_sensor_accuracy_error_);
    vx += vel_sensor_offset_;
    if (vel_sensor_noise_stddev_ > 1e-5) {
      vx += vel_dist_(vel_rng_) * vel_sensor_noise_stddev_; // サイコロを振るのはここだけ！
    }
    if (vel_sensor_resolution_ > 1e-5) {
      vx = std::round(vx / vel_sensor_resolution_) * vel_sensor_resolution_;
    }
    delayed_vx_ = vx;
  }
}

void SimModelDelaySteerAccGearedWoFallGuard::initializeInputQueue(const double & dt)
{
  size_t acc_input_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
  acc_input_queue_.resize(acc_input_queue_size);
  size_t brake_input_queue_size = static_cast<size_t>(round(brake_delay_ / dt));
  brake_input_queue_.resize(brake_input_queue_size);

  double initial_acc_cmd = 0.0;
  double initial_brake_cmd = 0.0;

  if (state_(IDX::PEDAL_ACCX) > 0.0) {
    initial_acc_cmd = (state_(IDX::PEDAL_ACCX) / (1.0 + acc_accuracy_error_)) + acc_dead_band_;
  }
  else if (state_(IDX::PEDAL_ACCX) < 0.0) {
    double jump_cmd = std::abs(state_(IDX::PEDAL_ACCX));
    double deadzoned_cmd = (jump_cmd / (1.0 + brake_accuracy_error_)) - brake_jump_value_;
    double brake_cmd_abs = std::max(0.0, deadzoned_cmd) + brake_dead_band_;
    initial_brake_cmd = -brake_cmd_abs;
  }

  std::fill(acc_input_queue_.begin(), acc_input_queue_.end(), initial_acc_cmd);
  std::fill(brake_input_queue_.begin(), brake_input_queue_.end(), initial_brake_cmd);
  brake_hysteresis_state_ = std::abs(initial_brake_cmd);

  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  steer_input_queue_.resize(steer_input_queue_size);
  const double initial_steer_cmd = (state_(IDX::STEER) - steer_bias_) / (1.0 + steer_accuracy_error_);
  std::fill(steer_input_queue_.begin(), steer_input_queue_.end(), initial_steer_cmd);

  size_t vel_input_queue_size = static_cast<size_t>(std::round(vel_sensor_delay_ / dt));
  vel_history_queue_.resize(vel_input_queue_size);
  std::fill(vel_history_queue_.begin(), vel_history_queue_.end(), state_(IDX::VX));
  delayed_vx_ = state_(IDX::VX);
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

  // =========================================================================
  // 指令値の「正・0・負」および「現在のペダル状態」による3パターン分離
  // =========================================================================
  // ゼロ判定のための閾値（1e-5）
  constexpr double eps = 1e-5;

  const double current_tc = std::invoke([&]() {
    if (pedal_acc_des > (acc_offset_ + eps)) {
      // 【パターン1：正（踏み込み加速）】➔ 純粋なアクセル動特性
      return acc_time_constant_;
    }
    else if (pedal_acc_des < (-brake_offset_ - eps)) {
      // 【パターン2：負（踏み込み制動）】➔ 純粋なブレーキ作動動特性
      return brake_time_constant_;
    }
    else {
      // 【パターン3：ゼロ（ペダル全離し・コースティング）】
      // 💡 指令は0だが、現在の車両状態（pedal_acc）を見て、残っている力が抜けるスピードを決める
      if (pedal_acc < 0.0) {
        // 現在ブレーキが残っているなら、ブレーキの油圧・空圧が抜けるスピードを適用
        return brake_time_constant_;
      } else {
        // 現在アクセル（推力）が残っているなら、エンジン回転が落ちるスピードを適用
        return acc_time_constant_;
      }
    }
  });

  const double current_jerk_lim = std::invoke([&]() {
    if (pedal_acc_des > (acc_offset_ + eps)) {
      return acc_rate_lim_;
    }
    else if (pedal_acc_des < (-brake_offset_ - eps)) {
      return brake_rate_lim_;
    }
    else {
      // 指令が0のとき、現在残っている力に合わせて変化率の制限（ジャークリミット）を切り替える
      return (pedal_acc < 0.0) ? brake_rate_lim_ : acc_rate_lim_;
    }
  });
  // =========================================================================

  // 真のタイヤ角度(state)から、バイアスを引きギア比で割って、モーター位置(u)を逆算する
  const double current_steer_with_bias = (steer - steer_bias_) / (1.0 + steer_accuracy_error_);
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
    using autoware_vehicle_msgs::msg::GearCommand;
    const auto gear = input(IDX_U::GEAR);
    if (gear == GearCommand::NONE || gear == GearCommand::PARK) {
      return 0.0;
    }

    // 1. 空気抵抗（速度の2乗に比例し、常に進行方向と逆向きに働く力）
    const double air_drag = -air_drag_coef_ * vel * std::abs(vel);

    // 2. エンジン推力（アクセルペダルが踏まれている時のみ、ギア方向に従って発生）
    double engine_acc = 0.0;
    if (pedal_acc >= 0.0) {
      if (gear == GearCommand::NEUTRAL) {
        engine_acc = 0.0;
      } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
        engine_acc = -pedal_acc;
      } else {
        engine_acc = pedal_acc;
      }
    }
      // 🌟 挿入：静止摩擦モデル（クーロン摩擦の近似）
      // =========================================================================
      // vel_epsilon: 速度をゼロに引き込む仮想バネの強さを決めるスケール
      const double vel_epsilon = 0.02;
      const double k = 1.0 / vel_epsilon; // 仮想的なバネ定数

    // 3. 車体が持つ「最大静止摩擦力」（ブレーキ踏力 ＋ 常に働く転がり抵抗）
    const double brake_force = (pedal_acc < 0.0) ? -pedal_acc : 0.0;
    const double friction_limit = brake_force + rolling_resistance_;

    // 理想の摩擦力（エンジン推力、坂道重力、空気抵抗をすべて相殺し、車速をゼロに引き込む力）
    double ideal_friction = -engine_acc - input(IDX_U::SLOPE_ACCX) - air_drag - (k * vel);

    // 実際の摩擦力は、限界値（ブレーキ＋転がり抵抗）の範囲内で発揮される
    double actual_friction = std::clamp(ideal_friction, -friction_limit, friction_limit);

    // 4. 最終的な加速度の合算（ニュートンの運動方程式）
    return engine_acc + input(IDX_U::SLOPE_ACCX) + air_drag + actual_friction;
  }();

  const double raw_acc_rate = -(pedal_acc - pedal_acc_des) / current_tc;
  const double pedal_acc_rate = sat(raw_acc_rate, current_jerk_lim, -current_jerk_lim);

  d_state(IDX::STEER) = steer_rate * (1.0 + steer_accuracy_error_);
  d_state(IDX::PEDAL_ACCX) = pedal_acc_rate;

  return d_state;
}

}  // namespace autoware::simulator::simple_planning_simulator
