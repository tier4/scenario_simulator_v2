// Copyright 2026 TIER IV, Inc.
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

// C wrapper around scenario_simulator's SimModelInterface vehicle models.
//
// SimModelInterface 派生を直接使い、複数モデル種別 (ideal_steer_acc /
// delay_steer_acc_geared_wo_fall_guard / ... をケース別解析で切り替える。
//
// API:
//   factory: vm_create_<type>(...)        → VmModel *
//   common : vm_set_input / vm_step / vm_step_dt / vm_get_x/y/yaw/vx/vy/steer/ax/wz / vm_destroy
//   reset  : vm_reset_full / vm_reset_state
//   delay  : vm_set_queues / vm_get_acc_q_size / vm_get_steer_q_size
//            (delay 系派生のみ動作。ideal 系では no-op / 0 を返す)
//
// このラッパーは ament_target_dependencies(autoware_vehicle_msgs) と
// sim_model_interface.cpp / 各派生 .cpp を link する必要がある (CMakeLists.txt 参照)。

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

#include <autoware_vehicle_msgs/msg/gear_command.hpp>

#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_delay_steer_acc_geared_for_diffusion_planner.hpp>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_delay_steer_acc_geared_wo_fall_guard.hpp>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_ideal_steer_acc.hpp>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_interface.hpp>

namespace
{
using autoware::simulator::simple_planning_simulator::SimModelDelaySteerAccGearedForDiffusionPlanner;
using autoware::simulator::simple_planning_simulator::SimModelDelaySteerAccGearedWoFallGuard;
using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
}  // namespace

enum class VmModelType {
  IDEAL_STEER_ACC = 0,
  DELAY_STEER_ACC_GEARED_WO_FALL_GUARD = 1,
  DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER = 2,
};

struct VmModel
{
  VmModelType type;
  std::unique_ptr<SimModelInterface> impl;
  double sub_dt;
  // delay 系用。reset_full の warmup と vm_set_queues の
  // バイアス計算で使う。
  double steer_bias = 0.0;
};

// --- internal helpers ---------------------------------------------------
// delay queue を持つ派生を横断して扱うためのヘルパ。
// SimModelInterface には queue API が無いため、queue API を公開する派生へ
// dynamic_cast し、見つかった派生に対し fn を適用する (見つからなければ dflt)。
template <typename R, typename Fn>
static R for_queue_model(VmModel * m, R dflt, Fn fn)
{
  if (auto * w = dynamic_cast<SimModelDelaySteerAccGearedWoFallGuard *>(m->impl.get())) {
    return fn(w);
  }
  if (auto * d = dynamic_cast<SimModelDelaySteerAccGearedForDiffusionPlanner *>(m->impl.get())) {
    return fn(d);
  }
  return dflt;
}

static int acc_queue_size(VmModel * m)
{
  return for_queue_model(m, 0, [](auto * model) { return model->getAccQueueSize(); });
}

static int steer_queue_size(VmModel * m)
{
  return for_queue_model(m, 0, [](auto * model) { return model->getSteerQueueSize(); });
}

static void set_input_queues(
  VmModel * m, const std::deque<double> & acc_dq, const std::deque<double> & steer_dq)
{
  for_queue_model(m, 0, [&](auto * model) {
    model->setInputQueues(acc_dq, steer_dq);
    return 0;  // ideal 系等は queue 無し → no-op
  });
}

// 構築済み state ベクタ s で delay 系モデルをリセットし、定常入力で delay queue を充填する
// (state 設定 → 定常入力で queue warmup → state 復元)。delay 系派生で共通。
static void warmup_delay_queues(
  VmModel * m, const Eigen::VectorXd & s, double ax, double steer_state)
{
  m->impl->setState(s);
  m->impl->setGear(GearCommand::DRIVE);

  Eigen::VectorXd wu(4);
  wu << ax, static_cast<double>(GearCommand::DRIVE), 0.0, steer_state;
  m->impl->setInput(wu);
  const int warmup = std::max(acc_queue_size(m), steer_queue_size(m));
  for (int i = 0; i < warmup; ++i) m->impl->update(m->sub_dt);

  m->impl->setState(s);  // restore actual state (queues now contain warm-up history)
}

// full-RHS delay 派生 (for_diffusion_planner) のみ: state 設定後に遅延状態履歴バッファを
// 現在 state で定数充填する。command queue の warmup とは別で、遅延状態は「窓前は定常」と
// して seed する (warmup の過渡ではなく)。他モデルでは no-op。
static void reset_state_queues(VmModel * m)
{
  if (auto * d = dynamic_cast<SimModelDelaySteerAccGearedForDiffusionPlanner *>(m->impl.get())) {
    d->resetStateQueues();
  }
}

// ============================================================
// C interface
// ============================================================
extern "C" {

// ---- factories (model-specific) ---------------------------------------

VmModel * vm_create_ideal_steer_acc(double wheelbase, double sub_dt)
{
  auto * m = new VmModel{};
  m->type = VmModelType::IDEAL_STEER_ACC;
  m->impl = std::make_unique<SimModelIdealSteerAcc>(wheelbase);
  m->sub_dt = sub_dt;
  m->steer_bias = 0.0;
  return m;
}

VmModel * vm_create_delay_steer_acc_geared_wo_fall_guard(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double sub_dt, double acc_delay, double acc_time_constant, double steer_delay,
  double steer_time_constant, double steer_dead_band, double steer_bias,
  double debug_acc_scaling_factor, double debug_steer_scaling_factor, double k_us)
{
  auto * m = new VmModel{};
  m->type = VmModelType::DELAY_STEER_ACC_GEARED_WO_FALL_GUARD;
  m->impl = std::make_unique<SimModelDelaySteerAccGearedWoFallGuard>(
    vx_lim, steer_lim, vx_rate_lim, steer_rate_lim, wheelbase, sub_dt, acc_delay,
    acc_time_constant, steer_delay, steer_time_constant, steer_dead_band, steer_bias,
    debug_acc_scaling_factor, debug_steer_scaling_factor, k_us);
  m->sub_dt = sub_dt;
  m->steer_bias = steer_bias;
  return m;
}

// full-RHS delay 派生。差分はステア・加速度の遅延が指令のみ → 右辺全体 (状態フィードバックも t-d)
// になった点。全遅延が 0 なら wo_fall_guard と bit 一致。
VmModel * vm_create_delay_steer_acc_geared_for_diffusion_planner(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double sub_dt, double acc_delay, double acc_time_constant, double steer_delay,
  double steer_time_constant, double steer_dead_band, double steer_bias,
  double debug_acc_scaling_factor, double debug_steer_scaling_factor, double k_us,
  double xy_heading_rate_coeff, double use_rk4)
{
  auto * m = new VmModel{};
  m->type = VmModelType::DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER;
  m->impl = std::make_unique<SimModelDelaySteerAccGearedForDiffusionPlanner>(
    vx_lim, steer_lim, vx_rate_lim, steer_rate_lim, wheelbase, sub_dt, acc_delay,
    acc_time_constant, steer_delay, steer_time_constant, steer_dead_band, steer_bias,
    debug_acc_scaling_factor, debug_steer_scaling_factor, k_us,
    xy_heading_rate_coeff, use_rk4 > 0.5);
  m->sub_dt = sub_dt;
  m->steer_bias = steer_bias;
  return m;
}

void vm_destroy(VmModel * m) { delete m; }

// ---- input --------------------------------------------------------------

void vm_set_input(VmModel * m, double accel_des, double steer_des)
{
  Eigen::VectorXd u;
  switch (m->type) {
    case VmModelType::IDEAL_STEER_ACC:
      // IDX_U: AX_DES=0, STEER_DES=1
      u.resize(2);
      u << accel_des, steer_des;
      break;
    case VmModelType::DELAY_STEER_ACC_GEARED_WO_FALL_GUARD:
    case VmModelType::DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER:
      // IDX_U: (PEDAL_)ACCX_DES=0, GEAR=1, SLOPE_ACCX=2, STEER_DES=3
      u.resize(4);
      u << accel_des, static_cast<double>(GearCommand::DRIVE), 0.0, steer_des;
      break;
  }
  m->impl->setInput(u);
  m->impl->setGear(GearCommand::DRIVE);
}

// ---- integration --------------------------------------------------------

void vm_step(VmModel * m) { m->impl->update(m->sub_dt); }

// 任意 dt で 1 step 進める。端数積分 (interval < sub_dt) に使う。
// 注意 (delay 系派生): update() は dt に依らず必ず delay queue を 1 tick (= 構築時 sub_dt
// 相当) push/pop するため、dt < sub_dt で呼ぶと「物理時間は dt 秒」「遅延 queue は sub_dt
// 秒分」進むという位相ずれが発生する。run_rollout は各反復冒頭で vm_set_queues により
// queue を実履歴で再構築するため累積はしないが、ステップ内では delay 応答が ~sub_dt だけ
// 短く見える系統誤差を持つ。
void vm_step_dt(VmModel * m, double dt) { m->impl->update(dt); }

// ---- state reset (full = state + delay-queue warmup) -------------------
// 末尾の wz/vy 引数は既存ABIとの互換性のため受け取り、現在のモデルでは無視される。

void vm_reset_full(
  VmModel * m, double x, double y, double yaw, double vx, double steer_actual, double ax,
  double, double)
{
  switch (m->type) {
    case VmModelType::IDEAL_STEER_ACC: {
      Eigen::VectorXd s(4);
      s << x, y, yaw, vx;
      m->impl->setState(s);
      break;
    }
    case VmModelType::DELAY_STEER_ACC_GEARED_WO_FALL_GUARD: {
      const double steer_state = steer_actual - m->steer_bias;
      Eigen::VectorXd s(7);
      s << x, y, yaw, vx, steer_state, ax, ax;
      warmup_delay_queues(m, s, ax, steer_state);
      break;
    }
    case VmModelType::DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER: {
      const double steer_state = steer_actual - m->steer_bias;
      Eigen::VectorXd s(7);
      s << x, y, yaw, vx, steer_state, ax, ax;
      warmup_delay_queues(m, s, ax, steer_state);
      // command queue は warmup 履歴で充填済み。遅延状態履歴は warmup の過渡ではなく復元後の
      // 定常 state で seed し直す (full-RHS delay 用)。
      reset_state_queues(m);
      break;
    }
  }
}

// ---- state reset (state only; queues untouched) ------------------------

void vm_reset_state(
  VmModel * m, double x, double y, double yaw, double vx, double steer_actual, double ax,
  double, double)
{
  switch (m->type) {
    case VmModelType::IDEAL_STEER_ACC: {
      Eigen::VectorXd s(4);
      s << x, y, yaw, vx;
      m->impl->setState(s);
      break;
    }
    case VmModelType::DELAY_STEER_ACC_GEARED_WO_FALL_GUARD: {
      const double steer_state = steer_actual - m->steer_bias;
      Eigen::VectorXd s(7);
      s << x, y, yaw, vx, steer_state, ax, ax;
      m->impl->setState(s);
      m->impl->setGear(GearCommand::DRIVE);
      break;
    }
    case VmModelType::DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER: {
      const double steer_state = steer_actual - m->steer_bias;
      Eigen::VectorXd s(7);
      s << x, y, yaw, vx, steer_state, ax, ax;
      m->impl->setState(s);
      m->impl->setGear(GearCommand::DRIVE);
      // state のみリセット時も遅延状態履歴を現在 state で seed し直す (command queue は不変)。
      reset_state_queues(m);
      break;
    }
  }
}

// ---- delay queue API (delay 系派生のみ動作) ----------------------------

void vm_set_queues(
  VmModel * m, const double * acc_q, int n_acc, const double * steer_q, int n_steer)
{
  std::deque<double> acc_dq(acc_q, acc_q + n_acc);
  std::deque<double> steer_dq(steer_q, steer_q + n_steer);
  set_input_queues(m, acc_dq, steer_dq);  // ideal 系等は queue 無し → no-op
}

int vm_get_acc_q_size(VmModel * m) { return acc_queue_size(m); }

int vm_get_steer_q_size(VmModel * m) { return steer_queue_size(m); }

// ---- getters (SimModelInterface virtual 経由) --------------------------

double vm_get_x(VmModel * m) { return m->impl->getX(); }
double vm_get_y(VmModel * m) { return m->impl->getY(); }
double vm_get_yaw(VmModel * m) { return m->impl->getYaw(); }
double vm_get_vx(VmModel * m) { return m->impl->getVx(); }
double vm_get_vy(VmModel * m) { return m->impl->getVy(); }
double vm_get_steer(VmModel * m) { return m->impl->getSteer(); }
double vm_get_ax(VmModel * m) { return m->impl->getAx(); }
// yaw rate (wz) はkinematicモデルでsteer/vxから導出する。
double vm_get_wz(VmModel * m) { return m->impl->getWz(); }

// ---- batch integration for open-loop tuning hotpath --------------------
// Integrate n_intervals command intervals from k0, snapping state at each
// horizon checkpoint.  All array arguments are full-dataset pointers; the
// k0 argument shifts the base index so per-k0 slicing in Python is avoided.
//
// horizons[0..n_horizons-1]: sorted ascending interval-tick offsets (e.g. [20,40]).
//   A checkpoint fires after completing interval j when horizons[h_idx]==j+1.
// n_full[k0+j]: integer sub-steps for interval j (int32).
// rem[k0+j]:    fractional remainder [s]; integrated if > rem_eps.
// steer_out:    raw getSteer() value (= steer_state + steer_bias; matches vm_get_steer).
void vm_integrate_to_horizons(
  VmModel * m,
  int n_intervals,
  int k0,
  const double * accel_des,
  const double * steer_des,
  const int * n_full,
  const double * rem,
  double rem_eps,
  int n_horizons,
  const int * horizons,
  double * x_out,
  double * y_out,
  double * yaw_out,
  double * vx_out,
  double * ax_out,
  double * steer_out)
{
  // Build type-specific input vector template once.
  Eigen::VectorXd u;
  bool is_ideal = false;
  switch (m->type) {
    case VmModelType::IDEAL_STEER_ACC:
      u.resize(2);
      is_ideal = true;
      break;
    default:
      u.resize(4);
      u(1) = static_cast<double>(GearCommand::DRIVE);
      u(2) = 0.0;
      break;
  }
  m->impl->setGear(GearCommand::DRIVE);

  int h_idx = 0;
  for (int j = 0; j < n_intervals && h_idx < n_horizons; ++j) {
    const int idx = k0 + j;
    if (is_ideal) {
      u(0) = accel_des[idx];
      u(1) = steer_des[idx];
    } else {
      u(0) = accel_des[idx];
      u(3) = steer_des[idx];
    }
    m->impl->setInput(u);

    const int nf = n_full[idx];
    for (int s = 0; s < nf; ++s) {
      m->impl->update(m->sub_dt);
    }
    const double r = rem[idx];
    if (r > rem_eps) {
      m->impl->update(r);
    }

    // Snap at horizon checkpoints (horizons are 1-indexed interval counts).
    while (h_idx < n_horizons && horizons[h_idx] == j + 1) {
      x_out[h_idx]     = m->impl->getX();
      y_out[h_idx]     = m->impl->getY();
      yaw_out[h_idx]   = m->impl->getYaw();
      vx_out[h_idx]    = m->impl->getVx();
      ax_out[h_idx]    = m->impl->getAx();
      steer_out[h_idx] = m->impl->getSteer();
      ++h_idx;
    }
  }
}

}  // extern "C"
