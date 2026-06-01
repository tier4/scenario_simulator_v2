// C wrapper around scenario_simulator's SimModelInterface vehicle models.
//
// SimModelInterface 派生を直接使い、複数モデル種別 (ideal_steer_acc /
// delay_steer_acc_geared_wo_fall_guard / ...) をケース別解析で切り替える。
//
// API:
//   factory: vm_create_<type>(...)        → VmModel *
//   common : vm_set_input / vm_step / vm_step_dt / vm_get_x/y/yaw/vx/steer/ax/wz / vm_destroy
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

#include <autoware_vehicle_msgs/msg/gear_command.hpp>

#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_delay_steer_acc_geared_wo_fall_guard.hpp>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_ideal_steer_acc.hpp>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_interface.hpp>

namespace
{
using autoware::simulator::simple_planning_simulator::SimModelDelaySteerAccGearedWoFallGuard;
using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
}  // namespace

enum class VmModelType {
  IDEAL_STEER_ACC = 0,
  DELAY_STEER_ACC_GEARED_WO_FALL_GUARD = 1,
};

struct VmModel
{
  VmModelType type;
  std::unique_ptr<SimModelInterface> impl;
  double sub_dt;
  // wo_fall_guard 用 (reset_full の warmup と vm_set_queues のバイアス計算で使う)
  double steer_bias = 0.0;
};

// --- internal helpers ---
static SimModelDelaySteerAccGearedWoFallGuard * as_wo_fall_guard(VmModel * m)
{
  return dynamic_cast<SimModelDelaySteerAccGearedWoFallGuard *>(m->impl.get());
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
      // IDX_U: PEDAL_ACCX_DES=0, GEAR=1, SLOPE_ACCX=2, STEER_DES=3
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
// 注意 (delay 系派生): SimModelDelaySteerAccGearedWoFallGuard::update() は dt に
// 依らず必ず delay queue を 1 tick (= 構築時 sub_dt 相当) push/pop するため、
// dt < sub_dt で呼ぶと「物理時間は dt 秒」「遅延 queue は sub_dt 秒分」進むという
// 位相ずれが発生する。analyze_per_step.run_per_step は各反復冒頭で
// vm_set_queues により queue を実履歴で再構築するため累積はしないが、ステップ内
// では delay 応答が ~sub_dt だけ短く見える系統誤差を持つ。
void vm_step_dt(VmModel * m, double dt) { m->impl->update(dt); }

// ---- state reset (full = state + delay-queue warmup) -------------------

void vm_reset_full(
  VmModel * m, double x, double y, double yaw, double vx, double steer_actual, double ax)
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

      // warmup with steady-state input fills the delay queues
      Eigen::VectorXd wu(4);
      wu << ax, static_cast<double>(GearCommand::DRIVE), 0.0, steer_state;
      m->impl->setInput(wu);
      auto * w = as_wo_fall_guard(m);
      const int warmup = std::max(w->getAccQueueSize(), w->getSteerQueueSize());
      for (int i = 0; i < warmup; ++i) m->impl->update(m->sub_dt);

      // restore actual state (queues now contain warm-up history)
      m->impl->setState(s);
      break;
    }
  }
}

// ---- state reset (state only; queues untouched) ------------------------

void vm_reset_state(
  VmModel * m, double x, double y, double yaw, double vx, double steer_actual, double ax)
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
  }
}

// ---- delay queue API (delay 系派生のみ動作) ----------------------------

void vm_set_queues(
  VmModel * m, const double * acc_q, int n_acc, const double * steer_q, int n_steer)
{
  auto * w = as_wo_fall_guard(m);
  if (!w) return;  // ideal 系等は queue 無し → no-op
  std::deque<double> acc_dq(acc_q, acc_q + n_acc);
  std::deque<double> steer_dq(steer_q, steer_q + n_steer);
  w->setInputQueues(acc_dq, steer_dq);
}

int vm_get_acc_q_size(VmModel * m)
{
  auto * w = as_wo_fall_guard(m);
  return w ? w->getAccQueueSize() : 0;
}

int vm_get_steer_q_size(VmModel * m)
{
  auto * w = as_wo_fall_guard(m);
  return w ? w->getSteerQueueSize() : 0;
}

// ---- getters (SimModelInterface virtual 経由) --------------------------

double vm_get_x(VmModel * m) { return m->impl->getX(); }
double vm_get_y(VmModel * m) { return m->impl->getY(); }
double vm_get_yaw(VmModel * m) { return m->impl->getYaw(); }
double vm_get_vx(VmModel * m) { return m->impl->getVx(); }
double vm_get_steer(VmModel * m) { return m->impl->getSteer(); }
double vm_get_ax(VmModel * m) { return m->impl->getAx(); }
// yaw rate (wz)。k_us 依存の calc_yaw_rate を経由するため、per-step でも understeer の
// 寄与を直接観測できる (steer/位置指標は k_us 非感度なのに対し wz は感度を持つ)。
double vm_get_wz(VmModel * m) { return m->impl->getWz(); }

}  // extern "C"
