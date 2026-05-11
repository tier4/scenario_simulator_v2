// Self-contained C wrapper for DELAY_STEER_ACC_GEARED_WO_FALL_GUARD.
//
// Reproduces the original C++ implementation (sim_model_interface.cpp +
// sim_model_delay_steer_acc_geared_wo_fall_guard.cpp) without any ROS 2
// message dependencies.  Only Eigen is required.
//
// Compile (example):
//   g++ -shared -fPIC -O2 -std=c++17 -I/usr/include/eigen3 \
//       -o libvehicle_model_wrapper.so vehicle_model_c_wrapper.cpp
//
// Delay queue initialization strategy used in vm_reset_full():
//   1. setState to actual vehicle state at t_k
//   2. Run warmup_steps with steady-state input (ax, steer_state) to fill queues
//   3. setState back to original  →  queues have the correct warm-up contents
//   4. Caller then sets actual command and calls vm_step()

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>

// ---- GearCommand constants (from autoware_vehicle_msgs/msg/GearCommand) ----
static constexpr uint8_t GEAR_NONE    =  0;
static constexpr uint8_t GEAR_NEUTRAL =  1;
static constexpr uint8_t GEAR_DRIVE   =  2;  // and DRIVE_2..DRIVE_18
static constexpr uint8_t GEAR_REVERSE = 20;
static constexpr uint8_t GEAR_REVERSE2 = 21;
static constexpr uint8_t GEAR_PARK    = 22;

// ---- State / Input index layout ----------------------------------------
// IDX:   X=0, Y=1, YAW=2, VX=3, STEER=4, ACCX=5, PEDAL_ACCX=6
// IDX_U: PEDAL_ACCX_DES=0, GEAR=1, SLOPE_ACCX=2, STEER_DES=3

struct SimModel {
    // ---- params ----
    double vx_lim_, vx_rate_lim_;
    double steer_lim_, steer_rate_lim_;
    double wheelbase_;
    double acc_tc_, steer_tc_;
    double steer_db_, steer_bias_;
    double dbg_acc_, dbg_steer_;

    // ---- state & input ----
    Eigen::VectorXd state_;   // dim 7
    Eigen::VectorXd input_;   // dim 4
    uint8_t gear_;

    // ---- delay queues ----
    std::deque<double> acc_q_, steer_q_;
    double sub_dt_;
    int acc_q_size_, steer_q_size_;

    // ---- constructor ----
    SimModel(double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim,
             double wheelbase, double sub_dt,
             double acc_delay, double acc_tc,
             double steer_delay, double steer_tc,
             double steer_db, double steer_bias,
             double dbg_acc = 1.0, double dbg_steer = 1.0)
    : vx_lim_(vx_lim), vx_rate_lim_(vx_rate_lim),
      steer_lim_(steer_lim), steer_rate_lim_(steer_rate_lim),
      wheelbase_(wheelbase),
      acc_tc_(std::max(acc_tc, 0.03)),
      steer_tc_(std::max(steer_tc, 0.03)),
      steer_db_(steer_db), steer_bias_(steer_bias),
      dbg_acc_(std::max(dbg_acc, 0.0)), dbg_steer_(std::max(dbg_steer, 0.0)),
      sub_dt_(sub_dt), gear_(GEAR_DRIVE)
    {
        state_ = Eigen::VectorXd::Zero(7);
        input_ = Eigen::VectorXd::Zero(4);

        acc_q_size_   = std::max(0, (int)std::round(acc_delay   / sub_dt));
        steer_q_size_ = std::max(0, (int)std::round(steer_delay / sub_dt));
        acc_q_.assign(acc_q_size_,   0.0);
        steer_q_.assign(steer_q_size_, 0.0);
    }

    void setState(const Eigen::VectorXd & s) { state_ = s; }
    void setInput(const Eigen::VectorXd & u) { input_ = u; }
    void setGear(uint8_t g) { gear_ = g; }

    double getX()     const { return state_(0); }
    double getY()     const { return state_(1); }
    double getYaw()   const { return state_(2); }
    double getVx()    const { return state_(3); }
    double getAx()    const { return state_(5); }
    double getSteer() const { return state_(4) + steer_bias_; }  // internal + bias

    // ---- calcModel (copied from sim_model_delay_steer_acc_geared_wo_fall_guard.cpp) ----
    Eigen::VectorXd calcModel(const Eigen::VectorXd & state,
                              const Eigen::VectorXd & input) const
    {
        auto sat = [](double val, double u, double l) {
            return std::max(std::min(val, u), l);
        };

        const double vel       = sat(state(3), vx_lim_, -vx_lim_);
        const double pedal_acc = sat(state(6), vx_rate_lim_, -vx_rate_lim_);
        const double yaw       = state(2);
        const double steer_st  = state(4);
        const double pa_des    = sat(input(0), vx_rate_lim_, -vx_rate_lim_) * dbg_acc_;
        const double sd_sat    = sat(input(3), steer_lim_, -steer_lim_) * dbg_steer_;

        // steer_diff uses getSteer() = state(4) + bias
        const double steer_measured = steer_st + steer_bias_;
        double steer_diff = steer_measured - sd_sat;
        if      (steer_diff >  steer_db_) steer_diff -= steer_db_;
        else if (steer_diff < -steer_db_) steer_diff += steer_db_;
        else                              steer_diff  = 0.0;
        const double steer_rate = sat(-steer_diff / steer_tc_, steer_rate_lim_, -steer_rate_lim_);

        // longitudinal dynamics (gear-dependent)
        const double gear  = input(1);
        const double slope = input(2);
        double d_vx = 0.0;
        if (pedal_acc >= 0.0) {
            if      (gear == GEAR_NONE || gear == GEAR_PARK) d_vx = 0.0;
            else if (gear == GEAR_NEUTRAL)                   d_vx = slope;
            else if (gear == GEAR_REVERSE || gear == GEAR_REVERSE2)
                                                             d_vx = -pedal_acc + slope;
            else                                             d_vx =  pedal_acc + slope;
        } else {
            if      (vel > 0.0)  d_vx = pedal_acc + slope;
            else if (vel < 0.0)  d_vx = -pedal_acc + slope;
            else if (-pedal_acc >= std::abs(slope)) d_vx = 0.0;
            else                 d_vx = slope;
        }

        Eigen::VectorXd ds = Eigen::VectorXd::Zero(7);
        ds(0) = vel * std::cos(yaw);
        ds(1) = vel * std::sin(yaw);
        ds(2) = vel * std::tan(steer_st) / wheelbase_;
        ds(3) = d_vx;
        ds(4) = steer_rate;
        ds(5) = 0.0;   // ACCX updated after Euler in update()
        ds(6) = -(pedal_acc - pa_des) / acc_tc_;
        return ds;
    }

    // ---- update (Euler, copied from sim_model_delay_steer_acc_geared_wo_fall_guard.cpp) ----
    void update(double dt)
    {
        // delay queue
        Eigen::VectorXd delayed = input_;
        acc_q_.push_back(input_(0));
        delayed(0) = acc_q_.front(); acc_q_.pop_front();
        steer_q_.push_back(input_(3));
        delayed(3) = steer_q_.front(); steer_q_.pop_front();
        delayed(1) = input_(1);  // GEAR (no delay)
        delayed(2) = input_(2);  // SLOPE (no delay)

        const auto prev_state = state_;
        // Euler integration
        state_ += calcModel(state_, delayed) * dt;

        // velocity limit
        state_(3) = std::max(-vx_lim_, std::min(state_(3), vx_lim_));

        // stop condition
        if (prev_state(3) * state_(3) <= 0.0 &&
            -state_(6) >= std::abs(delayed(2)))
        {
            state_(3) = 0.0;
        }

        // ACCX = actual acceleration = Δvx/dt
        state_(5) = (state_(3) - prev_state(3)) / dt;
    }
};

// ---- per-step reset with queue warmup ----
// steer_actual: measured steering angle (getSteer() = internal + bias)
static void reset_full(SimModel * m,
                       double x, double y, double yaw, double vx,
                       double steer_actual, double ax)
{
    double steer_state = steer_actual - m->steer_bias_;

    // Initial state
    Eigen::VectorXd s(7);
    s << x, y, yaw, vx, steer_state, ax, ax;
    m->setState(s);
    m->setGear(GEAR_DRIVE);

    // Warmup with steady-state input to fill delay queues.
    //   acc_queue  ← ax          (so delayed_acc  = ax)
    //   steer_queue← steer_state (so delayed_steer = steer_state)
    Eigen::VectorXd wu_input(4);
    wu_input << ax, (double)GEAR_DRIVE, 0.0, steer_state;
    m->setInput(wu_input);

    int warmup = std::max(m->acc_q_size_, m->steer_q_size_);
    for (int i = 0; i < warmup; i++) {
        m->update(m->sub_dt_);
    }

    // Reset state back to actual  (queues retain warmed-up contents).
    m->setState(s);
}

// ============================================================
// C interface
// ============================================================
extern "C" {

SimModel * vm_create(
    double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim,
    double wheelbase, double sub_dt,
    double acc_delay, double acc_tc,
    double steer_delay, double steer_tc,
    double steer_dead_band, double steer_bias)
{
    return new SimModel(vx_lim, steer_lim, vx_rate_lim, steer_rate_lim,
                        wheelbase, sub_dt,
                        acc_delay, acc_tc,
                        steer_delay, steer_tc,
                        steer_dead_band, steer_bias,
                        1.0, 1.0);
}

void vm_reset_full(SimModel * m,
                   double x, double y, double yaw, double vx,
                   double steer_actual, double ax)
{
    reset_full(m, x, y, yaw, vx, steer_actual, ax);
}

void vm_set_input(SimModel * m, double accel_des, double steer_des)
{
    Eigen::VectorXd u(4);
    u << accel_des, (double)GEAR_DRIVE, 0.0, steer_des;
    m->setInput(u);
}

void vm_step(SimModel * m) { m->update(m->sub_dt_); }

// Reset state only — queues are NOT touched.
// Call vm_set_queues() afterward to supply actual past command history.
void vm_reset_state(SimModel * m,
                    double x, double y, double yaw, double vx,
                    double steer_actual, double ax)
{
    double steer_state = steer_actual - m->steer_bias_;
    Eigen::VectorXd s(7);
    s << x, y, yaw, vx, steer_state, ax, ax;
    m->setState(s);
    m->setGear(GEAR_DRIVE);
}

// Overwrite delay queue contents with actual past command history.
// acc_q[0..n_acc-1]   : accel commands oldest→newest (n_acc == acc_q_size)
// steer_q[0..n_steer-1]: steer commands oldest→newest (n_steer == steer_q_size)
void vm_set_queues(SimModel * m,
                   const double * acc_q,   int n_acc,
                   const double * steer_q, int n_steer)
{
    m->acc_q_.clear();
    for (int i = 0; i < n_acc;   ++i) m->acc_q_.push_back(acc_q[i]);
    m->steer_q_.clear();
    for (int i = 0; i < n_steer; ++i) m->steer_q_.push_back(steer_q[i]);
}

int vm_get_acc_q_size(SimModel * m)   { return m->acc_q_size_; }
int vm_get_steer_q_size(SimModel * m) { return m->steer_q_size_; }

double vm_get_x(SimModel * m)     { return m->getX(); }
double vm_get_y(SimModel * m)     { return m->getY(); }
double vm_get_yaw(SimModel * m)   { return m->getYaw(); }
double vm_get_vx(SimModel * m)    { return m->getVx(); }
double vm_get_steer(SimModel * m) { return m->getSteer(); }
double vm_get_ax(SimModel * m)    { return m->getAx(); }

void vm_destroy(SimModel * m) { delete m; }

} // extern "C"
