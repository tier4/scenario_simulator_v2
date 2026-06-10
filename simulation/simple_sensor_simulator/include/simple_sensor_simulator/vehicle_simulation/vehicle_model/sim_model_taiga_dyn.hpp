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

#ifndef AUTOWARE__SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_TAIGA_DYN_HPP_  // NOLINT
#define AUTOWARE__SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_TAIGA_DYN_HPP_  // NOLINT

#include <Eigen/Core>
#include <Eigen/LU>
#include <deque>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_interface.hpp>

namespace autoware::simulator::simple_planning_simulator
{

/**
 * @class SimModelTaigaDyn
 * @brief Dynamic-bicycle vehicle model with lateral tire dynamics.
 *
 * Unlike the kinematic models in this package (where the yaw rate is an
 * instantaneous algebraic function of vx and the steering angle), this model
 * carries the lateral velocity (vy) and the yaw rate (wz) as proper dynamic
 * states driven by linear front/rear cornering forces and yaw inertia:
 *
 *   d(vy) = -(Cf+Cr)/(m·vx)·vy + ((lr·Cr - lf·Cf)/(m·vx) - vx)·wz + Cf/m·δ
 *   d(wz) = (lr·Cr - lf·Cf)/(Iz·vx)·vy - (lf²·Cf + lr²·Cr)/(Iz·vx)·wz + lf·Cf/Iz·δ
 *
 * The longitudinal channel (acceleration first-order lag, gear handling and
 * the geared stop condition) and the steering first-order lag with input delay
 * queues mirror the geared longitudinal model used elsewhere in this package so
 * that the two share the same command interface and replay semantics.
 *
 * Because the lateral equations divide by vx, they are singular at standstill.
 * Below @p vx_min_dyn the model relaxes vy→0 and wz→ the kinematic yaw rate
 * vx·tan(δ)/L, which keeps stop-and-go segments numerically stable.
 */
class SimModelTaigaDyn : public SimModelInterface
{
public:
  /**
   * @param [in] vx_lim velocity limit [m/s]
   * @param [in] steer_lim steering limit [rad]
   * @param [in] vx_rate_lim acceleration limit [m/ss]
   * @param [in] steer_rate_lim steering angular velocity limit [rad/ss]
   * @param [in] wheelbase vehicle wheelbase length [m]
   * @param [in] dt delta time information to set input buffer for delay
   * @param [in] acc_delay time delay for accel command [s]
   * @param [in] acc_time_constant time constant for 1D model of accel dynamics
   * @param [in] steer_delay time delay for steering command [s]
   * @param [in] steer_time_constant time constant for 1D model of steering dynamics
   * @param [in] steer_dead_band dead band for steering angle [rad]
   * @param [in] steer_bias steering bias [rad]
   * @param [in] debug_acc_scaling_factor scaling factor for accel command
   * @param [in] debug_steer_scaling_factor scaling factor for steering command
   * @param [in] mass vehicle mass [kg]
   * @param [in] inertia_z yaw moment of inertia about the CG [kg·m²]
   * @param [in] lf distance from the CG to the front axle [m]
   * @param [in] lr distance from the CG to the rear axle [m]
   * @param [in] cornering_stiffness_front front-axle cornering stiffness [N/rad]
   * @param [in] cornering_stiffness_rear rear-axle cornering stiffness [N/rad]
   * @param [in] vx_min_dyn longitudinal speed below which the lateral channel
   *             falls back to the kinematic yaw rate to avoid the 1/vx singularity [m/s]
   */
  SimModelTaigaDyn(
    double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
    double dt, double acc_delay, double acc_time_constant, double steer_delay,
    double steer_time_constant, double steer_dead_band, double steer_bias,
    double debug_acc_scaling_factor, double debug_steer_scaling_factor, double mass,
    double inertia_z, double lf, double lr, double cornering_stiffness_front,
    double cornering_stiffness_rear, double vx_min_dyn);

  /**
   * @brief default destructor
   */
  ~SimModelTaigaDyn() = default;

  /**
   * @brief replace internal acc / steer delay queues with caller-supplied histories.
   *        Used by the C wrapper to inject actual past command history for
   *        per-step real-vs-sim replay.
   */
  void setInputQueues(
    const std::deque<double> & acc_queue, const std::deque<double> & steer_queue);

  /**
   * @brief sizes of the internal delay queues (driven by acc_delay / steer_delay / dt).
   */
  int getAccQueueSize() const;
  int getSteerQueueSize() const;

private:
  const double MIN_TIME_CONSTANT;  //!< @brief minimum time constant

  enum IDX {
    X = 0,
    Y,
    YAW,
    VX,
    STEER,
    ACCX,
    PEDAL_ACCX,
    VY,
    WZ,
  };
  enum IDX_U { PEDAL_ACCX_DES = 0, GEAR, SLOPE_ACCX, STEER_DES };

  const double vx_lim_;          //!< @brief velocity limit [m/s]
  const double vx_rate_lim_;     //!< @brief acceleration limit [m/ss]
  const double steer_lim_;       //!< @brief steering limit [rad]
  const double steer_rate_lim_;  //!< @brief steering angular velocity limit [rad/s]
  const double wheelbase_;       //!< @brief vehicle wheelbase length [m]

  std::deque<double> acc_input_queue_;       //!< @brief buffer for accel command
  std::deque<double> steer_input_queue_;     //!< @brief buffer for steering command
  const double acc_delay_;                   //!< @brief time delay for accel command [s]
  const double acc_time_constant_;           //!< @brief time constant for accel dynamics
  const double steer_delay_;                 //!< @brief time delay for steering command [s]
  const double steer_time_constant_;         //!< @brief time constant for steering dynamics
  const double steer_dead_band_;             //!< @brief dead band for steering angle [rad]
  const double steer_bias_;                  //!< @brief steering angle bias [rad]
  const double debug_acc_scaling_factor_;    //!< @brief scaling factor for accel command
  const double debug_steer_scaling_factor_;  //!< @brief scaling factor for steering command

  const double mass_;       //!< @brief vehicle mass [kg]
  const double inertia_z_;  //!< @brief yaw moment of inertia [kg·m²]
  const double lf_;         //!< @brief CG-to-front-axle distance [m]
  const double lr_;         //!< @brief CG-to-rear-axle distance [m]
  const double cf_;         //!< @brief front cornering stiffness [N/rad]
  const double cr_;         //!< @brief rear cornering stiffness [N/rad]
  const double vx_min_dyn_;  //!< @brief kinematic-fallback speed threshold [m/s]

  /**
   * @brief kinematic yaw rate vx·tan(δ)/L, used as the low-speed fallback target.
   */
  double calc_kinematic_yaw_rate(double vel, double steer) const;

  /**
   * @brief number of explicit-Euler substeps needed to integrate one outer dt
   *        stably. The lateral (vy, wz) block is stiff at low speed — its
   *        eigenvalues scale as ~(Cf+Cr)/(m·vx) and ~(lf²Cf+lr²Cr)/(Iz·vx), so a
   *        single SUB_DT step is unstable just above vx_min_dyn. We pick
   *        n = ceil(dt · λ_max) so the per-substep λ·h stays below ~1.
   */
  int lateral_substeps(double vel, double dt) const;

  /**
   * @brief set queue buffer for input command
   * @param [in] dt delta time
   */
  void initializeInputQueue(const double & dt);

  double getX() override;
  double getY() override;
  double getYaw() override;
  double getVx() override;
  double getVy() override;
  double getAx() override;
  double getWz() override;
  double getSteer() override;
  void update(const double & dt) override;

  /**
   * @brief calculate derivative of states with the dynamic-bicycle model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd & state, const Eigen::VectorXd & input) override;
};

}  // namespace autoware::simulator::simple_planning_simulator

// NOLINTNEXTLINE
#endif  // AUTOWARE__SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_TAIGA_DYN_HPP_
