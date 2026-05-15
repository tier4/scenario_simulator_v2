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

#ifndef AUTOWARE__SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DELAY_STEER_ACC_GEARED_WO_FALL_GUARD_HPP_  // NOLINT
#define AUTOWARE__SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DELAY_STEER_ACC_GEARED_WO_FALL_GUARD_HPP_  // NOLINT

#include <Eigen/Core>
#include <Eigen/LU>
#include <deque>
#include <iostream>
#include <queue>
#include <random>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_interface.hpp>

namespace autoware::simulator::simple_planning_simulator
{

class SimModelDelaySteerAccGearedWoFallGuard : public SimModelInterface
{
public:
  /**
   * @param [in] vx_lim velocity limit [m/s]
   * @param [in] acc_lim maximum acceleration limit [m/ss]
   * @param [in] brake_lim maximum deceleration limit [m/ss]
   * @param [in] acc_rate_lim acceleration jerk limit [m/s^3]
   * @param [in] brake_rate_lim deceleration jerk limit [m/s^3]
   * @param [in] steer_lim steering limit [rad]
   * @param [in] steer_rate_lim steering angular velocity limit [rad/ss]
   * @param [in] wheelbase vehicle wheelbase length [m]
   * @param [in] dt delta time information to set input buffer for delay
   * @param [in] acc_delay time delay for accel command [s]
   * @param [in] brake_delay time delay for brake command [s]
   * @param [in] acc_time_constant time constant for 1D model of accel dynamics
   * @param [in] brake_time_constant time constant for 1D model of brake dynamics
   * @param [in] acc_accuracy_error accuracy error gain for acc command (e.g. 0.05 for +5%)
   * @param [in] brake_accuracy_error accuracy error gain for brake command (e.g. 0.05 for +5%)
   * @param [in] brake_hysteresis_width hysteresis width for brake command [m/ss]
   * @param [in] acc_dead_band dead band for acc [m/ss]
   * @param [in] brake_dead_band dead band for brake [m/ss]
   * @param [in] brake_jump_value minimum output value when brake is activated [m/ss]
   * @param [in] acc_offset constant offset for acc [m/ss]
   * @param [in] brake_offset constant offset for brake [m/ss]
   * @param [in] acc_resolution resolution (step size) for acc command [m/ss]
   * @param [in] brake_resolution resolution (step size) for brake command [m/ss]
   * @param [in] steer_delay time delay for steering command [s]
   * @param [in] steer_time_constant time constant for 1D model of steering dynamics
   * @param [in] steer_dead_band dead band for steering angle [rad]
   * @param [in] steer_bias steering bias [rad]
   * @param [in] steer_accuracy_error accuracy error gain for steering (e.g. 0.05 for +5%)
   * @param [in] steer_resolution resolution (step size) for steering angle [rad]
   * @param [in] steer_hysteresis_width hysteresis width (backlash) for steering angle [rad]
   * @param [in] vel_sensor_delay time delay for velocity sensor [s]
   * @param [in] vel_sensor_resolution resolution (step size) for velocity sensor [m/s]
   * @param [in] vel_sensor_noise_stddev standard deviation of velocity sensor noise [m/s]
   * @param [in] vel_sensor_noise_seed random seed for velocity sensor noise to ensure reproducibility
   * @param [in] vel_sensor_accuracy_error accuracy error gain for velocity sensor (e.g. 0.03 for +3%)
   * @param [in] vel_sensor_offset constant offset for velocity sensor [m/s]
   * @param [in] debug_acc_scaling_factor scaling factor for accel command
   * @param [in] debug_steer_scaling_factor scaling factor for steering command
   */
  SimModelDelaySteerAccGearedWoFallGuard(
    double vx_lim, double acc_lim, double brake_lim, double acc_rate_lim, double brake_rate_lim, double steer_lim, double steer_rate_lim, double wheelbase,
    double dt, double acc_delay, double brake_delay, double acc_time_constant, double brake_time_constant,
    double acc_accuracy_error, double brake_accuracy_error, double brake_hysteresis_width, double acc_dead_band, double brake_dead_band, double brake_jump_value, double acc_offset, double brake_offset, double acc_resolution, double brake_resolution,
    double steer_delay, double steer_time_constant, double steer_dead_band, double steer_bias,
    double steer_accuracy_error, double steer_resolution, double steer_hysteresis_width,
    double vel_sensor_delay, double vel_sensor_resolution, double vel_sensor_noise_stddev, int vel_sensor_noise_seed, double vel_sensor_accuracy_error, double vel_sensor_offset,
    double debug_acc_scaling_factor, double debug_steer_scaling_factor);

  /**
   * @brief default destructor
   */
  ~SimModelDelaySteerAccGearedWoFallGuard() = default;

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
  };
  enum IDX_U { PEDAL_ACCX_DES = 0, GEAR, SLOPE_ACCX, STEER_DES };

  const double vx_lim_;          //!< @brief velocity limit [m/s]
  const double acc_lim_;
  const double brake_lim_;
  const double acc_rate_lim_;
  const double brake_rate_lim_;
  const double steer_lim_;       //!< @brief steering limit [rad]
  const double steer_rate_lim_;  //!< @brief steering angular velocity limit [rad/s]
  const double wheelbase_;       //!< @brief vehicle wheelbase length [m]

  std::deque<double> acc_input_queue_;       //!< @brief buffer for accel command
  std::deque<double> brake_input_queue_;     //!< @brief buffer for brake command
  std::deque<double> steer_input_queue_;     //!< @brief buffer for steering command
  const double acc_delay_;                   //!< @brief time delay for accel command [s]
  const double brake_delay_;                 //!< @brief time delay for brake command [s]
  const double acc_time_constant_;           //!< @brief time constant for accel dynamics
  const double brake_time_constant_;         //!< @brief time constant for brake dynamics
  const double acc_accuracy_error_;
  const double brake_accuracy_error_;
  const double brake_hysteresis_width_;
  const double acc_dead_band_;
  const double brake_dead_band_;
  const double brake_jump_value_;
  const double acc_offset_;
  const double brake_offset_;
  const double acc_resolution_;
  const double brake_resolution_;
  const double steer_delay_;                 //!< @brief time delay for steering command [s]
  const double steer_time_constant_;         //!< @brief time constant for steering dynamics
  const double steer_dead_band_;             //!< @brief dead band for steering angle [rad]
  const double steer_bias_;                  //!< @brief steering angle bias [rad]
  const double steer_accuracy_error_;
  const double steer_resolution_;
  const double steer_hysteresis_width_;
  const double vel_sensor_delay_;
  const double vel_sensor_resolution_;
  const double vel_sensor_noise_stddev_;
  const double vel_sensor_accuracy_error_;
  const double vel_sensor_offset_;
  const double debug_acc_scaling_factor_;    //!< @brief scaling factor for accel command
  const double debug_steer_scaling_factor_;  //!< @brief scaling factor for steering command

  double prev_brake_cmd_; // ヒステリシス計算用に前回のブレーキ指令値を記憶する変数
  double prev_steer_cmd_;

  std::deque<double> vel_history_queue_;       // 車速の遅延用バッファ
  double delayed_vx_;                          // 遅延適用後の物理車速
  std::mt19937 vel_rng_;                       // 乱数生成器
  std::normal_distribution<double> vel_dist_;  // 正規分布

  /**
   * @brief set queue buffer for input command
   * @param [in] dt delta time
   */
  void initializeInputQueue(const double & dt);

  /**
   * @brief get vehicle position x
   */
  double getX() override;

  /**
   * @brief get vehicle position y
   */
  double getY() override;

  /**
   * @brief get vehicle angle yaw
   */
  double getYaw() override;

  /**
   * @brief get vehicle velocity vx
   */
  double getVx() override;

  /**
   * @brief get vehicle lateral velocity
   */
  double getVy() override;

  /**
   * @brief get vehicle longitudinal acceleration
   */
  double getAx() override;

  /**
   * @brief get vehicle angular-velocity wz
   */
  double getWz() override;

  /**
   * @brief get vehicle steering angle
   */
  double getSteer() override;

  /**
   * @brief update vehicle states
   * @param [in] dt delta time [s]
   */
  void update(const double & dt) override;

  /**
   * @brief calculate derivative of states with time delay steering model
   * @param [in] state current model state
   * @param [in] input input vector to model
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd & state, const Eigen::VectorXd & input) override;
};

}  // namespace autoware::simulator::simple_planning_simulator

// NOLINTNEXTLINE
#endif  // AUTOWARE__SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_DELAY_STEER_ACC_GEARED_WO_FALL_GUARD_HPP_
