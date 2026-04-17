// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#ifndef SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_EXTERNAL_HPP_
#define SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_EXTERNAL_HPP_

#include <mutex>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_interface.hpp>

/**
 * @class SimModelExternal
 * @brief Vehicle model that delegates dynamics computation to an external simulator.
 *
 * Instead of computing vehicle dynamics internally, this model receives the vehicle
 * state from an external simulator (e.g., Godot) via ROS 2 topic subscriptions.
 * The update() method is a no-op; state is injected via setExternalState().
 */
class SimModelExternal : public SimModelInterface
{
public:
  /**
   * @brief constructor
   * State dimension: 3 (x, y, yaw). Input dimension: 0 (no internal inputs).
   */
  SimModelExternal()
  : SimModelInterface(3 /* dim_x: x, y, yaw */, 0 /* dim_u: no internal inputs */)
  {
  }

  ~SimModelExternal() = default;

  /**
   * @brief Inject ego state from the external simulator.
   * Called from the ROS 2 subscription callback (spinner thread).
   * Thread-safe via mutex.
   * @param x   position x in model-relative frame [m]
   * @param y   position y in model-relative frame [m]
   * @param yaw heading angle in model-relative frame [rad]
   * @param vx  longitudinal velocity [m/s]
   * @param vy  lateral velocity [m/s]
   * @param ax  longitudinal acceleration [m/s^2]
   * @param wz  yaw rate [rad/s]
   * @param steer steering tire angle [rad]
   */
  void setExternalState(
    double x, double y, double yaw, double vx, double vy, double ax, double wz, double steer)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    external_state_.x = x;
    external_state_.y = y;
    external_state_.yaw = yaw;
    external_state_.vx = vx;
    external_state_.vy = vy;
    external_state_.ax = ax;
    external_state_.wz = wz;
    external_state_.steer = steer;
  }

  double getX() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return external_state_.x;
  }

  double getY() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return external_state_.y;
  }

  double getYaw() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return external_state_.yaw;
  }

  double getVx() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return external_state_.vx;
  }

  double getVy() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return external_state_.vy;
  }

  double getAx() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return external_state_.ax;
  }

  double getWz() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return external_state_.wz;
  }

  double getSteer() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return external_state_.steer;
  }

  /**
   * @brief No-op: external simulator drives state updates via setExternalState().
   */
  void update(const double & /*dt*/) override {}

  /**
   * @brief Returns zero vector: Runge-Kutta integration is not used for external model.
   */
  Eigen::VectorXd calcModel(
    const Eigen::VectorXd & /*state*/, const Eigen::VectorXd & /*input*/) override
  {
    return Eigen::VectorXd::Zero(3);
  }

private:
  mutable std::mutex mutex_;

  struct ExternalState
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double ax = 0.0;
    double wz = 0.0;
    double steer = 0.0;
  } external_state_;
};

#endif  // SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_EXTERNAL_HPP_
