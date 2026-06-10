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

#ifndef AUTOWARE__SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_TAIGA_X_HPP_  // NOLINT
#define AUTOWARE__SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_TAIGA_X_HPP_  // NOLINT

#include <Eigen/Core>
#include <memory>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_interface.hpp>

namespace autoware::simulator::simple_planning_simulator
{

/**
 * @class SimModelTaigaX
 * @brief High-fidelity rigid-body + tire vehicle model backed by a real-time
 *        physics engine running on a flat ground plane.
 *
 * The engine integrates the chassis rigid body, suspension, and tire forces at
 * a fixed internal timestep; each call to update(dt) advances the engine by
 * ceil(dt / fixed_dt) substeps. Unlike the analytical models in this package,
 * lateral velocity, yaw rate, wheel spin, and suspension load are emergent
 * states of the physics solver rather than closed-form expressions.
 *
 * The physics dependency is isolated behind a PIMPL so that translation units
 * which merely include this header do not need the physics SDK on the include
 * path; only the implementation file pulls in the SDK. The SDK itself is
 * provided by the physx_vendor package (see CMakeLists.txt).
 *
 * The SimModelInterface state vector is a 6-element mirror
 * [X, Y, YAW, VX, VY, WZ] kept in sync with the engine; longitudinal
 * acceleration and steering angle are tracked separately.
 */
class SimModelTaigaX : public SimModelInterface
{
public:
  /**
   * @param [in] wheelbase vehicle wheelbase length [m]
   * @param [in] track_width vehicle track width [m]
   * @param [in] mass vehicle mass [kg]
   * @param [in] inertia_z yaw moment of inertia about the CG [kg·m²]
   * @param [in] cg_offset_x longitudinal CG offset from the geometric center [m]
   * @param [in] max_steer steering tire-angle limit [rad]
   * @param [in] max_accel maximum longitudinal acceleration used to map the
   *             acceleration command to throttle [m/s²]
   * @param [in] max_brake maximum longitudinal deceleration used to map the
   *             acceleration command to brake [m/s²]
   * @param [in] wheel_radius wheel radius [m]
   * @param [in] fixed_dt internal physics substep [s]
   */
  SimModelTaigaX(
    double wheelbase, double track_width, double mass, double inertia_z, double cg_offset_x,
    double max_steer, double max_accel, double max_brake, double wheel_radius, double fixed_dt);

  ~SimModelTaigaX() override;

  /**
   * @brief teleport the chassis and seed body-frame velocities without
   *        rebuilding the engine vehicle. Used for per-step replay reset and
   *        for the closed-loop external-pose overwrite path.
   */
  void setFullState(
    double x, double y, double yaw, double vx, double vy, double wz, double ax, double steer);

private:
  enum IDX { X = 0, Y, YAW, VX, VY, WZ };
  enum IDX_U { ACCX_DES = 0, GEAR, SLOPE_ACCX, STEER_DES };

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
   * @brief unused for this model: the physics engine performs its own
   *        integration, so this returns a zero derivative.
   */
  Eigen::VectorXd calcModel(const Eigen::VectorXd & state, const Eigen::VectorXd & input) override;

  /// @brief refresh the SimModelInterface state mirror from the engine.
  void syncStateFromEngine();

  struct Impl;
  std::unique_ptr<Impl> impl_;

  const double max_steer_;
  const double max_accel_;
  const double max_brake_;
  const double fixed_dt_;

  double ax_{0.0};      //!< @brief longitudinal acceleration [m/s²]
  double steer_{0.0};   //!< @brief steering tire angle [rad]
};

}  // namespace autoware::simulator::simple_planning_simulator

// NOLINTNEXTLINE
#endif  // AUTOWARE__SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_TAIGA_X_HPP_
