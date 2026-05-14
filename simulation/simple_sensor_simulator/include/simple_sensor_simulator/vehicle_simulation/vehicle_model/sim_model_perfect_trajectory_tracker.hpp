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

#ifndef SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_PERFECT_TRAJECTORY_TRACKER_HPP_
#define SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_PERFECT_TRAJECTORY_TRACKER_HPP_

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <deque>
#include <eigen3/Eigen/Core>
#include <geometry_msgs/msg/pose.hpp>
#include <mutex>
#include <rclcpp/time.hpp>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model_interface.hpp>

/**
 * @class SimModelPerfectTrajectoryTracker
 * @brief Vehicle model that follows the planning trajectory exactly,
 *        without any controller or vehicle dynamics. Equivalent to
 *        autoware_perfect_tracker, reimplemented as a SimModelInterface
 *        derivative so it integrates with scenario_simulator's vehicle
 *        simulation framework.
 *
 * State: (x, y, yaw, vx) in model-relative frame
 * Input: none (trajectory is injected via setTrajectory())
 *
 * Extended API (not in SimModelInterface):
 *   setInitialReference() -- called once at construction
 *   setTrajectory()       -- called every simulation step
 *   getZ() / getPitch() / getRoll()
 */
class SimModelPerfectTrajectoryTracker : public SimModelInterface
{
public:
  explicit SimModelPerfectTrajectoryTracker(double delay_time_sec);

  /**
   * @brief Store the initial map-frame reference for coordinate conversion.
   *        Must be called once before the first update().
   */
  void setInitialReference(
    const geometry_msgs::msg::Pose & initial_pose, const Eigen::Matrix3d & initial_rotation_matrix);

  /**
   * @brief Inject a new trajectory into the delay queue.
   *        Thread-safe; may be called from a subscriber callback thread.
   *        Duplicate stamps are silently dropped.
   */
  void setTrajectory(
    const rclcpp::Time & stamp, const autoware_planning_msgs::msg::Trajectory & msg);

  // Extended getters (z/pitch not available in SimModelInterface)
  double getZ() const;
  double getPitch() const;
  double getRoll() const { return 0.0; }

  // SimModelInterface overrides
  double getX() override;
  double getY() override;
  double getYaw() override;
  double getVx() override;
  double getVy() override { return 0.0; }
  double getAx() override;
  double getWz() override;
  double getSteer() override;
  void update(const double & dt) override;
  Eigen::VectorXd calcModel(const Eigen::VectorXd & state, const Eigen::VectorXd & input) override;

private:
  enum IDX { X = 0, Y, YAW, VX };

  struct StampedTrajectory
  {
    rclcpp::Time stamp;
    autoware_planning_msgs::msg::Trajectory msg;
  };

  double interpolateZ(
    const autoware_planning_msgs::msg::Trajectory & traj, std::size_t hint_idx, double x, double y,
    double fallback_z) const;

  double calculatePitchFromLanelet(double x, double y, double yaw) const;

  const double delay_time_sec_;

  mutable std::mutex mutex_;
  std::deque<StampedTrajectory> trajectory_queue_;
  static constexpr std::size_t kQueueMax = 100;

  geometry_msgs::msg::Pose initial_pose_{};
  Eigen::Matrix3d initial_rotation_matrix_{Eigen::Matrix3d::Identity()};
  bool reference_initialized_{false};
  double initial_yaw_{0.0};

  // Derived quantities updated in update()
  double current_ax_{0.0};
  double current_wz_{0.0};
  double current_steer_{0.0};
  double current_z_map_{0.0};
  double current_pitch_{0.0};

  // z-component of position in initial frame, updated each cycle to maintain exact R^T/R roundtrip.
  // Using z=0 in the roundtrip introduces a per-step decay of sin²(pitch)*state_X in map-frame x.
  double state_z_initial_frame_{0.0};
};

#endif  // SIMPLE_PLANNING_SIMULATOR__VEHICLE_MODEL__SIM_MODEL_PERFECT_TRAJECTORY_TRACKER_HPP_
