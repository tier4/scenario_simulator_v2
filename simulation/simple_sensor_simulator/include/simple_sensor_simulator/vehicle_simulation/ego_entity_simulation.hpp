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

#ifndef TRAFFIC_SIMULATOR__VEHICLE_SIMULATION__EGO_ENTITY_SIMULATION_HPP_
#define TRAFFIC_SIMULATOR__VEHICLE_SIMULATION__EGO_ENTITY_SIMULATION_HPP_

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <concealer/autoware_universe.hpp>
#include <diffusion_planner_lockstep_msgs/srv/plan_trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model.hpp>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>

namespace vehicle_simulation
{
/// Per-frame planner inputs assembled by ScenarioSimulator (which owns the
/// ground-truth entity states) for the lockstep diffusion_planner service.
struct LockstepPlannerInput
{
  rclcpp::Time ros_time;
  autoware_perception_msgs::msg::TrackedObjects tracked_objects;
  std::optional<autoware_perception_msgs::msg::TrafficLightGroupArray> traffic_signals;
};

enum class VehicleModelType {
  DELAY_STEER_ACC,
  DELAY_STEER_ACC_GEARED,
  DELAY_STEER_ACC_GEARED_WO_FALL_GUARD,
  DELAY_STEER_MAP_ACC_GEARED,
  DELAY_STEER_VEL,
  EXTERNAL,
  EXTERNAL_PERFECT_TRAJECTORY_TRACKER,
  IDEAL_STEER_ACC,
  IDEAL_STEER_ACC_GEARED,
  IDEAL_STEER_VEL,
  PERFECT_TRAJECTORY_TRACKER,
};

class EgoEntitySimulation
{
public:
  const std::unique_ptr<concealer::AutowareUniverse> autoware;

private:
  const VehicleModelType vehicle_model_type_;

  const double wheel_base_;

  const std::shared_ptr<SimModelInterface> vehicle_model_ptr_;

  std::optional<double> previous_linear_velocity_, previous_angular_velocity_;

  traffic_simulator::CanonicalizedEntityStatus status_;

  const geometry_msgs::msg::Pose initial_pose_;

  const Eigen::Matrix3d initial_rotation_matrix_;

  static auto getVehicleModelType() -> VehicleModelType;

  static auto makeSimulationModel(
    const VehicleModelType, const double step_time,
    const traffic_simulator_msgs::msg::VehicleParameters &)
    -> const std::shared_ptr<SimModelInterface>;

  const bool consider_acceleration_by_road_slope_;

  Eigen::Vector3d world_relative_position_;

public:
  const traffic_simulator_msgs::msg::VehicleParameters vehicle_parameters;

  auto calculateAccelerationBySlope() const -> double;

private:
  SimModelExternal * external_model_ = nullptr;

  SimModelPerfectTrajectoryTracker * perfect_tracker_model_ = nullptr;

  auto initializePerfectTrajectoryFollowerMode() -> void;

  // Lockstep diffusion_planner invocation (PERFECT_TRAJECTORY_TRACKER only).
  // The client lives on the concealer node whose dedicated spinner thread
  // processes the response while the ZMQ thread blocks on future.wait_for().
  // The initializers below are the parameter defaults; see
  // initializePerfectTrajectoryFollowerMode.
  bool lockstep_enabled_ = false;

  double lockstep_planner_period_ = 0.1;

  double lockstep_service_timeout_sec_ = 60.0;

  double next_planner_call_time_ = 0.0;

  bool lockstep_received_first_trajectory_ = false;

  rclcpp::Client<diffusion_planner_lockstep_msgs::srv::PlanTrajectory>::SharedPtr plan_client_;

  autoware_vehicle_msgs::msg::TurnIndicatorsCommand last_turn_indicators_command_;

  auto callPlannerService(LockstepPlannerInput && input, const double step_time) -> void;

  std::optional<concealer::Subscriber<nav_msgs::msg::Odometry>> ego_odometry_sub_;

  std::optional<concealer::Subscriber<geometry_msgs::msg::AccelWithCovarianceStamped>>
    ego_accel_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    external_initial_pose_pub_;

  geometry_msgs::msg::PoseWithCovarianceStamped external_initial_pose_msg_;

  bool godot_ready_ = false;

  auto initializeExternalMode() -> void;

  auto onKinematicState(const nav_msgs::msg::Odometry & msg) -> void;

  auto getCurrentPose(const double pitch_angle = 0.0) const -> geometry_msgs::msg::Pose;

  auto getCurrentTwist() const -> geometry_msgs::msg::Twist;

  auto getCurrentAccel(const double step_time) const -> geometry_msgs::msg::Accel;

  auto getLinearJerk(double step_time) -> double;

  auto updatePreviousValues() -> void;

public:
  auto setAutowareStatus() -> void;

  explicit EgoEntitySimulation(
    const traffic_simulator_msgs::msg::EntityStatus &,
    const traffic_simulator_msgs::msg::VehicleParameters &, double,
    const rclcpp::Parameter & use_sim_time, const bool consider_acceleration_by_road_slope);

  auto overwrite(
    const traffic_simulator_msgs::msg::EntityStatus & status, const double current_time,
    const double step_time, bool is_npc_logic_started) -> void;

  auto update(
    const double current_time, const double step_time, const bool is_npc_logic_started,
    std::optional<LockstepPlannerInput> lockstep_input) -> void;

  // True when the next call to update(current_time, ...) will invoke the
  // lockstep planner service, i.e. when a LockstepPlannerInput must be
  // assembled for that frame.
  auto isLockstepPlannerCallDue(const double current_time, const double step_time) const -> bool;

  auto requestSpeedChange(double value) -> void;

  auto getStatus() const -> const traffic_simulator_msgs::msg::EntityStatus;

  auto setStatus(const traffic_simulator_msgs::msg::EntityStatus & status) -> void;

  auto updateStatus(const double current_time, const double step_time) -> void;
};
}  // namespace vehicle_simulation

#endif  // TRAFFIC_SIMULATOR__VEHICLE_SIMULATION__EGO_ENTITY_SIMULATION_HPP_
