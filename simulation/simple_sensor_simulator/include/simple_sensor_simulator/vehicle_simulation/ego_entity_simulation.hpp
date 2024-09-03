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

#include <concealer/autoware.hpp>
#include <memory>
#include <simple_sensor_simulator/vehicle_simulation/vehicle_model/sim_model.hpp>
#include <traffic_simulator/data_type/entity_status.hpp>
#include <traffic_simulator/data_type/lanelet_pose.hpp>
#include <traffic_simulator/hdmap_utils/hdmap_utils.hpp>
#include <traffic_simulator_msgs/msg/entity_status.hpp>
#include <traffic_simulator_msgs/msg/polyline_trajectory.hpp>
#include <traffic_simulator_msgs/msg/vehicle_parameters.hpp>

namespace vehicle_simulation
{
enum class VehicleModelType {
  DELAY_STEER_ACC,
  DELAY_STEER_ACC_GEARED,
  DELAY_STEER_MAP_ACC_GEARED,
  DELAY_STEER_VEL,
  IDEAL_STEER_ACC,
  IDEAL_STEER_ACC_GEARED,
  IDEAL_STEER_VEL,
};

class EgoEntitySimulation
{
public:
  const std::unique_ptr<concealer::Autoware> autoware;

private:
  const VehicleModelType vehicle_model_type_;

  const std::shared_ptr<SimModelInterface> vehicle_model_ptr_;

  std::optional<double> previous_linear_velocity_, previous_angular_velocity_;

  traffic_simulator::CanonicalizedEntityStatus status_;

  geometry_msgs::msg::Pose initial_pose_;

  static auto getVehicleModelType() -> VehicleModelType;

  static auto makeSimulationModel(
    const VehicleModelType, const double step_time,
    const traffic_simulator_msgs::msg::VehicleParameters &)
    -> const std::shared_ptr<SimModelInterface>;

  const bool consider_acceleration_by_road_slope_;

  Eigen::Vector3d world_relative_position_;

public:
  const std::shared_ptr<hdmap_utils::HdMapUtils> hdmap_utils_ptr_;

  const traffic_simulator_msgs::msg::VehicleParameters vehicle_parameters;

private:
  auto calculateEgoPitch() const -> double;

  auto getCurrentPose(const double pitch_angle) const -> geometry_msgs::msg::Pose;

  auto getCurrentTwist() const -> geometry_msgs::msg::Twist;

  auto getCurrentAccel(const double step_time) const -> geometry_msgs::msg::Accel;

  auto getLinearJerk(double step_time) -> double;

  auto updatePreviousValues() -> void;

public:
  auto setAutowareStatus() -> void;

  explicit EgoEntitySimulation(
    const traffic_simulator_msgs::msg::EntityStatus &,
    const traffic_simulator_msgs::msg::VehicleParameters &, double,
    const std::shared_ptr<hdmap_utils::HdMapUtils> &, const rclcpp::Parameter & use_sim_time,
    const bool consider_acceleration_by_road_slope);

  auto overwrite(
    const traffic_simulator_msgs::msg::EntityStatus & status, const double current_time,
    const double step_time, bool is_npc_logic_started) -> void;

  auto update(const double current_time, const double step_time, const bool is_npc_logic_started)
    -> void;

  auto requestSpeedChange(double value) -> void;

  auto getStatus() const -> const traffic_simulator_msgs::msg::EntityStatus;

  auto setStatus(const traffic_simulator_msgs::msg::EntityStatus & status) -> void;

  auto updateStatus(const double current_time, const double step_time) -> void;
};
}  // namespace vehicle_simulation

#endif  // TRAFFIC_SIMULATOR__VEHICLE_SIMULATION__EGO_ENTITY_SIMULATION_HPP_
